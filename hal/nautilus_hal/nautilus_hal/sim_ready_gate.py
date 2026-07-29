"""Sim bringup gate: hold the (paused-spawned) world until the graph is whole.

The v2 sweep campaign lost 4 of 5 runs to two init races that only bite
under heavy parallel load:

- the model was spawned into a *running* world, free-falling for however
  long node/bridge bringup took (bags opening at 30+ m, watchdog
  misreads), or never being picked up by physics at all (runs frozen at
  the spawn pose);
- Fast DDS endpoint matching under a 16-slot startup stampede left
  individual readers dead or minutes late (plant commands ignored,
  recorded channels empty), invisibly, while every process looked alive.

The v3 campaign then showed a third race that both of those fixes pass:
30 % of 64-slot launches came up with the buoyancy plugin never applying
commanded volume as *force* — graph whole, spawn complete, physics
stepping (odometry and volume streams publishing, tank telemetry moving),
hull pinned at its ~0.44 m float depth with depth std ~ 0 for the whole
run. Frozen and clean runs have statistically identical scenarios
(multivariate AUC 0.538): it is a per-launch init coin flip under load.
Hull motion in response to a commanded volume change is the only signal
that discriminates it — the volume echo can track the setpoint while the
force path is dead.

This node closes all three by construction. The sim launches spawn
Gazebo paused and run this gate, which:

1. waits until every required ROS node is discovered AND the wire that
   each failure mode used to break is graph-visible:
   - >= 1 subscriber on the buoyancy-engine command topic (the ros_gz
     parameter_bridge's ROS->gz reader -- the exact link whose death
     froze v2 runs),
   - >= 1 publisher on the ground-truth odometry topic (the gz->ROS
     side of the same bridge),
   - a publisher AND a subscriber on /bcu/rpm (controller -> BCU bridge
     command path),
   - >= 1 subscriber on each recorder sentinel topic (the rosbag
     recorder is attached) when recording,
   - the model exists in the gz world (``gz model --list``): the spawn
     completed — and every system Configured on it — while still
     paused, so unpausing can never race the spawn itself;
2. only then unpauses the world (``gz service .../control``, retried)
   and verifies physics is actually stepping by waiting for the first
   odometry AND buoyancy-volume-state messages;
3. runs the physics-liveness probe (``probe_enabled`` — opt-in: the
   launches default it off and run_sweep injects ``physics_probe:=true``
   into every sweep run): commands a bladder deflate big enough to beat
   the reserve buoyancy and requires the hull to actually sink, then
   requires the plant to restore itself and hold before continuing. See
   _probe_step for the phase-by-phase contract;
4. publishes one latched ``/sim/ready`` (UUVTopics.SIM_READY) and stays
   alive to hold the latch -- ``auto_mission`` gates the mission start
   on it, so a run can never begin against a half-built graph, a
   falling vehicle, or a force-dead buoyancy plugin.

If readiness is not reached within ``ready_timeout_s`` (or a probe phase
misses its deadline) the gate writes ``run_verdict.json`` (verdict
``abort_init``, plus what was missing and the probe telemetry) and exits
nonzero; the sim launches turn that exit into a full launch Shutdown,
and the sweep runner retries the run instead of letting it burn its
whole wall budget recording garbage.

Probe design constraints, so nobody re-derives them wrong:

- The bcu_sim_bridge publishes a CONSTANT spawn setpoint at 10 Hz on the
  same buoyancy command topic once synced (its integrator seeds once from
  Gazebo's first volume echo and never re-reads it). The probe therefore
  contests the topic at ``probe_pub_rate_hz`` >= 5x the bridge rate so
  the plugin's last-received-target mostly follows the probe — and the
  restore phase is simply the gate going silent: the bridge's republish
  pulls the plugin back to the spawn volume on its own. Both facts are
  load-bearing; a bridge that re-synced from the echo, or a scenario
  raising rig.bridge.publish_rate_hz toward the probe rate, breaks them.
- At the surface float, waterplane stiffness turns small volume deltas
  into millimetre draft changes. ``probe_delta_m3`` must exceed the
  reserve buoyancy (spawn-minus-neutral volume, ~3.5e-4 m3 nominal,
  ~4.45e-4 at the worst sampled corner) so the hull actually sinks.
- The vehicle spawns shallow (z=-0.115) and passively settles ~0.3 m down
  to its waterplane equilibrium — in frozen runs too (that is how they
  end up pinned at 0.44 m). A baseline taken during that settle would let
  the settle's own motion satisfy the response threshold and pass frozen
  runs, so BASELINE completes only once the trailing z window is
  quiescent (peak-to-peak under probe_quiescence_ptp_m): after that, any
  probe_dz_m sink can only be commanded.
- The commanded sink is indistinguishable from a leg entry to the
  HeaveAugmentPlugin entry-momentum servo (armed at the float: 0.44 m >=
  trigger_depth 0.3, sink >= trigger_speed 0.03), and its 60 N would bury
  the ~2-4 N restore. The gate suppresses the trigger over gz-transport
  for the probe's duration; a lost suppress message fails safe (restore
  times out -> abort_init -> retry).
- Probe phases are timed on odometry-stamp SIM time, not wall time: under
  sweep load RTF runs 0.3-0.7 and wall deadlines would false-fail healthy
  runs whose physics is merely slow. Only ``ready_timeout_s`` stays
  wall-clock — it is the contract with run_sweep's bringup budget.
- Everything upstream of the probe stays untouched by it: fault schedules
  arm on the first /command, the watchdog arms on /command, auto_mission
  waits for /sim/ready — so the probe sees a healthy, uncommanded plant
  and is invisible to every downstream consumer. The ~0.3 m pre-mission
  dip lands under every offline classifier threshold (frozen: peak < 1 m,
  deep_start: median of the first samples >= 2 m).

Sim-only by definition -- nothing here exists on hardware.
"""

import json
import statistics
import subprocess
import sys
from collections import deque
from pathlib import Path

import rclpy
from py_pkg.scenarios.spec.rig import PlantSpec, SimSpec
from py_pkg.uuv_ros_core import (
    UUVQoS,
    UUVTopics,
    create_publisher_for_topic,
    now_s,
    spin_node,
)
from rclpy.node import Node
from std_msgs.msg import Bool, Float64
from nav_msgs.msg import Odometry

from .constants import SimTopics

EXIT_ABORT_INIT = 5

# The unpause service call; `gz service --timeout 2000` self-limits to
# ~2 s, so this is only the backstop against a wedged transport.
_GZ_SERVICE_TIMEOUT_S = 5.0
# The spawn probe. `gz model --list` waits on /gazebo/worlds with its own
# ~5.3 s internal timeout, i.e. LONGER than we are willing to block the
# executor: while Gazebo is still loading the world every probe would
# otherwise burn 5 s of a single-threaded spin. Cap it well short so a
# not-yet-ready sim costs a cheap miss instead of a stalled poll.
_GZ_MODEL_LIST_TIMEOUT_S = 2.0
# Unpause is idempotent but not free (one `gz` ruby fork per call). Once
# it succeeds we stop asking; until then we retry on a slow cadence with
# a hard cap, rather than re-forking on every poll for the whole timeout.
_UNPAUSE_RETRY_PERIOD_S = 5.0
_MAX_UNPAUSE_ATTEMPTS = 8
# The suppress publish; like unpause it forks the `gz` ruby CLI, so the
# `true` re-sends (papering over subscriber-discovery lag) are capped.
_GZ_TOPIC_TIMEOUT_S = 5.0
_MAX_SUPPRESS_SENDS = 5

# Physics-liveness probe phases (see _probe_step). Module constants so the
# verdict JSON, the failure labels, and the tests all spell them the same.
PROBE_BASELINE = "baseline"
PROBE_DEFLATE = "deflate"
PROBE_RESTORE = "restore"

# Probe numbers. These ARE the contract (nothing threads them per-launch;
# only probe_enabled and the two scenario-coupled knobs below are ROS
# params) — change them here, with the module docstring's physics.
#
# Deflate size: must beat the worst sampled reserve buoyancy (4.45e-4 m3)
# with margin so the hull demonstrably sinks; the scenario-derived floor
# in probe_target_m3 keeps the excursion inside the bridge's interval.
_PROBE_DELTA_M3 = 6.5e-4
# Baseline: minimum sampling period, the quiescence window/threshold that
# waits out the passive spawn settle, and the deadline bounding the wait.
_PROBE_BASELINE_MIN_S = 3.0
_PROBE_QUIESCENCE_S = 2.0
_PROBE_QUIESCENCE_PTP_M = 0.03
_PROBE_BASELINE_DEADLINE_S = 30.0
# Deflate: required sink and its deadline (~8 s contested pump-down at
# the plugin's 1e-4 m3/s against the bridge republish + sink + margin).
_PROBE_DZ_M = 0.10
_PROBE_DEADLINE_S = 25.0
# Restore: volume/z proximity to baseline, how long both must HOLD
# (stability proof before releasing the entry servo), and the deadline
# (~6.5 s uncontested re-inflate + refloat + margin).
_RESTORE_VOL_EPS_M3 = 5.0e-5
_RESTORE_Z_EPS_M = 0.10
_RESTORE_HOLD_S = 3.0
_RESTORE_DEADLINE_S = 40.0
# Minimum odometry samples a baseline may be computed from: 100 Hz
# odometry makes this trivially available unless the stream is stalled —
# in which case the global timeout reaps us. The buffer bound only needs
# to cover the quiescence window (2 s @ 100 Hz) with margin.
_PROBE_MIN_BASELINE_SAMPLES = 10
_PROBE_BASELINE_BUFFER = 256


def quiescent_baseline_z(
    samples,
    t_now: float,
    window_s: float,
    ptp_max_m: float,
):
    """Median float z of the trailing window, or None while not quiescent.

    Guards the probe against the passive spawn settle (module docstring):
    quiescence — peak-to-peak of the trailing ``window_s`` of z under
    ``ptp_max_m`` — is the settle-complete proof that makes a subsequent
    sink attributable to the commanded volume alone.
    """
    zs = [z for t, z in samples if t >= t_now - window_s]
    if len(zs) < _PROBE_MIN_BASELINE_SAMPLES or (max(zs) - min(zs)) > ptp_max_m:
        return None
    return statistics.median(zs)


def probe_target_m3(baseline_m3: float, delta_m3: float, floor_m3: float) -> float:
    """Deflate set point: baseline minus the probe delta, floor-clamped.

    The floor (the scenario's plant bladder_min) keeps the excursion
    inside the bridge's operating interval so the synthesized tank
    telemetry never leaves its calibrated range while the probe holds the
    plant off-spawn.
    """
    return max(floor_m3, baseline_m3 - delta_m3)


def hull_responded(baseline_z: float, z_now: float, dz_min_m: float) -> bool:
    """Did the hull sink at least dz_min_m below its float baseline?

    Odometry z is negative-down (raw gz world frame), so sinking means z
    DECREASING; the response is baseline_z - z_now, positive while sunk.
    """
    return (baseline_z - z_now) >= dz_min_m


def within_eps(a: float, b: float, eps: float) -> bool:
    """|a - b| <= eps — the restore checks (volume echo back at the spawn
    baseline; hull z back at the float, either side — it may bob)."""
    return abs(a - b) <= eps


def probe_failure_label(
    phase: str,
    dz_m: float,
    threshold_m: float,
    elapsed_s: float,
    baseline_vol_m3: float,
    vol_now_m3: float,
) -> str:
    """One-line failure detail for the abort_init `missing` list.

    Carries the measured hull response AND the volume-echo excursion: a
    dead force path with a tracking echo (dz ~ 0, vol moved) is the v3
    frozen signature; dz ~ 0 with a frozen echo means the command never
    even reached the plugin (the v2 bridge-reader death). ``threshold_m``
    is the sink requirement in DEFLATE and the z epsilon in RESTORE.
    """
    vols = f"vol {baseline_vol_m3:.4e}->{vol_now_m3:.4e}"
    if phase == PROBE_DEFLATE:
        return (
            f"physics:no-hull-response(dz={dz_m:.3f}m<{threshold_m:.3f}m "
            f"in {elapsed_s:.0f}s; {vols})"
        )
    return (
        f"physics:no-restore(dz={dz_m:.3f}m vs eps={threshold_m:.3f}m "
        f"in {elapsed_s:.0f}s; {vols})"
    )


def missing_requirements(
    required_nodes: list[str],
    live_nodes: set[str],
    endpoint_checks: list[tuple[str, int]],
) -> list[str]:
    """Pure readiness predicate: what is still missing from the graph.

    ``endpoint_checks`` is a list of (label, count) pairs where count is
    the currently observed number of endpoints; a count of 0 reports the
    label. Returns a sorted human-readable list, empty when ready.
    """
    missing = [f"node:{n}" for n in required_nodes if n not in live_nodes]
    missing += [label for label, count in endpoint_checks if count < 1]
    return sorted(missing)


class SimReadyGate(Node):
    def __init__(self, *, exit_fn=sys.exit) -> None:
        super().__init__("sim_ready_gate")

        # The gz WORLD name (the SDF <world name=...>), NOT the .world
        # filename: dave_ocean_waves.world declares <world
        # name="oceans_waves">, and the control service lives at
        # /world/<sdf-name>/control. SimSpec owns that string (and the
        # same warning) so a scenario switching worlds reaches us here.
        self.declare_parameter("world_name", SimSpec().world_name)
        self.declare_parameter("model_name", SimSpec().model_name)
        self.declare_parameter("required_nodes", [""])
        self.declare_parameter("recorder_topics", [""])
        self.declare_parameter("ready_timeout_s", 600.0)
        self.declare_parameter("poll_period_s", 1.0)
        # "" = write no verdict file (interactive use).
        self.declare_parameter("verdict_path", "")
        # Physics-liveness probe. Opt-in everywhere (run_sweep injects
        # `physics_probe:=true`; the launches default it off) — the node
        # default matches so no layer disagrees. The two scenario-coupled
        # knobs are threaded per-run by gate_launch from the loaded
        # scenario (floor = the plant's bladder_min; publish rate >= 5x
        # the bcu bridge's, whose last-received-target contention the
        # probe must dominate). Every other probe number is a module
        # constant above.
        self.declare_parameter("probe_enabled", False)
        self.declare_parameter("probe_floor_m3", PlantSpec().bladder_min_m3)
        self.declare_parameter("probe_pub_rate_hz", 50.0)

        self._world = self.get_parameter("world_name").value
        model = self._model_name = self.get_parameter("model_name").value
        self._required_nodes = [
            n for n in (self.get_parameter("required_nodes").value or []) if n
        ]
        self._recorder_topics = [
            t for t in (self.get_parameter("recorder_topics").value or []) if t
        ]
        self._ready_timeout_s = float(self.get_parameter("ready_timeout_s").value)
        self._verdict_path = self.get_parameter("verdict_path").value
        self._exit_fn = exit_fn

        self._probe_enabled = bool(self.get_parameter("probe_enabled").value)
        self._probe_floor_m3 = float(self.get_parameter("probe_floor_m3").value)
        self._probe_pub_rate_hz = float(self.get_parameter("probe_pub_rate_hz").value)

        self._buoyancy_cmd_topic = SimTopics.BUOYANCY_COMMAND.format(model_name=model)
        self._odom_topic = SimTopics.ODOMETRY.format(model_name=model)
        self._volume_topic = SimTopics.BUOYANCY_VOLUME_STATE.format(model_name=model)
        self._suppress_topic = SimTopics.HEAVE_ENTRY_SUPPRESS.format(model_name=model)

        # Proof-of-stepping probes: both only ever publish while physics
        # is integrating, so a message on each == the world is unpaused
        # AND the model is being simulated AND the gz->ROS bridge works.
        # Tracked as a labelled set so the readiness test and the failure
        # detail are the same object (`not self._unseen` / `sorted(...)`).
        # Both are high-rate streams (odometry 100 Hz, buoyancy volume one
        # message per 1 ms physics step), so the handles are kept and the
        # subscriptions destroyed the moment the gate opens -- see
        # _open_gate. Left alive they would deserialize a kHz-class stream
        # in Python for the whole multi-ks run to re-set an already-true
        # flag.
        self._unseen = {f"msg:{self._odom_topic}", f"msg:{self._volume_topic}"}
        self._odom_sub = self.create_subscription(
            Odometry, self._odom_topic, self._on_odom, UUVQoS.SENSOR_STREAM
        )
        self._volume_sub = self.create_subscription(
            Float64, self._volume_topic, self._on_volume, 10
        )

        self._ready_pub = create_publisher_for_topic(self, UUVTopics.SIM_READY)
        # Probe command publisher, created up front even though it first
        # publishes minutes later: DDS endpoint matching under a slot
        # stampede is exactly the latency disease this node exists to
        # absorb, so let it match during the graph wait. Same QoS as the
        # bcu_sim_bridge's publisher on this topic (depth 10, volatile).
        self._probe_pub = (
            self.create_publisher(Float64, self._buoyancy_cmd_topic, 10)
            if self._probe_enabled
            else None
        )
        # Probe state. Phase timing runs on odometry-stamp SIM seconds
        # (_odom_t): under sweep load RTF sits at 0.3-0.7 and wall-clock
        # deadlines would false-fail healthy-but-slow physics. _phase is
        # None until the stepping proof passes; it doubles as the "did the
        # probe run" flag for the verdict detail.
        self._phase = None
        self._t_phase = 0.0
        self._odom_t = 0.0
        self._latest_z = None
        self._latest_volume = None
        # Bounded: only the trailing quiescence window is ever consumed,
        # and 100 Hz odometry would otherwise grow this for the whole
        # settle wait and pin it for the run's lifetime.
        self._baseline_samples = deque(maxlen=_PROBE_BASELINE_BUFFER)
        self._baseline_z = None
        self._baseline_vol = None
        self._probe_target = None
        self._probe_timer = None
        self._suppress_sends = 0
        self._t_restore_ok = None
        self._unpause_requested = False
        self._unpause_ok = False
        self._unpause_attempts = 0
        self._last_unpause_ts = 0.0
        self._done = False
        self._t_start = now_s(self)

        poll_period_s = float(self.get_parameter("poll_period_s").value)
        self._poll_timer = self.create_timer(poll_period_s, self._poll)

        self.get_logger().info(
            f"sim_ready_gate: world={self._world}, "
            f"{len(self._required_nodes)} required nodes, "
            f"{len(self._recorder_topics)} recorder sentinels, "
            f"timeout {self._ready_timeout_s:.0f}s"
        )

    def _on_odom(self, msg) -> None:
        self._unseen.discard(f"msg:{self._odom_topic}")
        self._latest_z = msg.pose.pose.position.z
        # Sim time from the stamp, node clock as the fallback for an
        # unstamped source — the run_watchdog convention.
        stamp = msg.header.stamp
        t = stamp.sec + stamp.nanosec * 1e-9
        self._odom_t = t if t > 0.0 else now_s(self)
        if self._phase == PROBE_BASELINE:
            self._baseline_samples.append((self._odom_t, self._latest_z))

    def _on_volume(self, msg) -> None:
        self._unseen.discard(f"msg:{self._volume_topic}")
        self._latest_volume = msg.data

    def _graph_missing(self) -> list[str]:
        live = set(self.get_node_names())
        endpoint_checks = [
            (
                f"sub:{self._buoyancy_cmd_topic} (ros_gz bridge reader)",
                self.count_subscribers(self._buoyancy_cmd_topic),
            ),
            (
                f"pub:{self._odom_topic} (ros_gz bridge writer)",
                self.count_publishers(self._odom_topic),
            ),
            (
                f"sub:{UUVTopics.BCU_RPM} (BCU bridge reader)",
                self.count_subscribers(UUVTopics.BCU_RPM),
            ),
            (
                f"pub:{UUVTopics.BCU_RPM} (bcu_node writer)",
                self.count_publishers(UUVTopics.BCU_RPM),
            ),
        ]
        endpoint_checks += [
            (f"sub:{t} (bag recorder)", self.count_subscribers(t))
            for t in self._recorder_topics
        ]
        missing = missing_requirements(self._required_nodes, live, endpoint_checks)
        # gz-side spawn proof, checked LAST (it shells out). Topic
        # presence can't prove this — the bidirectional parameter_bridge
        # advertises both buoyancy_engine gz topics with or without a
        # model — but the entity list can: the model appearing means the
        # spawn fully completed (all systems Configured on it) while the
        # world is still paused, so unpausing cannot race it.
        if not missing and not self._gz_model_spawned():
            missing = [f"gz-model:{self._model_name} (spawn incomplete)"]
        return missing

    def _gz_model_spawned(self) -> bool:
        try:
            proc = subprocess.run(
                ["gz", "model", "--list"],
                capture_output=True,
                text=True,
                timeout=_GZ_MODEL_LIST_TIMEOUT_S,
            )
        except (subprocess.TimeoutExpired, OSError):
            return False
        return any(
            token.strip("- ") == self._model_name
            for token in proc.stdout.splitlines()
        )

    def _poll(self) -> None:
        if self._done:
            return
        elapsed = now_s(self) - self._t_start

        if not self._unpause_requested:
            missing = self._graph_missing()
            if missing:
                if elapsed >= self._ready_timeout_s:
                    self._fail(elapsed, missing)
                else:
                    self.get_logger().info(
                        f"sim_ready_gate: waiting ({elapsed:.0f}s) — "
                        f"missing: {', '.join(missing)}",
                        throttle_duration_sec=10.0,
                    )
                return
            self._unpause_requested = True
            self.get_logger().info(
                f"sim_ready_gate: graph complete after {elapsed:.0f}s — unpausing"
            )

        if not self._unseen:
            if self._probe_enabled:
                self._probe_step(elapsed)
            else:
                self._open_gate(elapsed)
            return
        if elapsed >= self._ready_timeout_s:
            self._fail(elapsed, sorted(self._unseen))
            return
        self._unpause()

    def _probe_step(self, elapsed: float) -> None:
        """Advance the physics-liveness probe one poll tick.

        Phases (module docstring has the physics):

        BASELINE  suppress the entry servo, wait out the passive spawn
                  settle (quiescence gate), then take a median float z
                  and the spawn volume echo.
        DEFLATE   publish the deflate set point at probe_pub_rate_hz until
                  the hull has demonstrably sunk _PROBE_DZ_M.
        RESTORE   go silent; the bridge's 10 Hz spawn republish re-inflates
                  the plugin and the hull refloats on its own. Volume and
                  z must HOLD at the baseline for _RESTORE_HOLD_S before
                  the entry servo is released and the gate opens.

        Per-phase deadlines run on odometry-stamp sim seconds; the wall
        ready_timeout_s check below still bounds the whole probe, so a
        stalled odometry stream (frozen phase clock) cannot strand us.
        """
        if elapsed >= self._ready_timeout_s:
            self._fail(elapsed, [f"probe:{self._phase} exceeded ready_timeout_s"])
            return
        phase_s = self._odom_t - self._t_phase

        if self._phase is None:
            self._phase = PROBE_BASELINE
            self._t_phase = self._odom_t
            self._suppress_entry(True)
            self.get_logger().info(
                f"sim_ready_gate: probe baseline ({elapsed:.0f}s) — entry servo "
                f"suppressed"
            )
        elif self._phase == PROBE_BASELINE:
            # Re-send the suppress while sampling (a single gz CLI publish
            # can race the plugin's subscriber discovery; the message is
            # idempotent) — capped: each send synchronously forks the ruby
            # CLI inside the executor, and a lost suppress already fails
            # safe via the RESTORE deadline.
            self._suppress_entry(True)
            if phase_s < _PROBE_BASELINE_MIN_S:
                return
            baseline = quiescent_baseline_z(
                self._baseline_samples,
                self._odom_t,
                _PROBE_QUIESCENCE_S,
                _PROBE_QUIESCENCE_PTP_M,
            )
            if baseline is None:
                if phase_s >= _PROBE_BASELINE_DEADLINE_S:
                    zs = [z for _, z in self._baseline_samples] or [float("nan")]
                    self._fail(
                        elapsed,
                        [
                            f"physics:no-quiescent-baseline("
                            f"z=[{min(zs):.3f},{max(zs):.3f}]m in {phase_s:.0f}s"
                            f" — hull still translating; deep spawn?)"
                        ],
                    )
                return
            self._baseline_z = baseline
            self._baseline_vol = self._latest_volume
            self._probe_target = probe_target_m3(
                self._baseline_vol, _PROBE_DELTA_M3, self._probe_floor_m3
            )
            self._phase = PROBE_DEFLATE
            self._t_phase = self._odom_t
            self._probe_timer = self.create_timer(
                1.0 / self._probe_pub_rate_hz, self._publish_probe_target
            )
            self.get_logger().info(
                f"sim_ready_gate: probe deflate — z0={self._baseline_z:.3f}m, "
                f"vol {self._baseline_vol:.4e}->{self._probe_target:.4e} m3"
            )
        elif self._phase == PROBE_DEFLATE:
            if hull_responded(self._baseline_z, self._latest_z, _PROBE_DZ_M):
                self.destroy_timer(self._probe_timer)
                self._probe_timer = None
                self._phase = PROBE_RESTORE
                self._t_phase = self._odom_t
                self.get_logger().info(
                    f"sim_ready_gate: hull responded "
                    f"(dz={self._baseline_z - self._latest_z:.3f}m) — restoring"
                )
            elif phase_s >= _PROBE_DEADLINE_S:
                self._fail_probe(elapsed, _PROBE_DZ_M, phase_s)
        elif self._phase == PROBE_RESTORE:
            restored = within_eps(
                self._latest_volume, self._baseline_vol, _RESTORE_VOL_EPS_M3
            ) and within_eps(self._latest_z, self._baseline_z, _RESTORE_Z_EPS_M)
            if restored:
                if self._t_restore_ok is None:
                    self._t_restore_ok = self._odom_t
                elif self._odom_t - self._t_restore_ok >= _RESTORE_HOLD_S:
                    self._suppress_entry(False)
                    self._open_gate(elapsed)
            else:
                self._t_restore_ok = None
                if phase_s >= _RESTORE_DEADLINE_S:
                    self._fail_probe(elapsed, _RESTORE_Z_EPS_M, phase_s)

    def _publish_probe_target(self) -> None:
        msg = Float64()
        msg.data = self._probe_target
        self._probe_pub.publish(msg)

    def _fail_probe(self, elapsed: float, dz_needed_m: float, phase_s: float) -> None:
        self._fail(
            elapsed,
            [
                probe_failure_label(
                    self._phase,
                    self._baseline_z - self._latest_z,
                    dz_needed_m,
                    phase_s,
                    self._baseline_vol,
                    self._latest_volume,
                )
            ],
        )

    def _suppress_entry(self, on: bool) -> None:
        # Fire-and-forget by design: the plugin treats a lost `true` as
        # "servo may fire during the probe", which the RESTORE deadline
        # converts into a retryable abort_init; a lost `false` is repaired
        # by launch teardown (fresh plugin next run). Never worth blocking
        # readiness on the publish result — and the `true` re-sends are
        # capped, since each one forks the ruby CLI inside the executor.
        if on:
            if self._suppress_sends >= _MAX_SUPPRESS_SENDS:
                return
            self._suppress_sends += 1
        cmd = [
            "gz",
            "topic",
            "-t",
            self._suppress_topic,
            "-m",
            "gz.msgs.Boolean",
            "-p",
            f"data: {'true' if on else 'false'}",
        ]
        try:
            subprocess.run(
                cmd, capture_output=True, text=True, timeout=_GZ_TOPIC_TIMEOUT_S
            )
        except (subprocess.TimeoutExpired, OSError) as exc:
            self.get_logger().warn(f"sim_ready_gate: entry-suppress publish: {exc}")

    def _unpause(self) -> None:
        # Idempotent on the gz side, but each call forks the `gz` ruby
        # CLI, so ask once and only re-ask on a slow cadence if the call
        # itself failed. Without this a run that never starts stepping
        # re-forks every poll for the whole ready_timeout_s.
        if self._unpause_ok or self._unpause_attempts >= _MAX_UNPAUSE_ATTEMPTS:
            return
        t = now_s(self)
        if (
            self._unpause_attempts
            and t - self._last_unpause_ts < _UNPAUSE_RETRY_PERIOD_S
        ):
            return
        self._last_unpause_ts = t
        self._unpause_attempts += 1
        cmd = [
            "gz",
            "service",
            "-s",
            f"/world/{self._world}/control",
            "--reqtype",
            "gz.msgs.WorldControl",
            "--reptype",
            "gz.msgs.Boolean",
            "--timeout",
            "2000",
            "--req",
            "pause: false",
        ]
        try:
            proc = subprocess.run(
                cmd, capture_output=True, text=True, timeout=_GZ_SERVICE_TIMEOUT_S
            )
            if proc.returncode == 0:
                self._unpause_ok = True
            else:
                self.get_logger().warn(
                    f"sim_ready_gate: unpause attempt "
                    f"{self._unpause_attempts}/{_MAX_UNPAUSE_ATTEMPTS} failed: "
                    f"{proc.stderr.strip()}"
                )
        except (subprocess.TimeoutExpired, OSError) as exc:
            self.get_logger().warn(
                f"sim_ready_gate: unpause attempt "
                f"{self._unpause_attempts}/{_MAX_UNPAUSE_ATTEMPTS} errored: {exc}"
            )

    def _open_gate(self, elapsed: float) -> None:
        self._done = True
        self._poll_timer.cancel()
        # The probes have told us everything they can. Drop them before
        # settling in for the run: odometry is 100 Hz and the buoyancy
        # volume publishes once per 1 ms physics step, and this process
        # outlives the whole mission purely to hold a latch.
        self.destroy_subscription(self._odom_sub)
        self.destroy_subscription(self._volume_sub)
        if self._probe_pub is not None:
            self.destroy_publisher(self._probe_pub)
            self._probe_pub = None
        msg = Bool()
        msg.data = True
        self._ready_pub.publish(msg)
        probed = "physics probed, " if self._phase is not None else ""
        self.get_logger().info(
            f"sim_ready_gate: OPEN after {elapsed:.0f}s — world stepping, "
            f"{probed}/sim/ready latched"
        )
        # Stay alive: the latch must persist for late-joining subscribers
        # (auto_mission), and exiting would trip the launch's failure
        # handler.

    def _fail(self, elapsed: float, missing: list[str]) -> None:
        self._done = True
        self._poll_timer.cancel()
        if self._probe_timer is not None:
            self.destroy_timer(self._probe_timer)
            self._probe_timer = None
        if self._phase is not None:
            # The servo may still be suppressed; teardown replaces the
            # plugin next run, but don't leave it muted on our account.
            self._suppress_entry(False)
        detail = {
            "verdict": "abort_init",
            "elapsed_s": round(elapsed, 1),
            "missing": missing,
        }
        if self._phase is not None:
            # Probe telemetry for pilot forensics: which phase died, how
            # far the hull and the volume echo actually moved.
            def _r(v, digits):
                return None if v is None else round(v, digits)

            detail["probe"] = {
                "phase": self._phase,
                "baseline_z": _r(self._baseline_z, 3),
                "last_z": _r(self._latest_z, 3),
                "dz_m": (
                    None
                    if self._baseline_z is None or self._latest_z is None
                    else round(self._baseline_z - self._latest_z, 3)
                ),
                "baseline_vol_m3": _r(self._baseline_vol, 7),
                "last_vol_m3": _r(self._latest_volume, 7),
            }
        if self._verdict_path:
            path = Path(self._verdict_path)
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(json.dumps(detail) + "\n")
        self.get_logger().error(
            f"sim_ready_gate: NOT ready after {elapsed:.0f}s — "
            f"missing: {', '.join(missing)} — aborting run"
        )
        self._exit_fn(EXIT_ABORT_INIT)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimReadyGate()
    spin_node(node)


if __name__ == "__main__":
    main()
