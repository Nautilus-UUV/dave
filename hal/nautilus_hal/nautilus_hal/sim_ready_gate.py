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

This node closes both by construction. The sim launches now spawn
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
3. publishes one latched ``/sim/ready`` (UUVTopics.SIM_READY) and stays
   alive to hold the latch -- ``auto_mission`` gates the mission start
   on it, so a run can never begin against a half-built graph or a
   falling vehicle.

If readiness is not reached within ``ready_timeout_s`` the gate writes
``run_verdict.json`` (verdict ``abort_init``, plus what was missing) and
exits nonzero; the sim launches turn that exit into a full launch
Shutdown, and the sweep runner retries the run instead of letting it
burn its whole wall budget recording garbage.

Sim-only by definition -- nothing here exists on hardware.
"""

import json
import subprocess
import sys
from pathlib import Path

import rclpy
from py_pkg.scenarios.spec.rig import SimSpec
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

        self._buoyancy_cmd_topic = SimTopics.BUOYANCY_COMMAND.format(model_name=model)
        self._odom_topic = SimTopics.ODOMETRY.format(model_name=model)
        self._volume_topic = SimTopics.BUOYANCY_VOLUME_STATE.format(model_name=model)

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

    def _on_odom(self, _msg) -> None:
        self._unseen.discard(f"msg:{self._odom_topic}")

    def _on_volume(self, _msg) -> None:
        self._unseen.discard(f"msg:{self._volume_topic}")

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
            self._open_gate(elapsed)
            return
        if elapsed >= self._ready_timeout_s:
            self._fail(elapsed, sorted(self._unseen))
            return
        self._unpause()

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
        msg = Bool()
        msg.data = True
        self._ready_pub.publish(msg)
        self.get_logger().info(
            f"sim_ready_gate: OPEN after {elapsed:.0f}s — world stepping, "
            f"/sim/ready latched"
        )
        # Stay alive: the latch must persist for late-joining subscribers
        # (auto_mission), and exiting would trip the launch's failure
        # handler.

    def _fail(self, elapsed: float, missing: list[str]) -> None:
        self._done = True
        self._poll_timer.cancel()
        detail = {
            "verdict": "abort_init",
            "elapsed_s": round(elapsed, 1),
            "missing": missing,
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
