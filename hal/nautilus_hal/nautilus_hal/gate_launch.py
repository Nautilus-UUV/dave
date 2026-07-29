"""Launch-side composition helper for the sim_ready_gate.

Shared by the ``*_sim`` HAL launches so the required-node roster and the
failure wiring live in exactly one place. Import from inside an
``OpaqueFunction`` (like ``render_sdf``) so launch files stay import-light.

The gate's process exit is a *failure* signal by contract — on success it
stays alive holding the /sim/ready latch — so the OnProcessExit handler
can unconditionally turn any exit into a full launch Shutdown, which the
sweep runner reaps (and retries, via the abort_init verdict the gate
writes first).
"""

from pathlib import Path

from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node


def physics_probe_launch_argument() -> DeclareLaunchArgument:
    """The shared ``physics_probe`` argument declaration for the ``*_sim``
    launches — one canonical name/default/description so the three copies
    cannot drift (same motive as ``gate_actions_from_context`` itself)."""
    return DeclareLaunchArgument(
        "physics_probe",
        default_value="false",
        description=(
            "If true, sim_ready_gate proves the buoyancy force path is "
            "alive (commanded sink + self-restore at the surface float) "
            "before latching /sim/ready, aborting the run (abort_init) if "
            "the hull never responds — the v3 frozen-init race. run_sweep "
            "injects true into every sweep run; interactive runs default "
            "off (a z=-5 spawn adds a long quiescence wait before the "
            "probe). Design record: nautilus_hal/sim_ready_gate.py."
        ),
    )

# Node names composed by bridge.launch.py.
_BRIDGE_NODES = [
    "nautilus_bcu_bridge",
    "nautilus_external_sensor_bridge",
    "nautilus_imu_bridge",
    "nautilus_anomaly_label_bridge",
]

# Node names composed by py_pkg/launch/control_stack.launch.py (sim
# composition; stm_com/can_com only join on hardware flags).
#
# Deliberately only the nodes a valid RECORDING depends on. mqtt_bridge,
# bcu_debug and acu_debug are also composed by control_stack, but none of
# them is on the run's critical path — requiring them buys no coverage and
# turns any future decision to make one conditional (the composition
# already gates stm_com/can_com behind IfCondition) into a 600 s timeout
# that aborts every run in a sweep.
_CONTROL_STACK_NODES = [
    "imu_prefilter",
    "attitude_node",
    "bcu_node",
    "acu_control_node",
    "pathfinding_node",
    "liveness_node",
]


# The gate must be the one that gives up FIRST: only it can say why a
# bringup failed (it writes the abort_init verdict the sweep runner
# retries on). run_sweep's TIMEOUT_BRINGUP_SEC is 600 s, so leaving the
# gate at the same 600 s meant the orchestrator's SIGKILL landed as the
# gate was writing its verdict — the run then reaped as a verdict-less
# timeout, which for the watchdog-less launches is not even retryable.
# This margin buys the gate room to conclude and tear the launch down.
_GATE_TEARDOWN_MARGIN_S = 120.0
_DEFAULT_READY_TIMEOUT_S = 600.0 - _GATE_TEARDOWN_MARGIN_S


def sim_ready_gate_actions(
    *,
    model_name: str,
    record: bool,
    watchdog: bool,
    bag_path: str,
    # gz SDF world name — dave_ocean_waves.world declares
    # <world name="oceans_waves">; the unpause service path uses this.
    # SimSpec owns the default; pass the scenario's so a world switch
    # doesn't send the unpause to a nonexistent service path.
    world_name: str,
    ready_timeout_s: float = _DEFAULT_READY_TIMEOUT_S,
    # Physics-liveness probe (sink-and-restore before latching /sim/ready).
    # Default OFF at every layer: the probe exists for unattended sweeps
    # (run_sweep injects `physics_probe:=true` into every run), while an
    # interactive z=-5 spawn would pay minutes of ascent+quiescence before
    # the mission could start, and the Tier-3 drivers that kick missions on
    # IMU-flow (not /sim/ready) would collide with the probe's restore.
    probe_enabled: bool = False,
    # Scenario-coupled probe knobs (see gate_actions_from_context, which
    # derives them from the loaded scenario so a sweep that samples the
    # bladder interval or the bridge rate cannot silently invalidate the
    # probe's floor-clamp / topic-contention margins).
    probe_floor_m3: float = 1.0e-3,
    probe_pub_rate_hz: float = 50.0,
) -> list:
    """Gate node + failure handler for one sim launch.

    ``record``/``watchdog`` extend the required-node roster to match what
    the launch actually composed; ``bag_path`` (may be empty) places the
    abort_init verdict next to the bag, same convention as run_watchdog.
    """
    from .constants import SimTopics, throttled
    from py_pkg.uuv_ros_core import UUVTopics

    required = list(_BRIDGE_NODES) + list(_CONTROL_STACK_NODES)
    # [""] not []: launch_ros can't type an empty sequence parameter
    # (aborts the whole launch with "got '()'"); the node declares this
    # same sentinel default and filters out empty strings.
    recorder_topics: list[str] = [""]
    if record:
        required.append("nautilus_record_throttle")
        # Sentinels across the recorded set: the classifier's must-have
        # channel, the ground-truth channel, and the label channel. Built
        # from the same registry constants and the same `throttled` rule
        # bridge.launch.py records with — a literal here would silently
        # stop matching the recorder on any topic rename and strand the
        # gate until it timed out.
        recorder_topics = [
            throttled(UUVTopics.EXTERNAL_PRESSURE),
            throttled(SimTopics.ODOMETRY.format(model_name=model_name)),
            throttled(UUVTopics.ANOMALY_LABEL),
        ]
    if watchdog:
        required.append("run_watchdog")

    verdict_path = (
        str(Path(bag_path).parent / "run_verdict.json") if bag_path.strip() else ""
    )

    gate_node = Node(
        package="nautilus_hal",
        executable="sim_ready_gate",
        name="sim_ready_gate",
        output="screen",
        parameters=[
            {
                "world_name": world_name,
                "model_name": model_name,
                "required_nodes": required,
                "recorder_topics": recorder_topics,
                "verdict_path": verdict_path,
                "ready_timeout_s": float(ready_timeout_s),
                "probe_enabled": bool(probe_enabled),
                "probe_floor_m3": float(probe_floor_m3),
                "probe_pub_rate_hz": float(probe_pub_rate_hz),
            }
        ],
    )
    return [
        gate_node,
        RegisterEventHandler(
            OnProcessExit(
                target_action=gate_node,
                on_exit=[EmitEvent(event=Shutdown(reason="sim_ready_gate failed"))],
            )
        ),
    ]


def gate_actions_from_context(context) -> list:
    """``sim_ready_gate_actions`` wired to a sim launch's own arguments.

    The three ``*_sim`` launches each had a byte-identical OpaqueFunction
    body doing this; the only variation was whether ``watchdog`` was a
    real argument, which ``LaunchConfiguration(name, default=...)``
    absorbs. Keeping it here means a new sim launch adds one action
    instead of pasting a scenario load and four argument decodes.
    """
    from launch.substitutions import LaunchConfiguration
    from py_pkg.scenarios.loader import load_scenario

    def cfg(name: str, default: str = "") -> str:
        return LaunchConfiguration(name, default=default).perform(context)

    scenario = load_scenario(cfg("scenario"))
    return sim_ready_gate_actions(
        model_name=scenario.rig.sim.model_name,
        world_name=scenario.rig.sim.world_name,
        record=cfg("record", "false").lower() == "true",
        watchdog=cfg("watchdog", "false").lower() == "true",
        bag_path=cfg("bag_path"),
        probe_enabled=cfg("physics_probe", "false").lower() == "true",
        # Per-run values, not nominals: the probe's floor clamp must track
        # THIS run's bladder interval, and its publish rate must keep >=5x
        # dominance over THIS run's bcu-bridge republish (last-received-
        # target contention — see the gate's module docstring).
        probe_floor_m3=scenario.rig.plant.bladder_min_m3,
        probe_pub_rate_hz=max(
            50.0, 5.0 * float(scenario.rig.bridges.bcu.publish_rate_hz)
        ),
    )
