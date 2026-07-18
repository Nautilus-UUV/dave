import os
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _sim_data_root():
    # Inside our container the workspace mounts at /ros2_ws, so absolute
    # paths are sane; on the host we leave it relative so bags land next
    # to whatever cwd the operator launched from (usually the workspace
    # root).
    if (
        os.path.exists("/.dockerenv")
        or "APPTAINER_CONTAINER" in os.environ
        or "SINGULARITY_CONTAINER" in os.environ
    ):
        return "/ros2_ws/sim_data"
    return "./sim_data"


def _default_scenario_path() -> str:
    return os.path.join(
        get_package_share_directory("py_pkg"),
        "scenarios",
        "library",
        "nominal.yaml",
    )


def _wire_bridges(context, *_args, **_kwargs):
    # Loaded inside OpaqueFunction so LaunchConfiguration is resolvable.
    from py_pkg.scenarios.compile import (
        params_for_bcu_bridge,
        params_for_external_sensor_bridge,
        params_for_imu_bridge,
    )
    from py_pkg.scenarios.loader import load_scenario

    scenario = load_scenario(LaunchConfiguration("scenario").perform(context))
    rig = scenario.rig
    parent_seed = scenario.seed

    return [
        Node(
            package="nautilus_hal",
            executable="bcu_sim_bridge",
            name="nautilus_bcu_bridge",
            output="screen",
            parameters=[params_for_bcu_bridge(rig, parent_seed)],
        ),
        Node(
            package="nautilus_hal",
            executable="external_sensor_sim_bridge",
            name="nautilus_external_sensor_bridge",
            output="screen",
            parameters=[params_for_external_sensor_bridge(rig, parent_seed)],
        ),
        Node(
            package="nautilus_hal",
            executable="imu_sim_bridge",
            name="nautilus_imu_bridge",
            output="screen",
            parameters=[params_for_imu_bridge(rig, parent_seed)],
        ),
    ]


def _maybe_record(context, *_args, **_kwargs):
    """Resolve the bag output path and emit a rosbag ExecuteProcess if record:=true.

    The synthesized default is `{sim_data_root}/[{sampler_id}/]{run_id}_{ts}/raw`.
    An empty sampler_id (the default) collapses the middle segment so the
    historical layout `{sim_data_root}/{run_id}_{ts}/raw` is preserved
    bit-for-bit. An explicit `bag_path:=...` short-circuits the synthesis.
    """
    if LaunchConfiguration("record").perform(context).lower() != "true":
        return []

    explicit = LaunchConfiguration("bag_path").perform(context).strip()
    if explicit:
        bag_path = explicit
    else:
        timestamp = datetime.now().strftime("%Y_%m_%d-%H_%M_%S")
        sampler_id = LaunchConfiguration("sampler_id").perform(context).strip()
        run_id = LaunchConfiguration("run_id").perform(context).strip()
        parts = [_sim_data_root()]
        if sampler_id:
            parts.append(sampler_id)
        parts.append(f"{run_id}_{timestamp}")
        parts.append("raw")
        bag_path = "/".join(parts)

    # Ground-truth odometry comes straight off Gazebo via the parameter
    # bridge in dave_robot_models.
    from py_pkg.scenarios.loader import load_scenario
    from py_pkg.uuv_ros_core import TOPIC_MESSAGE_MAP, UUVTopics

    scenario = load_scenario(LaunchConfiguration("scenario").perform(context))
    model_name = scenario.rig.sim.model_name
    gt_odom_topic = f"/model/{model_name}/odometry"

    def _registry_entry(topic):
        # record_throttle wants the "pkg/msg/Type" string; derive it from
        # the registry's message class so a registry rename can't silently
        # desync the bag from the live topics.
        msg_cls = TOPIC_MESSAGE_MAP[topic]
        return topic, f"{msg_cls.__module__.split('.')[0]}/msg/{msg_cls.__name__}"

    # Every recorded topic is funnelled through a record_throttle node so
    # the bag has a single uniform sample rate, independent of the live
    # publish rates. Topics published only by the control stack
    # (/acu/{pitch,roll}, /imu/filtered, /position/{target,estimation})
    # silently produce empty streams when bridges run standalone; that's the
    # intended soft-failure mode.
    record_rate_hz = 1.0
    record_topics = [
        _registry_entry(UUVTopics.IMU),
        _registry_entry(UUVTopics.IMU_FILTERED),
        _registry_entry(UUVTopics.EXTERNAL_PRESSURE),
        _registry_entry(UUVTopics.BCU_RPM),
        _registry_entry(UUVTopics.BCU_FEEDBACK_RPM),
        _registry_entry(UUVTopics.BCU_VALVES),
        _registry_entry(UUVTopics.BCU_FEEDBACK_VALVES),
        _registry_entry(UUVTopics.BCU_FLOW_RATE),
        _registry_entry(UUVTopics.BCU_PRESSURE),
        # Sim-only streams with no registry entry: the fault injector's
        # debug topic and Gazebo's model-scoped ground-truth odometry.
        ("/bcu/rpm/fault", "std_msgs/msg/Int32"),
        _registry_entry(UUVTopics.ACU_PITCH),
        _registry_entry(UUVTopics.ACU_ROLL),
        _registry_entry(UUVTopics.POSITION_TARGET),
        _registry_entry(UUVTopics.POSITION_ESTIMATION),
        (gt_odom_topic, "nav_msgs/msg/Odometry"),
    ]

    throttle_nodes = []
    throttled_topic_names = []
    for input_topic, msg_type in record_topics:
        output_topic = f"{input_topic}/throttled"
        throttled_topic_names.append(output_topic)
        # ROS node names must be valid identifiers — turn slashes into
        # underscores and strip the leading one so /bcu/rpm/fault becomes
        # record_throttle_bcu_rpm_fault.
        node_suffix = input_topic.strip("/").replace("/", "_")
        throttle_nodes.append(
            Node(
                package="nautilus_hal",
                executable="record_throttle",
                name=f"record_throttle_{node_suffix}",
                output="screen",
                parameters=[
                    {
                        "input_topic": input_topic,
                        "output_topic": output_topic,
                        "rate_hz": record_rate_hz,
                        "msg_type": msg_type,
                    }
                ],
            )
        )

    # Sweep orchestrators pass `none` and compress the closed bag after each reap instead.
    bag_compression = (
        LaunchConfiguration("bag_compression").perform(context).strip().lower()
    )
    if bag_compression not in ("file", "none"):
        raise ValueError(
            f"bag_compression must be 'file' or 'none', got {bag_compression!r}"
        )

    cmd = [
        "ros2",
        "bag",
        "record",
        "-o",
        bag_path,
        "-s",
        "mcap",
    ]
    if bag_compression == "file":
        cmd += [
            "--compression-mode",
            "file",
            "--compression-format",
            "zstd",
        ]
    cmd += throttled_topic_names
    return throttle_nodes + [ExecuteProcess(cmd=cmd, output="screen")]


def generate_launch_description():
    scenario_arg = DeclareLaunchArgument(
        "scenario",
        default_value=_default_scenario_path(),
        description=(
            "Path to a scenario YAML. Loaded via py_pkg.scenarios.load_scenario "
            "to parameterize every bridge; defaults to the installed nominal (fault-injection off)."
        ),
    )
    record_arg = DeclareLaunchArgument(
        "record",
        default_value="false",
        description=(
            "If true, record the HAL-published topics (BCU/IMU/external "
            "pressure plus ACU commands and joint-state GT) and the "
            "control-stack outputs (filtered IMU, position target / "
            "estimation) plus the Gazebo ground-truth odometry to an MCAP "
            "rosbag alongside the bridges. Every recorded topic is "
            "throttled to 1 Hz via a sibling <topic>/throttled stream so "
            "the bag has a single uniform sample rate; the live streams "
            "stay at their original rates (IMU 50 Hz, GT odom 100 Hz, "
            "BCU/external 10 Hz) for the estimator, controllers, and "
            "GT-comparison tests."
        ),
    )
    sampler_id_arg = DeclareLaunchArgument(
        "sampler_id",
        default_value="",
        description=(
            "Optional parent folder for grouping bags from one sampler "
            "invocation: ./sim_data/{sampler_id}/{run_id}_{timestamp}/raw. "
            "Empty (the default) drops the middle segment, preserving the "
            "historical ./sim_data/{run_id}_{timestamp}/raw layout."
        ),
    )
    run_id_arg = DeclareLaunchArgument(
        "run_id",
        default_value="dive",
        description=(
            "Run identifier baked into the bag output dir as "
            "./sim_data/[{sampler_id}/]{run_id}_{timestamp}/raw."
        ),
    )
    bag_path_arg = DeclareLaunchArgument(
        "bag_path",
        default_value="",
        description=(
            "Full output directory for the rosbag; when non-empty this "
            "overrides the sampler_id/run_id synthesis above."
        ),
    )
    bag_compression_arg = DeclareLaunchArgument(
        "bag_compression",
        default_value="file",
        description=(
            "Rosbag2 compression mode. 'file' (default) compresses each "
            "MCAP at shutdown; 'none' disables in-recorder compression so "
            "a SIGKILL during teardown can't strip metadata.yaml or leave "
            "half-finalized files. Sweep runners should pass 'none' and "
            "compress the closed bag after each reap."
        ),
    )

    return LaunchDescription(
        [
            scenario_arg,
            record_arg,
            sampler_id_arg,
            run_id_arg,
            bag_path_arg,
            bag_compression_arg,
            OpaqueFunction(function=_wire_bridges),
            OpaqueFunction(function=_maybe_record),
        ]
    )
