"""Surface sim composition.

Brings up everything needed to fire the SURFACE mission (mission_id=2),
which drives the glider back to gauge pressure 0 from wherever it is
and self-terminates once the BCU has held neutral on the surface for a
configurable hold window:

    HAL bridges + Gazebo + glider robot
        + py_pkg control_stack    (imu_prefilter, attitude_node, bcu_node,
                                   acu_node, pathfinding_node)
        + optional MissionCommand + start auto-publish

Reused by ``test/sim/test_surface_sim.py``. Mission data is not in the
scenario YAML — for SURFACE the target is fixed at gauge 0, so the only
mission-side knob is autostart:

    ros2 launch nautilus_hal surface_sim.launch.py headless:=false \\
        mission_autostart:=true
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

_MISSION_ID_SURFACE = 2


def _build_robot_launch(context, *_args, **_kwargs):
    """Render the SDF from the scenario YAML (if hydrodynamics block set)
    and include dave_robot.launch.py with the resulting path.

    A scenario without a `rig.hydrodynamics:` block round-trips to the
    canonical model.sdf so this code path is bit-identical to today's
    launch for nominal/baseline.
    """
    from nautilus_hal.render_sdf import description_file_for_scenario

    scenario_path = LaunchConfiguration("scenario").perform(context)
    description_file = description_file_for_scenario(scenario_path)
    return [
        LogInfo(msg=f"Spawning SDF: {description_file}"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [
                    os.path.join(
                        FindPackageShare("dave_demos").find("dave_demos"),
                        "launch",
                        "dave_robot.launch.py",
                    )
                ]
            ),
            launch_arguments={
                "z": "-5",
                "roll": "3.141592653589793",
                "yaw": "1.5707963267948966",
                "namespace": "glider_nautilus",
                "world_name": "dave_ocean_waves",
                "paused": "false",
                "gui": LaunchConfiguration("gui").perform(context),
                "headless": LaunchConfiguration("headless").perform(context),
                "description_file": description_file,
            }.items(),
        ),
    ]


def generate_launch_description():
    record = LaunchConfiguration("record")
    run_id = LaunchConfiguration("run_id")
    sampler_id = LaunchConfiguration("sampler_id")
    scenario = LaunchConfiguration("scenario")
    bag_path = LaunchConfiguration("bag_path")
    bag_compression = LaunchConfiguration("bag_compression")

    bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    FindPackageShare("nautilus_hal").find("nautilus_hal"),
                    "launch",
                    "bridge.launch.py",
                )
            ]
        ),
        launch_arguments={
            "record": record,
            "run_id": run_id,
            "sampler_id": sampler_id,
            "scenario": scenario,
            "bag_path": bag_path,
            "bag_compression": bag_compression,
        }.items(),
    )

    # Robot spawn is built inside `_build_robot_launch`, which resolves
    # the SDF path from the scenario YAML (canonical when nominal,
    # Jinja-rendered when a sampled `rig.hydrodynamics:` block is set).
    control_stack_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    FindPackageShare("py_pkg").find("py_pkg"),
                    "launch",
                    "control_stack.launch.py",
                )
            ]
        ),
        launch_arguments={
            "scenario": scenario,
        }.items(),
    )

    # Latched mission autostart (one long-lived publisher holding /path and the
    # start command transient_local for the launch lifetime). Replaces the old
    # one-shot `ros2 topic pub --once` autostart whose latched samples vanished
    # when the publisher exited, intermittently leaving pathfinding stuck on
    # "waiting for /path". Surface mission holds at gauge 0; gated internally on
    # mission_autostart.
    mission_autostart_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    FindPackageShare("py_pkg").find("py_pkg"),
                    "launch",
                    "mission_autostart.launch.py",
                )
            ]
        ),
        launch_arguments={
            "mission_autostart": LaunchConfiguration("mission_autostart"),
            "mission_id": str(_MISSION_ID_SURFACE),
            # Arms bcu_node's tank-limit clamp via a latched DiveInit
            # carrying the scenario's plant tank endpoints.
            "scenario": LaunchConfiguration("scenario"),
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "scenario",
                default_value=os.path.join(
                    FindPackageShare("py_pkg").find("py_pkg"),
                    "scenarios",
                    "library",
                    "nominal.yaml",
                ),
                description=(
                    "Path to a scenario YAML. Parameterizes the HAL bridges "
                    "(rig block) and the control stack (control block); "
                    "defaults to the installed nominal. Mission data is NOT "
                    "in the YAML — see mission_autostart."
                ),
            ),
            DeclareLaunchArgument(
                "mission_autostart",
                default_value="false",
                description=(
                    "If true, the launch publishes the surface MissionCommand "
                    "(mission_id=2, target=0.0) plus `start` ~8 s after bringup."
                ),
            ),
            DeclareLaunchArgument(
                "gui",
                default_value="true",
                description="DAVE convention: keep true; use `headless` to toggle display.",
            ),
            DeclareLaunchArgument(
                "headless",
                default_value="true",
                description="True hides the Gazebo GUI; set false to visualize.",
            ),
            DeclareLaunchArgument(
                "hold",
                default_value="false",
                description=(
                    "Informational: SURFACE self-terminates ~10 s after reaching "
                    "the surface, but the launch keeps Gazebo and the control "
                    "stack running so the operator can fire another mission."
                ),
            ),
            DeclareLaunchArgument(
                "record",
                default_value="false",
                description="If true, also record HAL topics to an MCAP rosbag.",
            ),
            DeclareLaunchArgument(
                "run_id",
                default_value="surface",
                description=(
                    "Run identifier baked into the bag output dir as "
                    "./sim_data/[{sampler_id}/]{run_id}_{timestamp}/raw."
                ),
            ),
            DeclareLaunchArgument(
                "sampler_id",
                default_value="",
                description=(
                    "Optional parent folder for grouping bags from one sampler "
                    "invocation. Empty (default) preserves the historical layout."
                ),
            ),
            DeclareLaunchArgument(
                "bag_path",
                default_value="",
                description=(
                    "Explicit bag output directory; overrides the "
                    "sampler_id/run_id synthesis. Sweep runners set this so "
                    "they can post-process the bag deterministically."
                ),
            ),
            DeclareLaunchArgument(
                "bag_compression",
                default_value="file",
                description=(
                    "Rosbag2 compression mode. 'file' compresses each MCAP at "
                    "shutdown (default); 'none' disables it so a SIGKILL during "
                    "teardown can't strip metadata.yaml or leave half-finalized "
                    "files. Sweep runners pass 'none' and compress after each reap."
                ),
            ),
            bridge_launch,
            OpaqueFunction(function=_build_robot_launch),
            control_stack_launch,
            mission_autostart_launch,
        ]
    )
