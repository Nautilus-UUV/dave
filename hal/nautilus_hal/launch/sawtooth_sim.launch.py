"""Sawtooth glide sim composition.

Brings up everything needed to drive the simulated glider through one
or more SAWTOOTH dives -- a hard descend at -angle_rad to
target_pressure_pa, then a hard ascend at +angle_rad to
shallow_pressure_pa, repeated for n_oscillations dives before a final
ascent to the surface and self-terminating:

    HAL bridges + Gazebo + glider robot
        + py_pkg control_stack    (imu_prefilter, attitude_node, bcu_node,
                                   acu_node, pathfinding_node)
        + optional MissionCommand + start auto-publish

Reused by ``test/sim/test_sawtooth_sim.py``. Mission data is not in the
scenario YAML — pass it as launch args:

    ros2 launch nautilus_hal sawtooth_sim.launch.py headless:=false \\
        mission_autostart:=true target_pressure_pa:=147150.0 \\
        shallow_pressure_pa:=0.0 angle_rad:=0.6109 n_oscillations:=1

The mission self-terminates after `n_oscillations` dives (a final ascent
to the surface), but the launch keeps Gazebo and the controllers running
so you can fire another mission from the CLI by publishing /path +
/command directly.
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


_MISSION_ID_SAWTOOTH = 1


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
    # "waiting for /path". Gated internally on mission_autostart.
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
            "mission_id": str(_MISSION_ID_SAWTOOTH),
            "target_pressure_pa": LaunchConfiguration("target_pressure_pa"),
            "shallow_pressure_pa": LaunchConfiguration("shallow_pressure_pa"),
            "angle_rad": LaunchConfiguration("angle_rad"),
            "n_oscillations": LaunchConfiguration("n_oscillations"),
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
                    "in the YAML — see mission_autostart and friends below."
                ),
            ),
            DeclareLaunchArgument(
                "mission_autostart",
                default_value="false",
                description=(
                    "If true, the launch publishes the sawtooth MissionCommand "
                    "(mission_id=1) plus `start` ~8 s after bringup."
                ),
            ),
            DeclareLaunchArgument(
                "target_pressure_pa",
                default_value="147150.0",
                description=(
                    "Deep extremum in gauge Pa (~15 m at 147 150 Pa). "
                    "Only used when mission_autostart is true."
                ),
            ),
            DeclareLaunchArgument(
                "shallow_pressure_pa",
                default_value="0.0",
                description=(
                    "Shallow extremum in gauge Pa. 0 (default) climbs to the "
                    "surface between dives (the legacy profile). Only used "
                    "when mission_autostart is true."
                ),
            ),
            DeclareLaunchArgument(
                "angle_rad",
                default_value="0.6109",
                description=(
                    "Glide pitch magnitude in radians (alternates sign each "
                    "leg). 0.6109 ≈ 35 deg. Only used when mission_autostart "
                    "is true."
                ),
            ),
            DeclareLaunchArgument(
                "n_oscillations",
                default_value="1",
                description=(
                    "How many dives between the two pressures before the "
                    "mission ends with a final ascent to the surface. Only "
                    "used when mission_autostart is true."
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
                    "Informational: SAWTOOTH self-terminates after n_oscillations "
                    "dives, but the launch keeps the stack running so the "
                    "operator can fire another mission. Documents intent."
                ),
            ),
            DeclareLaunchArgument(
                "record",
                default_value="false",
                description="If true, also record HAL topics to an MCAP rosbag.",
            ),
            DeclareLaunchArgument(
                "run_id",
                default_value="sawtooth",
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
