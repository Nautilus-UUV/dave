"""
Centralized structural constants for the Nautilus HAL.

This module contains unit conversions and topic templates.
Tunable physical parameters should be managed via ROS 2 parameters/YAML.
"""


class Conversions:
    """Unit conversion factors."""

    M3_TO_ML = 1e6
    ML_TO_M3 = 1e-6
    CM_TO_M = 0.01
    MM_TO_M = 0.001
    # The dave sea_pressure_sensor plugin fills FluidPressure.fluid_pressure
    # in kPa (standardPressure=101.325, kPaPerM=9.80638) — FluidPressure
    # semantics require Pa, so every consumer rescales here. Keep in one
    # place so /external/pressure and /bcu/pressure can never drift apart.
    KPA_TO_PA = 1000.0


def sea_pressure_pa(fluid_pressure_kpa: float) -> int:
    """dave FluidPressure (kPa, despite the field name) -> integer Pa."""
    return int(fluid_pressure_kpa * Conversions.KPA_TO_PA)


# record_throttle republishes every recorded stream on a sibling topic
# carrying this suffix. The rule lives here because three places must
# agree on it exactly: bridge.launch.py (builds the recorder's topic
# list), record_throttle (derives its outputs), and gate_launch (whose
# recorder sentinels ARE throttled topics -- a sentinel naming a topic
# nobody publishes makes the gate wait out its full timeout and abort
# every run in a sweep).
THROTTLED_SUFFIX = "/throttled"


def throttled(topic: str) -> str:
    """Sibling topic that ``record_throttle`` republishes ``topic`` on."""
    return f"{topic}{THROTTLED_SUFFIX}"


class SimTopics:
    """Templates for Gazebo/Simulation topics."""

    # model_name is substituted at runtime via .format(model_name=...).
    BUOYANCY_VOLUME_STATE = "/model/{model_name}/buoyancy_engine/current_volume"
    BUOYANCY_COMMAND = "/model/{model_name}/buoyancy_engine"
    SEA_PRESSURE = "/model/{model_name}/sea_pressure"
    IMU = "/model/{model_name}/imu"
    # Ground-truth model pose, bridged out of Gazebo by
    # dave_robot_models/config/glider_nautilus/robot_config.py:16.
    # Sim-only — production controllers must not depend on this.
    ODOMETRY = "/model/{model_name}/odometry"


class SimDebugTopics:
    """Templates for sim-only ROS-side debug egress.

    These topics are intentionally NOT registered in
    ``py_pkg.uuv_ros_core`` — production controllers must not be able to
    accidentally subscribe to them. They exist only so Tier 3 sim-integration
    tests can observe simulator-internal state (e.g. joint positions) that
    real hardware does not yet expose. The ``/sim/`` prefix is the contract:
    anything under it is privileged simulator feedback, not part of the
    glider's hardware-facing surface.
    """

    # model_name is substituted at runtime via .format(model_name=...).
    # Ground-truth model pose (geometry_msgs/Pose: spawn-frame-relative
    # orientation, position in metres/world frame) republished by
    # gt_pose_bridge. The Tier-3 paired test compares the gravity-tilt
    # estimator's /position/estimation pitch/roll against this truth -- it is a
    # sim diagnostic, never a production controller input.
    GROUND_TRUTH_POSE = "/sim/{model_name}/ground_truth/pose"

    # BCU pump-fault provenance (std_msgs/Float32): the constant
    # effectiveness the bcu_sim_bridge scales commanded RPM by, so bags
    # stay self-describing about the actuator fault. Predates the /sim/
    # prefix — keeps the legacy name so recorded-bag streams stay
    # comparable across sweeps. Never comms-gated.
    BCU_PUMP_FAULT = "/bcu/rpm/fault"
