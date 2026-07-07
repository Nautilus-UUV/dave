from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    namespace = LaunchConfiguration("namespace").perform(context)

    glider_nautilus_arguments = [
        f"/model/{namespace}/battery/battery/state@sensor_msgs/msg/BatteryState@gz.msgs.BatteryState",
        f"/model/{namespace}/joint/weight_joint/cmd_vel@std_msgs/msg/Float64@gz.msgs.Double",
        f"/model/{namespace}/joint/bladder_joint/cmd_thrust@std_msgs/msg/Float64@gz.msgs.Double",
        f"/model/{namespace}/joint/bladder_joint/ang_vel@std_msgs/msg/Float64@gz.msgs.Double",
        f"/model/{namespace}/navsat@sensor_msgs/msg/NavSatFix@gz.msgs.NavSat",
        f"/model/{namespace}/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry",
        f"/model/{namespace}/odometry_with_covariance@nav_msgs/msg/Odometry@gz.msgs.OdometryWithCovariance",
        f"/model/{namespace}/pose@geometry_msgs/msg/PoseArray@gz.msgs.Pose_V",
        f"/model/{namespace}/imu@sensor_msgs/msg/Imu@gz.msgs.IMU",
        f"/model/{namespace}/buoyancy_engine@std_msgs/msg/Float64@gz.msgs.Double",
        f"/model/{namespace}/buoyancy_engine/current_volume@std_msgs/msg/Float64@gz.msgs.Double",
        f"/world/oceans_waves/model/{namespace}/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model",
        f"/{namespace}/sea_pressure@sensor_msgs/msg/FluidPressure@gz.msgs.FluidPressure",
    ]

    glider_nautilus_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=glider_nautilus_arguments,
        output="screen",
    )

    return [glider_nautilus_bridge]


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            "namespace",
            default_value="",
            description="Namespace",
        ),
    ]

    return LaunchDescription(args + [OpaqueFunction(function=launch_setup)])
