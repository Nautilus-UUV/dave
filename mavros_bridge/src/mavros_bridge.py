#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import NavSatFix, Imu, FluidPressure
from nav_msgs.msg import Odometry

class MavrosBridge:
    def __init__(self) -> None:
        rospy.init_node("mavros_bridge", anonymous=True)

        rospy.Subscriber("/glider_nautilus/gps", NavSatFix, self._gps_cb)
        rospy.Subscriber("/glider_nautilus/imu", Imu, self._imu_cb)
        rospy.Subscriber("/glider_nautilus/pose_gt", Odometry, self._pose_cb)
        rospy.Subscriber("/glider_nautilus/pressure", FluidPressure, self._pressure_cb)

        self.glob_pos_raw_pub = rospy.Publisher("/mavros/global_position/raw/fix", NavSatFix)
        self.imu_data_pub = rospy.Publisher("/mavros/imu/data", Imu)
        self.glob_pos_local_pub = rospy.Publisher("/mavros/global_position/local", Odometry)
        self.imu_static_pressure_pub = rospy.Publisher("/mavros/imu/static_pressure", FluidPressure)

    def _gps_cb(self, data: NavSatFix) -> None:
        self.glob_pos_raw_pub.publish(data)
    
    def _imu_cb(self, data: Imu) -> None:
        self.imu_data_pub.publish(data)

    def _pose_cb(self, data: Odometry) -> None:
        self.glob_pos_local_pub.publish(data)
    
    def _pressure_cb(self, data: FluidPressure) -> None:
        self.imu_static_pressure_pub.publish(data)

    def loop(self):
        while not rospy.is_shutdown():
            rospy.sleep(1)

if __name__=="__main__":
    mavros_bridge = MavrosBridge()
    mavros_bridge.loop()