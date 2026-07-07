from py_pkg.scenarios.spec.rig import ImuNoiseSpec, SimSpec
from py_pkg.sensor_noise import GaussianQuantizedNoise, rng_from_seed
from py_pkg.uuv_ros_core import (
    UUVTopics,
    create_publisher_for_topic,
)
from sensor_msgs.msg import Imu

from ..constants import SimTopics
from .bridge_base import SimBridgeNode, run_bridge


class IMUSimBridge(SimBridgeNode):
    def __init__(self):
        super().__init__("nautilus_imu_bridge")

    def setup_bridges(self):
        self.declare_parameter("model_name", SimSpec().model_name)
        self.model_name = self.get_parameter("model_name").value

        # Per-axis Gaussian sensor noise as (x, y, z) double-array params
        # (defaults = the lake-fitted NoiseSpec values, so bare `ros2 run`
        # behaves like nominal; the scenario compiler injects a derived
        # seed). One RNG shared across the six channels — per-channel
        # RNGs from the same seed would draw identical z-scores, i.e.
        # spuriously axis-correlated noise.
        noise_def = ImuNoiseSpec()
        self.declare_parameter("noise_seed", 0)
        self.declare_parameter("noise_accel_sigma", list(noise_def.accel_sigma_mps2))
        self.declare_parameter("noise_gyro_sigma", list(noise_def.gyro_sigma_rads))
        rng = rng_from_seed(self.get_parameter("noise_seed").value)
        self.accel_noise = tuple(
            GaussianQuantizedNoise(sigma=sigma, rng=rng)
            for sigma in self.get_parameter("noise_accel_sigma").value
        )
        self.gyro_noise = tuple(
            GaussianQuantizedNoise(sigma=sigma, rng=rng)
            for sigma in self.get_parameter("noise_gyro_sigma").value
        )
        # One gate for the whole callback block: with noise compiled off
        # (all-zero sigmas) the 50 Hz stream skips six no-op calls/msg.
        self.noise_active = any(n.is_active for n in self.accel_noise + self.gyro_noise)

        # One physical IMU -> one topic, just like the STM bridge.
        self.imu_pub = create_publisher_for_topic(self, UUVTopics.IMU)

        # Gazebo IMU topic (bridged by ros_gz_bridge)
        self.sim_imu_sub = self.create_subscription(
            Imu,
            SimTopics.IMU.format(model_name=self.model_name),
            self.sim_imu_callback,
            10,
        )

        self.get_logger().info(
            f"Nautilus IMU Bridge: Listening for IMU on "
            f"{SimTopics.IMU.format(model_name=self.model_name)}"
        )
        self.get_logger().info(f"Nautilus IMU Bridge: Publishing to {UUVTopics.IMU}")

    def sim_imu_callback(self, msg):
        """Republish the sim IMU in the exact shape the STM emits on hardware.

        The point of this bridge is parity: the downstream prefilter ->
        attitude estimator must run unchanged in sim and on the bench. The STM
        provides accel (specific force, gravity included) and gyro in the NED
        body frame (x forward, y right, z down) and NO orientation (REP-145:
        orientation_covariance[0] = -1). The Gazebo glider model is authored NED
        too (see model.sdf), so its IMU already reports in that frame -- accel and
        gyro pass straight through with no axis remap, exactly matching what the
        STM's robot_specs mounting map produces on hardware. Gazebo also hands us
        a full orientation quaternion, so we strip it here, otherwise the
        gravity-tilt estimator would silently cheat off the simulator's
        ground-truth attitude in sim and behave differently on hardware.

        Gaussian noise (lake-fitted sigmas) is added per axis just before
        publishing — the SDF IMU sensor is clean, so this is the only
        noise source. Covariance fields stay untouched for STM parity
        (the hardware bridge sets only orientation_covariance[0] = -1).
        """
        msg.header.frame_id = "imu"
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0
        msg.orientation.w = 0.0
        msg.orientation_covariance[0] = -1.0
        if self.noise_active:
            la = msg.linear_acceleration
            la.x = self.accel_noise[0].apply(la.x)
            la.y = self.accel_noise[1].apply(la.y)
            la.z = self.accel_noise[2].apply(la.z)
            av = msg.angular_velocity
            av.x = self.gyro_noise[0].apply(av.x)
            av.y = self.gyro_noise[1].apply(av.y)
            av.z = self.gyro_noise[2].apply(av.z)
        self.imu_pub.publish(msg)


def main(args=None):
    run_bridge(IMUSimBridge, args)


if __name__ == "__main__":
    main()
