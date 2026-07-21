from py_pkg.scenarios.spec.rig import ExternalSensorBridgeSpec, NoiseSpec, SimSpec
from py_pkg.uuv_ros_core import UUVTopics, now_s
from sensor_msgs.msg import FluidPressure
from std_msgs.msg import Int32

from ..constants import SimTopics, sea_pressure_pa
from .bridge_base import SimBridgeNode, run_bridge


class ExternalSensorSimBridge(SimBridgeNode):
    def __init__(self):
        super().__init__("nautilus_external_sensor_bridge")

    def setup_bridges(self):
        self.declare_parameter("model_name", SimSpec().model_name)
        self.declare_parameter(
            "publish_rate_hz", ExternalSensorBridgeSpec().publish_rate_hz
        )
        # Sensor-noise model (defaults = lake-fitted NoiseSpec values: the
        # real external channel is quantization-dominated — sub-LSB
        # Gaussian on a 100 Pa comb).
        self.noise = self.declare_pressure_noise("noise", NoiseSpec().external_pressure)
        # Persistent sensor fault on this channel (fault_kind/_magnitude/
        # _drop_prob/_seed — unprefixed, matching this bridge's noise_*
        # convention): bias/drift/stuck wrap the noise chain, dropout
        # gates this stream's publish.
        self.fault_chan, self.fault_drop = self.declare_sensor_fault("", self.noise)
        # Uniform comms fault (no interception layer when inactive).
        self.declare_comms_drop()
        self.model_name = self.get_parameter("model_name").value
        self.latest_pressure = 0
        publish_rate_hz = self.get_parameter("publish_rate_hz").value
        self.pub_timer = self.create_timer(1.0 / publish_rate_hz, self.publish_at_rate)

        self.pressure_pub = self.create_bridged_publisher(UUVTopics.EXTERNAL_PRESSURE)
        self.sim_pressure_sub = self.create_subscription(
            FluidPressure,
            SimTopics.SEA_PRESSURE.format(model_name=self.model_name),
            self.sim_external_pressure_callback,
            10,
        )

        self.get_logger().info(
            f"Nautilus Sensor Bridge: Listening for sea pressure on "
            f"{SimTopics.SEA_PRESSURE.format(model_name=self.model_name)}"
        )

    def sim_external_pressure_callback(self, msg):
        self.latest_pressure = sea_pressure_pa(msg.fluid_pressure)

    def publish_at_rate(self):
        """Publish external sensors.

        The persistent-fault + noise chain runs per published tick (a
        fresh ADC read each cycle); the cached true value stays clean.
        A dropout fault suppresses only this publish, keeping the timer
        cadence (which distinguishes dropout from a stuck channel).
        """
        t_s = now_s(self)
        if self.fault_drop.should_drop(t_s):
            return
        pressure_msg = Int32(data=self.fault_chan.sample_int(self.latest_pressure, t_s))
        self.pressure_pub.publish(pressure_msg)


def main(args=None):
    run_bridge(ExternalSensorSimBridge, args)


if __name__ == "__main__":
    main()
