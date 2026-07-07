from py_pkg.scenarios.spec.rig import (
    BcuBridgeSpec,
    FaultInjectorSpec,
    NoiseSpec,
    PlantSpec,
    SimSpec,
)
from py_pkg.sensor_noise import rng_from_seed
from py_pkg.uuv_ros_core import (
    UUVTopics,
    create_publisher_for_topic,
    create_subscription_for_topic,
)
from std_msgs.msg import Float32, Float64, Int16, Int32, UInt8

from ..constants import Conversions, SimTopics
from ..injectors.fault_injection import BCUFaultInjector
from .bridge_base import SimBridgeNode, run_bridge


class BCUSimBridge(SimBridgeNode):
    def __init__(self):
        super().__init__("nautilus_bcu_bridge")

    def setup_bridges(self):
        # ====================
        # Params (defaults from the scenario dataclass, which mirrors
        # robot_specs at nominal — so ros2 run without a launch wrapper
        # still works exactly like today).
        # ====================
        sim_def = SimSpec()
        plant_def = PlantSpec()
        fault_def = FaultInjectorSpec()
        bridge_def = BcuBridgeSpec()

        self.declare_parameter("model_name", sim_def.model_name)
        self.declare_parameter("volume_per_rev_m3", plant_def.volume_per_rev_m3)
        self.declare_parameter("bladder_min_m3", plant_def.bladder_min_m3)
        self.declare_parameter("bladder_max_m3", plant_def.bladder_max_m3)
        self.declare_parameter(
            "tank_pressure_empty_pa", plant_def.tank_pressure_empty_pa
        )
        self.declare_parameter("tank_pressure_full_pa", plant_def.tank_pressure_full_pa)
        self.declare_parameter("fault_mttf_sec", fault_def.mttf_sec)
        self.declare_parameter("fault_num_levels", fault_def.num_levels)
        # 0 = "let `random.Random()` pick"
        self.declare_parameter("rng_seed", 0)
        self.declare_parameter("publish_rate_hz", bridge_def.publish_rate_hz)

        self.model_name = self.get_parameter("model_name").value
        self.volume_per_rev_m3 = self.get_parameter("volume_per_rev_m3").value
        self.bladder_min_m3 = self.get_parameter("bladder_min_m3").value
        self.bladder_max_m3 = self.get_parameter("bladder_max_m3").value
        self.tank_pressure_empty_pa = self.get_parameter("tank_pressure_empty_pa").value
        self.tank_pressure_full_pa = self.get_parameter("tank_pressure_full_pa").value
        # Live bladder fill (m3), refreshed from Gazebo. Until the first echo
        # arrives we report the empty endpoint by sitting at the operating min.
        self.latest_volume_m3 = self.bladder_min_m3

        # Seeded from Gazebo's first BUOYANCY_VOLUME_STATE callback (see
        # sim_bcu_volume_callback). Until then we don't push a volume back
        # to Gazebo — pushing 0.0 + delta would clobber the SDF-initialized
        # bladder state on the first RPM tick.
        self.current_volume = None

        # ====================
        # Fault Injection
        # ====================
        self.rpm_fault_injector = BCUFaultInjector(
            self,
            fault_topic="/bcu/rpm/fault",
            mttf_sec=self.get_parameter("fault_mttf_sec").value,
            num_levels=self.get_parameter("fault_num_levels").value,
            rng=rng_from_seed(self.get_parameter("rng_seed").value),
        )

        # ====================
        # Tank-pressure sensor noise
        # ====================
        self.tank_noise = self.declare_pressure_noise(
            "tank_noise", NoiseSpec().tank_pressure
        )

        # ====================
        # BCU RPM/flow control
        # ====================
        self.last_time = self.get_clock().now()
        self.rpm_sub = create_subscription_for_topic(
            self, UUVTopics.BCU_RPM, self.rpm_callback
        )
        self.flow_pub = create_publisher_for_topic(self, UUVTopics.BCU_FLOW_RATE)

        # ====================
        # Actuator feedback ("ping back")
        # ====================
        self._last_rpm = 0
        self._last_valves = 0
        self.valves_sub = create_subscription_for_topic(
            self, UUVTopics.BCU_VALVES, self.valves_callback
        )
        self.feedback_rpm_pub = create_publisher_for_topic(
            self, UUVTopics.BCU_FEEDBACK_RPM
        )
        self.feedback_valves_pub = create_publisher_for_topic(
            self, UUVTopics.BCU_FEEDBACK_VALVES
        )

        # ==============
        # BCU pressure / volume telemetry
        # ==============
        # Gazebo's buoyancy plugin only exposes bladder *volume*, so the tank
        # pressure sensor is synthesized from the fill state
        self.latest_volume_ml = 0  # bladder volume (mL), echoed from Gazebo
        publish_rate_hz = self.get_parameter("publish_rate_hz").value
        self.pub_timer = self.create_timer(1.0 / publish_rate_hz, self.publish_at_rate)
        self.bcu_pressure_pub = create_publisher_for_topic(self, UUVTopics.BCU_PRESSURE)
        self.bcu_volume_pub = create_publisher_for_topic(self, UUVTopics.BCU_VOLUME)
        self.sim_current_volume_sub = self.create_subscription(
            Float64,
            SimTopics.BUOYANCY_VOLUME_STATE.format(model_name=self.model_name),
            self.sim_bcu_volume_callback,
            10,
        )

        self.sim_volume_pub = self.create_publisher(
            Float64, SimTopics.BUOYANCY_COMMAND.format(model_name=self.model_name), 10
        )

        self.get_logger().info(f"Nautilus BCU Bridge: Listening on {UUVTopics.BCU_RPM}")

    def rpm_callback(self, msg):

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        raw_rpm = float(msg.data)
        rpm = self.rpm_fault_injector.apply(raw_rpm)
        # Cache the fault-adjusted effective rpm for the steady feedback echo.
        self._last_rpm = int(rpm)

        # rpm -> rps -> total revs in dt -> volume change
        rps = rpm / 60.0

        # Only integrate + push to Gazebo once we've synced to its SDF-
        # initialized bladder volume; otherwise we'd overwrite the initial
        # state. Flow-rate feedback below stays open-loop and fires regardless.
        if self.current_volume is not None:
            delta_vol = rps * self.volume_per_rev_m3 * dt

            self.current_volume += delta_vol
            self.current_volume = max(
                self.bladder_min_m3, min(self.current_volume, self.bladder_max_m3)
            )

            out_msg = Float64()
            out_msg.data = self.current_volume
            self.sim_volume_pub.publish(out_msg)

        # Mock Flow Rate feedback (m3/s)
        flow_msg = Float32()
        flow_msg.data = rps * self.volume_per_rev_m3
        self.flow_pub.publish(flow_msg)

    def valves_callback(self, msg):
        # Cache the commanded valve bitmask (bit0=v1, bit1=v2) for the steady
        # feedback echo. The bridge has no other use for valve commands today.
        self._last_valves = int(msg.data)

    def sim_bcu_volume_callback(self, msg):
        # Seed current_volume from Gazebo's first volume report so subsequent
        # RPM-driven integration starts at the SDF-initialized bladder state.
        # last_time is also reset to now so the first dt after sync doesn't
        # include the time spent waiting for Gazebo's first publish.
        if self.current_volume is None:
            self.current_volume = float(msg.data)
            self.last_time = self.get_clock().now()

        # Volume (m3) drives the tank-pressure map; mL is the telemetry unit.
        self.latest_volume_m3 = float(msg.data)
        self.latest_volume_ml = int(msg.data * Conversions.M3_TO_ML)

    def tank_pressure_pa(self) -> int:
        # Synthesize the internal tank sensor from the bladder fill. The bladder
        # is fed from the tank: oil in the external bladder is oil OUT of the
        # tank. So a full bladder (rising) means a drained tank reading the low
        # endpoint, and an empty bladder (sinking) means a tank full of oil
        # reading the high endpoint -- tank pressure runs INVERSE to bladder
        # fill. Dive tests: ~0.7-1.5 barg end to end. Endpoints are ROS params
        # so the curve is tunable per scenario.
        span = self.bladder_max_m3 - self.bladder_min_m3
        # Tank-fill fraction = how much oil is left in the tank = inverse of
        # bladder fill.
        frac = (self.bladder_max_m3 - self.latest_volume_m3) / span if span > 0 else 0.0
        frac = max(0.0, min(1.0, frac))
        p_gauge = self.tank_pressure_empty_pa + frac * (
            self.tank_pressure_full_pa - self.tank_pressure_empty_pa
        )
        return int(p_gauge)

    def publish_at_rate(self):
        # Tank pressure gets the sensor-noise chain per published tick
        # (fresh ADC read);
        self.bcu_pressure_pub.publish(
            Int32(data=self.tank_noise.apply_int(self.tank_pressure_pa()))
        )
        self.bcu_volume_pub.publish(Int32(data=self.latest_volume_ml))
        # Steady actuator-feedback heartbeats (echo of latest commanded state).
        self.feedback_rpm_pub.publish(Int16(data=self._last_rpm))
        self.feedback_valves_pub.publish(UInt8(data=self._last_valves))


def main(args=None):
    run_bridge(BCUSimBridge, args)


if __name__ == "__main__":
    main()
