from py_pkg.plant_dynamics import PumpDynamics, make_tank_pressure_map, pump_flow_active
from py_pkg.scenarios.spec.rig import (
    BcuBridgeSpec,
    BcuPumpFaultSpec,
    NoiseSpec,
    PlantSpec,
    SimSpec,
)
from py_pkg.uuv_ros_core import UUVTopics, create_subscription_for_topic
from std_msgs.msg import Float32, Float64, Int16, Int32, UInt8

from ..constants import Conversions, SimDebugTopics, SimTopics
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
        fault_def = BcuPumpFaultSpec()
        bridge_def = BcuBridgeSpec()

        self.declare_parameter("model_name", sim_def.model_name)
        self.declare_parameter("volume_per_rev_m3", plant_def.volume_per_rev_m3)
        self.declare_parameter("bladder_min_m3", plant_def.bladder_min_m3)
        self.declare_parameter("bladder_max_m3", plant_def.bladder_max_m3)
        self.declare_parameter(
            "tank_pressure_empty_pa", plant_def.tank_pressure_empty_pa
        )
        self.declare_parameter("tank_pressure_full_pa", plant_def.tank_pressure_full_pa)
        self.declare_parameter("pump_response_delay_s", plant_def.pump_response_delay_s)
        self.declare_parameter("pump_slew_rpm_per_s", plant_def.pump_slew_rpm_per_s)
        self.declare_parameter("pump_overshoot_frac", plant_def.pump_overshoot_frac)
        self.declare_parameter("tank_map_shape", plant_def.tank_map_shape)
        self.declare_parameter("tank_air_volume_m3", plant_def.tank_air_volume_m3)
        self.declare_parameter("fault_effectiveness", fault_def.effectiveness)
        self.declare_parameter("publish_rate_hz", bridge_def.publish_rate_hz)

        self.model_name = self.get_parameter("model_name").value
        self.volume_per_rev_m3 = self.get_parameter("volume_per_rev_m3").value
        self.bladder_min_m3 = self.get_parameter("bladder_min_m3").value
        self.bladder_max_m3 = self.get_parameter("bladder_max_m3").value
        self.tank_pressure_empty_pa = self.get_parameter("tank_pressure_empty_pa").value
        self.tank_pressure_full_pa = self.get_parameter("tank_pressure_full_pa").value
        # Tank sensor curve, bound once at startup: shape dispatch, the
        # free-cushion validity check, and the <= 0 = pinned-cushion
        # convention all live in make_tank_pressure_map, which fails here
        # (not on the first telemetry tick) for a bad configuration.
        self._tank_pressure = make_tank_pressure_map(
            self.get_parameter("tank_map_shape").value,
            self.bladder_min_m3,
            self.bladder_max_m3,
            self.tank_pressure_empty_pa,
            self.tank_pressure_full_pa,
            air_volume_m3=self.get_parameter("tank_air_volume_m3").value,
        )
        # Commanded -> effective shaft RPM (lake-fitted dead time + slew;
        # both <= 0 == passthrough). Plant truth, applied before the flow
        # integral AND the feedback echo, so sim feedback shows spin-up
        # exactly like the real STM's shaft-speed report.
        self.pump_dynamics = PumpDynamics(
            delay_s=self.get_parameter("pump_response_delay_s").value,
            slew_rpm_per_s=self.get_parameter("pump_slew_rpm_per_s").value,
            overshoot_frac=self.get_parameter("pump_overshoot_frac").value,
        )
        # Live bladder fill (m3), refreshed from Gazebo. Until the first echo
        # arrives we report the empty endpoint by sitting at the operating min.
        self.latest_volume_m3 = self.bladder_min_m3

        # Seeded from Gazebo's first BUOYANCY_VOLUME_STATE callback (see
        # sim_bcu_volume_callback). Until then we don't push a volume back
        # to Gazebo — pushing 0.0 + delta would clobber the SDF-initialized
        # bladder state on the first RPM tick.
        self.current_volume = None

        # ====================
        # Fault injection (persistent per-run, one drawn severity)
        # ====================
        # Pump degradation: the commanded RPM is scaled by the drawn
        # effectiveness, gated in time by the fault schedule (fault_onset_s
        # / _shape / _ramp_s / _period_s / _duty; defaults = felt from
        # t=0, the whole-run behavior). No RNG. Fail fast on a
        # nonsensical value (same convention as make_tank_pressure_map).
        self.fault_effectiveness = float(
            self.get_parameter("fault_effectiveness").value
        )
        if not (0.0 < self.fault_effectiveness <= 1.0):
            raise ValueError(
                f"fault_effectiveness must be in (0, 1], got {self.fault_effectiveness}"
            )
        if self.fault_effectiveness != 1.0:
            self.get_logger().warn(
                "persistent pump fault active:"
                f" effectiveness={self.fault_effectiveness}"
            )
        self.fault_schedule = self.declare_fault_schedule("fault_")
        # Fault telemetry: the effectiveness currently felt by the plant
        # (time-varying under a schedule; pre-onset it reads 1.0 — run
        # severity lives in the scenario/manifest, not this stream's
        # first sample). Same topic the old ladder used (raw publisher —
        # a sim-debug channel, deliberately outside the uuv_ros_core
        # registry). Never gated (provenance must stay truthful).
        self.fault_pub = self.create_publisher(
            Float32, SimDebugTopics.BCU_PUMP_FAULT, 10
        )
        self._current_effectiveness = 1.0

        # ====================
        # Tank-pressure sensor noise + persistent sensor fault
        # ====================
        self.tank_noise = self.declare_pressure_noise(
            "tank_noise", NoiseSpec().tank_pressure
        )
        # tank_fault_kind/_magnitude/_drop_prob/_seed: bias/drift/stuck
        # wrap the noise chain; dropout becomes a per-message gate on the
        # /bcu/pressure publish alone (siblings keep publishing).
        self.tank_fault, self.tank_drop = self.declare_sensor_fault(
            "tank_", self.tank_noise
        )

        # ====================
        # Comms fault: one shared per-message drop gate over every
        # bridged telemetry publisher (uniform frame loss, like a
        # degraded Pi<->STM link) — create_bridged_publisher below wires
        # each one through it. The Gazebo-facing plant path and the
        # fault-telemetry publisher above are deliberately NOT gated.
        # ====================
        self.declare_comms_drop()

        # ====================
        # BCU RPM/flow control
        # ====================
        self.last_time = self.get_clock().now()
        # Latest commanded RPM, held between messages (STM semantics: a
        # command stands until superseded, silence doesn't stop the pump).
        self._commanded_rpm = 0.0
        self.rpm_sub = create_subscription_for_topic(
            self, UUVTopics.BCU_RPM, self.rpm_callback
        )
        self.flow_pub = self.create_bridged_publisher(UUVTopics.BCU_FLOW_RATE)

        # ====================
        # Actuator feedback ("ping back")
        # ====================
        self._last_rpm = 0
        self._last_valves = 0
        self.valves_sub = create_subscription_for_topic(
            self, UUVTopics.BCU_VALVES, self.valves_callback
        )
        self.feedback_rpm_pub = self.create_bridged_publisher(
            UUVTopics.BCU_FEEDBACK_RPM
        )
        self.feedback_valves_pub = self.create_bridged_publisher(
            UUVTopics.BCU_FEEDBACK_VALVES
        )

        # ==============
        # BCU pressure / volume telemetry
        # ==============
        # Gazebo's buoyancy plugin only exposes bladder *volume*, so the tank
        # pressure sensor is synthesized from the fill state
        self.latest_volume_ml = 0  # bladder volume (mL), echoed from Gazebo
        publish_rate_hz = self.get_parameter("publish_rate_hz").value
        self.pub_timer = self.create_timer(1.0 / publish_rate_hz, self.publish_at_rate)
        self.bcu_pressure_pub = self.create_bridged_publisher(UUVTopics.BCU_PRESSURE)
        self.bcu_volume_pub = self.create_bridged_publisher(UUVTopics.BCU_VOLUME)
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
        # Cache only — the plant steps on the publish timer, so the
        # transient keeps evolving after the last message (regression:
        # test_bcu_bridge_feedback_decay).
        self._commanded_rpm = float(msg.data)

    def valves_callback(self, msg):
        # Cache the commanded valve bitmask (bit0=valve2/motor way,
        # bit1=valve1/free way — see robot_specs) for the steady feedback
        # echo and the transfer gate in publish_at_rate.
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
        # tank, so tank pressure runs INVERSE to bladder fill. Curve shape
        # ("linear" legacy oil map / "gaslaw" lake-fitted air-cushion
        # hyperbola), endpoints, and the optional free cushion volume are ROS
        # params, resolved once into self._tank_pressure at startup.
        return int(self._tank_pressure(self.latest_volume_m3))

    def _step_plant(self):
        # Advance the plant every timer tick against the held command:
        # fault scaling, the dead-time/slew transient, the volume
        # integral, and the flow echo all keep evolving (spin-up AND
        # spin-down) between and after command messages.
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        # Schedule-gated effectiveness: healthy (1.0) until the onset fires,
        # then the drawn severity, blended by the envelope m(t).
        self._current_effectiveness = self.fault_schedule.blend(
            1.0, self.fault_effectiveness, now.nanoseconds / 1e9
        )
        rpm_cmd = self._commanded_rpm * self._current_effectiveness
        # Plant transient: dead time + slew between the (fault-adjusted)
        # command and what the shaft actually does.
        eff_rpm = self.pump_dynamics.step(now.nanoseconds / 1e9, rpm_cmd, dt)
        # Effective rpm for the feedback echo — the real STM reports
        # shaft speed, so the echo shows the spin-up/spin-down ramps.
        self._last_rpm = int(round(eff_rpm))

        # Hydraulic gate (pump_flow_active): the pump only carries flow
        # through valve 2 (the motor way). Closed valve = deadhead — the
        # shaft still spins (the feedback echo stays live) but no oil
        # moves, like hardware. Valve 1 (the passive free/bypass way) is
        # NOT modeled — a plant no-op in sim. rpm -> rps -> total revs
        # in dt -> volume change.
        rps = (eff_rpm / 60.0) if pump_flow_active(eff_rpm, self._last_valves) else 0.0
        flow_m3_per_s = rps * self.volume_per_rev_m3

        # Only integrate + push to Gazebo once we've synced to its SDF-
        # initialized bladder volume; otherwise we'd overwrite the initial
        # state (the flow echo stays open-loop until the sync). Once
        # synced, the echo reports the APPLIED flow: at a bladder rail the
        # volume clamp freezes the fill, so the pump deadheads — shaft
        # spinning, zero oil moved — and the flow echo must say so.
        if self.current_volume is not None:
            prev_volume = self.current_volume
            self.current_volume = max(
                self.bladder_min_m3,
                min(prev_volume + flow_m3_per_s * dt, self.bladder_max_m3),
            )

            out_msg = Float64()
            out_msg.data = self.current_volume
            self.sim_volume_pub.publish(out_msg)

            flow_m3_per_s = (
                (self.current_volume - prev_volume) / dt if dt > 0.0 else 0.0
            )

        # Mock Flow Rate feedback (m3/s)
        flow_msg = Float32()
        flow_msg.data = flow_m3_per_s
        self.flow_pub.publish(flow_msg)

    def publish_at_rate(self):
        # The publish timer doubles as the plant clock (see
        # BcuBridgeSpec.publish_rate_hz): step the plant, then publish
        # telemetry from the fresh state.
        self._step_plant()

        # Tank pressure gets the persistent-fault + sensor-noise chain
        # per published tick (fresh ADC read). A sensor dropout fault
        # suppresses only this stream's publish; the siblings below keep
        # their cadence (which is what distinguishes dropout from a
        # comms fault at the bridge level).
        if not self.tank_drop.should_drop(self.last_time.nanoseconds / 1e9):
            self.bcu_pressure_pub.publish(
                Int32(
                    data=self.tank_fault.sample_int(
                        self.tank_pressure_pa(),
                        self.last_time.nanoseconds / 1e9,
                    )
                )
            )
        self.bcu_volume_pub.publish(Int32(data=self.latest_volume_ml))
        # Steady actuator-feedback heartbeats (echo of the evolving plant).
        self.feedback_rpm_pub.publish(Int16(data=self._last_rpm))
        self.feedback_valves_pub.publish(UInt8(data=self._last_valves))
        # Actuator-fault provenance (the effectiveness felt this tick;
        # never gated).
        self.fault_pub.publish(Float32(data=self._current_effectiveness))


def main(args=None):
    run_bridge(BCUSimBridge, args)


if __name__ == "__main__":
    main()
