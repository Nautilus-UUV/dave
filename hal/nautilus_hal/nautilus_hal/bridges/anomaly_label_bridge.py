"""Sim-only per-timestamp ground-truth anomaly label bridge.

Broadcasts one ``nautilus_msgs/AnomalyLabel`` at a fixed 10 Hz on
``UUVTopics.ANOMALY_LABEL`` so that every timestamp in a recorded bag
is attributable to a class — including timestamps whose anomaly
signature is an *absence* (sensor dropout, comms drops). That is why
this is its own node: it is never routed through any fault or comms
gate, so the label stream stays intact precisely when the faulted
streams go quiet. Parameterized from ``Scenario.anomaly`` via
``compile.params_for_anomaly_label`` (the label block is validated
against ``rig.faults`` at load, so this bridge can never claim
"nominal" while a runtime fault is configured).

``anomaly_class`` / ``channel`` / ``archetype`` are constant for the
run; ``active`` marks when the fault is actually influencing published
data, so dataset builders can use run-level or timestamp-level labels
without label noise:

  nominal    -> always False (an explicit stream in every bag —
                downstream never infers from absence)
  bcu_pump   -> True iff the pump is actuating: last commanded
                /bcu/rpm != 0 AND the motor valve is open — mirroring
                the BCU bridge's flow condition. An idle degraded pump
                produces nominal-looking data. Approximation, by
                design: the ~1 s PumpDynamics decay tail after a
                command drops to 0 is not extended.
  sensor     -> True from the first sample. For drift this follows the
                FDI convention: the label marks the fault *condition*
                (present from t=0) even while the ramp is sub-noise,
                so detection delay is a measured property of the
                detector, not baked into the labels.
  comms      -> always True (drops are absences in the other streams).
  biofouling -> always True (physical hull state from spawn).

Not for hardware — labels are a property of the injected scenario.
"""

from nautilus_msgs.msg import AnomalyLabel
from py_pkg.plant_dynamics import pump_flow_active
from py_pkg.scenarios.spec.scenario import AnomalyLabelSpec
from py_pkg.uuv_ros_core import (
    UUVTopics,
    create_publisher_for_topic,
    create_subscription_for_topic,
)

from .bridge_base import SimBridgeNode, run_bridge

LABEL_RATE_HZ = 10.0


class AnomalyLabelBridge(SimBridgeNode):
    def __init__(self):
        super().__init__("nautilus_anomaly_label_bridge")

    def setup_bridges(self):
        self.declare_parameter("anomaly_class", "nominal")
        self.declare_parameter("channel", "")
        self.declare_parameter("archetype", "")
        # Round-trip through the owning spec: one vocabulary, and the
        # full cross-field validation (sensor needs channel+archetype,
        # nothing else may carry them) — fails fast on bad overrides.
        label = AnomalyLabelSpec(
            anomaly_class=self.get_parameter("anomaly_class").value,
            channel=self.get_parameter("channel").value,
            archetype=self.get_parameter("archetype").value,
        )
        self.anomaly_class = label.anomaly_class
        self.channel = label.channel
        self.archetype = label.archetype

        # bcu_pump activity gating taps the command direction (/bcu/rpm,
        # /bcu/valves are controller-published and never fault-gated), so
        # the label stays truthful even under a total comms fault.
        self._last_rpm_cmd = 0
        self._last_valves = 0
        if self.anomaly_class == "bcu_pump":
            self.rpm_sub = create_subscription_for_topic(
                self, UUVTopics.BCU_RPM, self._on_rpm
            )
            self.valves_sub = create_subscription_for_topic(
                self, UUVTopics.BCU_VALVES, self._on_valves
            )

        self.label_pub = create_publisher_for_topic(self, UUVTopics.ANOMALY_LABEL)
        # class/channel/archetype are run constants: build the message
        # once, refresh only stamp + active per tick.
        self._label_msg = AnomalyLabel(
            anomaly_class=self.anomaly_class,
            channel=self.channel,
            archetype=self.archetype,
        )
        self.pub_timer = self.create_timer(1.0 / LABEL_RATE_HZ, self._publish_label)

        if self.anomaly_class == "nominal":
            self.get_logger().info("anomaly label: nominal run")
        else:
            self.get_logger().warn(
                f"anomaly label: class={self.anomaly_class}"
                f" channel={self.channel or '-'} archetype={self.archetype or '-'}"
            )

    def _on_rpm(self, msg):
        self._last_rpm_cmd = int(msg.data)

    def _on_valves(self, msg):
        self._last_valves = int(msg.data)

    def _active(self) -> bool:
        if self.anomaly_class == "nominal":
            return False
        if self.anomaly_class == "bcu_pump":
            # Same hydraulic gate as the BCU bridge's flow integral,
            # fed with the commanded rpm (see module docstring).
            return pump_flow_active(self._last_rpm_cmd, self._last_valves)
        return True

    def _publish_label(self):
        msg = self._label_msg
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.active = self._active()
        self.label_pub.publish(msg)


def main(args=None):
    run_bridge(AnomalyLabelBridge, args)


if __name__ == "__main__":
    main()
