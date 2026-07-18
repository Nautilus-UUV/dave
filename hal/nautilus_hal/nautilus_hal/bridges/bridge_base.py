"""Shared base for the Nautilus HAL sim bridges.

Each concrete bridge declares the parameters it actually consumes via
``self.declare_parameter`` in ``setup_bridges``; defaults come from
``py_pkg.scenarios.spec.rig`` and ultimately from
``py_pkg.robot_specs``. The standalone ``ros2 run`` path therefore
matches today's behaviour without needing a YAML overlay.

Centralising the SIGINT-tolerant spin loop here is what
``run_bridge`` does.
"""

from typing import Type

import rclpy
from py_pkg.scenarios.spec.rig import CommsFaultSpec, PressureNoiseSpec, SensorFaultSpec
from py_pkg.sensor_faults import FaultyChannel, MessageDrop, gate_publisher
from py_pkg.sensor_noise import GaussianQuantizedNoise, rng_from_seed
from py_pkg.uuv_ros_core import create_publisher_for_topic
from rclpy.exceptions import InvalidHandle
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class SimBridgeNode(Node):
    """Template Method base — subclasses declare params in setup_bridges."""

    def __init__(self, node_name: str) -> None:
        super().__init__(node_name)
        self.setup_bridges()

    def setup_bridges(self) -> None:
        """Subclass hook: declare parameters, publishers, subscribers, timers."""
        raise NotImplementedError

    def declare_pressure_noise(
        self, prefix: str, default: PressureNoiseSpec
    ) -> GaussianQuantizedNoise:
        """Declare one pressure channel's noise params, return its model.

        Declares ``<prefix>_seed`` / ``<prefix>_sigma_pa`` /
        ``<prefix>_quantization_pa`` (defaults from the spec, so bare
        ``ros2 run`` behaves like nominal; the scenario compiler injects
        a derived seed) and builds the ``GaussianQuantizedNoise`` the
        bridge applies per published tick.
        """
        self.declare_parameter(f"{prefix}_seed", 0)
        self.declare_parameter(f"{prefix}_sigma_pa", default.sigma_pa)
        self.declare_parameter(f"{prefix}_quantization_pa", default.quantization_pa)
        return GaussianQuantizedNoise(
            sigma=self.get_parameter(f"{prefix}_sigma_pa").value,
            quantization_step=self.get_parameter(f"{prefix}_quantization_pa").value,
            rng=rng_from_seed(self.get_parameter(f"{prefix}_seed").value),
        )

    def declare_sensor_fault(
        self, prefix: str, noise: GaussianQuantizedNoise
    ) -> tuple[FaultyChannel, MessageDrop]:
        """Declare one channel's persistent sensor-fault params, return models.

        Declares ``<prefix>fault_kind`` / ``<prefix>fault_magnitude`` /
        ``<prefix>fault_drop_prob`` / ``<prefix>fault_seed`` (prefix is
        ``"tank_"`` on the BCU bridge, ``""`` on the external bridge —
        mirroring ``compile._sensor_fault_params``; defaults mean no
        fault). ``kind == "dropout"`` maps to a MessageDrop consulted at
        this channel's publish site; every other kind builds a
        FaultyChannel composed with the channel's calibrated noise chain.
        """
        default = SensorFaultSpec()
        self.declare_parameter(f"{prefix}fault_kind", default.kind)
        self.declare_parameter(f"{prefix}fault_magnitude", default.magnitude)
        self.declare_parameter(f"{prefix}fault_drop_prob", default.drop_prob)
        self.declare_parameter(f"{prefix}fault_seed", 0)
        kind = self.get_parameter(f"{prefix}fault_kind").value
        magnitude = self.get_parameter(f"{prefix}fault_magnitude").value
        drop_prob = self.get_parameter(f"{prefix}fault_drop_prob").value
        rng = rng_from_seed(self.get_parameter(f"{prefix}fault_seed").value)
        if kind == "dropout":
            channel = FaultyChannel(noise, kind="none")
            drop = MessageDrop(p=drop_prob, rng=rng)
        else:
            channel = FaultyChannel(noise, kind=kind, magnitude=magnitude)
            drop = MessageDrop()
        if kind != "none":
            self.get_logger().warn(
                f"persistent sensor fault active on '{prefix}fault_*':"
                f" kind={kind} magnitude={magnitude} drop_prob={drop_prob}"
            )
        return channel, drop

    def declare_comms_drop(self) -> MessageDrop:
        """Declare the uniform comms-fault params, arm the shared gate.

        One seeded per-message Bernoulli gate per bridge, stashed on
        ``self`` for ``create_bridged_publisher`` so a degraded link
        loses frames uniformly across the bridge's streams.
        """
        default = CommsFaultSpec()
        self.declare_parameter("comms_drop_prob", default.drop_prob)
        self.declare_parameter("comms_seed", 0)
        drop_prob = self.get_parameter("comms_drop_prob").value
        gate = MessageDrop(
            p=drop_prob, rng=rng_from_seed(self.get_parameter("comms_seed").value)
        )
        if gate.is_active:
            self.get_logger().warn(
                f"comms fault active: drop_prob={drop_prob} on every bridged publish"
            )
        self._comms_drop = gate
        return gate

    def create_bridged_publisher(self, topic):
        """Registry publisher for one bridged (Pi<->STM-emulating) stream.

        Composes ``create_publisher_for_topic`` with this bridge's
        shared comms-drop gate (``declare_comms_drop`` must run first),
        so every bridged telemetry stream is comms-gated by default —
        an ungated stream has to opt out visibly with a plain
        ``create_publisher``. Gazebo-facing plant publishers and the
        provenance/label streams are exactly those opt-outs.
        ``gate_publisher`` returns the raw publisher when the gate is
        inactive, so the nominal path has no interception layer.
        """
        return gate_publisher(create_publisher_for_topic(self, topic), self._comms_drop)


def run_bridge(node_cls: Type[SimBridgeNode], args=None) -> None:
    """SIGINT/SIGTERM-tolerant entrypoint shared by every bridge ``main``.

    Without this, launch_testing's exit-code check intermittently fails
    on Ctrl-C. ``InvalidHandle`` is also swallowed: when SIGTERM arrives
    while a timer or subscription callback is mid-publish, the
    publisher's underlying handle can be torn down before the publish
    completes. The bridge with the busiest shutdown window
    (bcu_sim_bridge — 10 Hz pub_timer plus rpm_callback both publishing)
    hit this often enough to flake the test.

    ``RuntimeError`` is also caught: rclpy.executors._take_subscription
    calls into pybind11's ``handle.take_message`` while the executor
    still has a pending wait on a subscription whose handle is being
    torn down by the SIGINT path. pybind11 raises a bare RuntimeError
    ("Unable to convert call argument '0' to Python object") which
    otherwise leaks out as exit code 1.
    """
    rclpy.init(args=args)
    node = node_cls()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, InvalidHandle, RuntimeError):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
