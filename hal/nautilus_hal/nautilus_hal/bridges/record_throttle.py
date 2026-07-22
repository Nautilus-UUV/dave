"""Generic rate-limiter used only to slow topics down before bag recording.

The IMU sensor (50 Hz) and the gz odometry publisher (100 Hz) legitimately
need to run fast for the estimator and for GT comparison, so the live streams
stay untouched. This node subscribes to each input stream, caches the latest
message, and republishes it on a sibling topic at a fixed rate. The bag
recorder then points at the siblings, which keeps the recorded set uniform
with the 10 Hz BCU/external_sensor bridges.

ONE process hosts every throttled stream (parallel-array parameters). It
used to be one process per topic — 17 extra Python interpreters and DDS
participants per sim run, which under a many-slot sweep stampede pushed
Fast DDS into failed/minutes-late reader creation (the v2 campaign's
empty-channel and late-channel bags). Same wire behavior, one participant.

Output names are derived, not configured: the sibling topic is
``constants.throttled(input)``, the one rule ``bridge.launch.py`` and the
gate's recorder sentinels also use.

Parameters:
    input_topics = topics to subscribe to (e.g. ``['/imu', ...]``)
    msg_types    = fully-qualified message types, e.g.
                   ``['sensor_msgs/msg/Imu', ...]`` (parallel to the above)
    rate_hz      = shared output publish rate
"""

from importlib import import_module

from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from ..constants import throttled
from .bridge_base import SimBridgeNode, run_bridge


def _resolve_msg_type(type_str: str):
    pkg, kind, name = type_str.split("/")
    module = import_module(f"{pkg}.{kind}")
    return getattr(module, name)


class _Throttle:
    """One input->output stream: cache the latest message, publish on tick."""

    def __init__(self, node, input_topic, output_topic, msg_type, sub_qos):
        self._latest = None
        self._sub = node.create_subscription(
            msg_type, input_topic, self._on_msg, sub_qos
        )
        self._pub = node.create_publisher(msg_type, output_topic, 10)

    def _on_msg(self, msg):
        self._latest = msg

    def tick(self):
        if self._latest is not None:
            self._pub.publish(self._latest)


class RecordThrottle(SimBridgeNode):
    def __init__(self):
        super().__init__("nautilus_record_throttle")

    def setup_bridges(self):
        self.declare_parameter("input_topics", [""])
        self.declare_parameter("msg_types", [""])
        self.declare_parameter("rate_hz", 10.0)

        input_topics = [t for t in self.get_parameter("input_topics").value if t]
        msg_types = [t for t in self.get_parameter("msg_types").value if t]
        rate_hz = float(self.get_parameter("rate_hz").value)

        if not input_topics or len(input_topics) != len(msg_types):
            raise ValueError(
                "record_throttle requires equal-length non-empty input_topics "
                "and msg_types"
            )

        # Subscribe BEST_EFFORT so we match both best-effort sensor publishers
        # (e.g. /imu on SENSOR_STREAM) and reliable ones — DDS allows a
        # best-effort request to bind to a reliable offer, but not the reverse.
        # Depth 1: every consumer reads only `_latest`, so a deeper queue just
        # makes Python deserialize backlogged samples it is about to discard —
        # and this node runs alongside 60+ sim slots, where it WILL fall behind.
        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._throttles = [
            _Throttle(self, inp, throttled(inp), _resolve_msg_type(mt), sub_qos)
            for inp, mt in zip(input_topics, msg_types)
        ]
        self._timer = self.create_timer(1.0 / rate_hz, self._tick)

        self.get_logger().info(
            f"record_throttle: {len(self._throttles)} streams @ {rate_hz} Hz "
            f"({', '.join(input_topics)})"
        )

    def _tick(self):
        for throttle in self._throttles:
            throttle.tick()


def main(args=None):
    run_bridge(RecordThrottle, args)


if __name__ == "__main__":
    main()
