import json
import time
from typing import Any, Dict, List, Optional, Union

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
except ImportError:
    rclpy = None
    Node = object
    String = None


JsonValue = Union[Dict[str, Any], List[Any], str, int, float, bool, None]


def _number(value: Any, default: Optional[float] = None) -> Optional[float]:
    """Return a numeric value, or default when the input is not numeric."""
    if isinstance(value, bool):
        return default
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _capacity_mbps(value: Any) -> Optional[float]:
    capacity = _number(value)
    return round(capacity / 1000.0, 3) if capacity is not None else None


def _counter_rate(
    current: Any, previous: Any, interval_seconds: Optional[float]
) -> Optional[float]:
    """Calculate a bytes/second rate from two monotonically increasing counters."""
    current_value = _number(current)
    previous_value = _number(previous)
    if (
        current_value is None
        or previous_value is None
        or interval_seconds is None
        or interval_seconds <= 0
        or current_value < previous_value
    ):
        return None
    return round((current_value - previous_value) / interval_seconds, 3)


def _link_telemetry(
    link: Dict[str, Any],
    previous: Optional[Dict[str, Any]],
    interval_seconds: Optional[float],
) -> Dict[str, Any]:
    """Build a compact, human-readable representation of one radio link."""
    stats = link.get("stats") or {}
    remote = link.get("remote") or {}
    previous_stats = (previous or {}).get("stats") or {}
    tx_bytes_per_second = _counter_rate(
        stats.get("tx_bytes"), previous_stats.get("tx_bytes"), interval_seconds
    )
    rx_bytes_per_second = _counter_rate(
        stats.get("rx_bytes"), previous_stats.get("rx_bytes"), interval_seconds
    )
    tx_packets = _number(stats.get("tx_packets"))
    retries = _number(link.get("tx_lretries"), 0) + _number(
        link.get("tx_sretries"), 0
    )

    return {
        "device": link.get("name") or link.get("ubntDish") or link.get("mac"),
        "mac": link.get("mac"),
        "ip": link.get("lastip"),
        "signal_dbm": _number(link.get("signal")),
        "rssi_db": _number(link.get("rssi")),
        "noise_floor_dbm": _number(link.get("noisefloor")),
        "snr_db": (
            round(_number(link.get("signal")) - _number(link.get("noisefloor")), 3)
            if _number(link.get("signal")) is not None
            and _number(link.get("noisefloor")) is not None
            else None
        ),
        "latency_ms": _number(link.get("tx_latency")),
        "ack_ms": _number(link.get("ack")),
        "distance_m": _number(link.get("distance")),
        "link_score": {
            "down": _number(link.get("dl_linkscore")),
            "up": _number(link.get("ul_linkscore")),
            "down_average": _number(link.get("dl_avg_linkscore")),
            "up_average": _number(link.get("ul_avg_linkscore")),
        },
        "capacity_mbps": {
            "down": _capacity_mbps(link.get("dl_capacity_expect")),
            "up": _capacity_mbps(link.get("ul_capacity_expect")),
            "channel": _capacity_mbps(link.get("cb_capacity_expect")),
        },
        "throughput_mbps": {
            "advertised_down": _number(link.get("dl_rate_expect")),
            "advertised_up": _number(link.get("ul_rate_expect")),
            "measured_down": (
                round(rx_bytes_per_second * 8 / 1_000_000, 3)
                if rx_bytes_per_second is not None
                else None
            ),
            "measured_up": (
                round(tx_bytes_per_second * 8 / 1_000_000, 3)
                if tx_bytes_per_second is not None
                else None
            ),
        },
        "traffic": {
            "rx_bytes": _number(stats.get("rx_bytes")),
            "tx_bytes": _number(stats.get("tx_bytes")),
            "rx_packets": _number(stats.get("rx_packets")),
            "tx_packets": tx_packets,
            "rx_packets_per_second": _number(stats.get("rx_pps")),
            "tx_packets_per_second": _number(stats.get("tx_pps")),
            "tx_retries": retries,
            "tx_retry_rate": (
                round(retries / tx_packets, 4)
                if tx_packets and tx_packets > 0
                else None
            ),
        },
        "remote": {
            "hostname": remote.get("hostname"),
            "platform": remote.get("platform"),
            "version": remote.get("version"),
            "cpu_load_percent": _number(remote.get("cpuload")),
            "temperature_c": _number(remote.get("temperature")),
            "uptime_seconds": _number(remote.get("uptime")),
        },
        "uptime_seconds": _number(link.get("uptime")),
    }


def normalize_network_data(
    payload: JsonValue,
    previous_payload: Optional[JsonValue] = None,
    interval_seconds: Optional[float] = None,
) -> Dict[str, Any]:
    if isinstance(payload, str):
        payload = json.loads(payload)
    if isinstance(previous_payload, str):
        previous_payload = json.loads(previous_payload)

    links = payload if isinstance(payload, list) else [payload]
    previous_links = (
        previous_payload
        if isinstance(previous_payload, list)
        else [previous_payload] if isinstance(previous_payload, dict) else []
    )
    normalized_links = [
        _link_telemetry(
            link,
            previous_links[index] if index < len(previous_links) else None,
            interval_seconds,
        )
        for index, link in enumerate(links)
        if isinstance(link, dict)
    ]
    return {
        "timestamp": time.time(),
        "link_count": len(normalized_links),
        "links": normalized_links,
    }


class NetworkTelemetryNode(Node):

    def __init__(self) -> None:
        super().__init__("network_telemetry")
        input_topic = self.declare_parameter("input_topic", "network/raw").value
        output_topic = self.declare_parameter(
            "output_topic", "network/telemetry"
        ).value
        self._previous_payload: Optional[JsonValue] = None
        self._previous_time: Optional[float] = None
        self._publisher = self.create_publisher(String, output_topic, 10)
        self.create_subscription(String, input_topic, self._on_network_data, 10)

    def _on_network_data(self, message: Any) -> None:
        """Normalize and publish one raw network message."""
        now = time.monotonic()
        interval = (
            now - self._previous_time if self._previous_time is not None else None
        )
        try:
            payload = json.loads(message.data)
            telemetry = normalize_network_data(
                payload, self._previous_payload, interval
            )
        except (json.JSONDecodeError, TypeError, ValueError) as exc:
            self.get_logger().error("Unable to parse network data: %s", exc)
            return

        output = String()
        output.data = json.dumps(telemetry, separators=(",", ":"))
        self._publisher.publish(output)
        self._previous_payload = payload
        self._previous_time = now


def main(args: Optional[List[str]] = None) -> None:
    """Run the network telemetry ROS node."""
    if rclpy is None:
        raise RuntimeError("ROS 2 is required to run NetworkTelemetryNode")
    rclpy.init(args=args)
    node = NetworkTelemetryNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()