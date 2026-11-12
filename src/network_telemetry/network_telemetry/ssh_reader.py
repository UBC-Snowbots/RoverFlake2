import json
import time
from typing import Any, List, Optional

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
except ImportError:
    rclpy = None
    Node = object
    String = None

try:
    from pexpect import pxssh
except ImportError:
    pxssh = None


class SshNetworkReaderNode(Node):
    """Read Ubiquiti JSON telemetry over SSH and publish it to network_reader."""

    def __init__(self) -> None:
        """Initialize SSH parameters, publisher, and polling timer."""
        super().__init__("ssh_network_reader")
        self._host = self.declare_parameter("host", "").value
        self._port = int(self.declare_parameter("port", 22).value)
        self._username = self.declare_parameter("username", "ubnt").value
        self._password = self.declare_parameter("password", "").value
        self._ssh_key = self.declare_parameter("ssh_key", "").value
        self._command = self.declare_parameter(
            "command", "cat /tmp/network.json"
        ).value
        self._timeout = float(self.declare_parameter("timeout_seconds", 10.0).value)
        interval = float(self.declare_parameter("poll_interval_seconds", 5.0).value)
        output_topic = self.declare_parameter("output_topic", "network/raw").value
        self._publisher = self.create_publisher(String, output_topic, 10)
        self._timer = self.create_timer(interval, self._poll)
        self._retries = int(self.declare_parameter("retries", 5).value)
        self._retry_delay = float(self.declare_parameter("retry_delay_seconds", 3.0).value)

    def _poll(self) -> None:
        """Fetch one JSON payload over SSH and publish it for normalization."""
        if not self._host:
            self.get_logger().error("SSH parameter 'host' must not be empty")
            return
        if pxssh is None:
            self.get_logger().error(
                "The pexpect package is required for the SSH reader"
            )
            return

        session = pxssh.pxssh(
            timeout=self._timeout,
            options={
                "StrictHostKeyChecking": "yes",
            },
        )
        try:
            login_kwargs = {
                "server": self._host,
                "username": self._username,
                "password": self._password,
                "port": self._port,
                "login_timeout": self._timeout,
            }
            if self._ssh_key:
                login_kwargs["ssh_key"] = self._ssh_key
            session.login(**login_kwargs)
            session.sendline(self._command)
            session.prompt(timeout=self._timeout)
            output = session.before.decode("utf-8", errors="replace")
            payload = self._json_payload(output)
            message = String()
            message.data = json.dumps(payload, separators=(",", ":"))
            self._publisher.publish(message)
        except (OSError, pxssh.ExceptionPxssh, json.JSONDecodeError) as exc:
            # Support the retries and retry delay parameters for transient errors, but log the error for visibility.
            self.get_logger().error(f"SSH read failed: {exc}")
            for attempt in range(self._retries):
                self.get_logger().info(f"Retrying... ({attempt + 1}/{self._retries})")
                time.sleep(self._retry_delay)
                try:
                    session.login(**login_kwargs)
                    session.sendline(self._command)
                    session.prompt(timeout=self._timeout)
                    output = session.before.decode("utf-8", errors="replace")
                    payload = self._json_payload(output)
                    message = String()
                    message.data = json.dumps(payload, separators=(",", ":"))
                    self._publisher.publish(message)
                    return  # Success, exit the retry loop
                except (OSError, pxssh.ExceptionPxssh, json.JSONDecodeError) as exc:
                    self.get_logger().error(f"SSH read failed on retry {attempt + 1}: {exc}")
        finally:
            if session:
                session.logout()

    @staticmethod
    def _json_payload(output: str) -> Any:
        """Extract and validate the JSON object or array from command output."""
        output = output.strip()
        try:
            return json.loads(output)
        except json.JSONDecodeError:
            start = min(
                (index for index in (output.find("{"), output.find("[")) if index >= 0),
                default=-1,
            )
            if start < 0:
                raise
            payload, _ = json.JSONDecoder().raw_decode(output[start:])
            return payload


def main(args: Optional[List[str]] = None) -> None:
    """Run the SSH network reader ROS node."""
    if rclpy is None:
        raise RuntimeError("ROS 2 is required to run SshNetworkReaderNode")
    rclpy.init(args=args)
    node = SshNetworkReaderNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
