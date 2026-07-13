"""ROS-to-UDP bridge for V2X messages."""

from rclpy.node import Node
from v2x_intf_msg.msg import Recognition

from v2x_intf_pkg.protocol.packet import PacketError
from v2x_intf_pkg.protocol.recognition import RecognitionCodec
from v2x_intf_pkg.transport import UdpTransport


class V2XBridgeNode(Node):
    """Publish received V2X packets and transmit subscribed ROS messages."""

    def __init__(self, remote_host: str, remote_port: int, local_port: int):
        super().__init__("v2x_bridge")
        self._transport = UdpTransport(remote_host, remote_port, local_port)
        self._codec = RecognitionCodec()
        self._publisher = self.create_publisher(
            Recognition, "v2x/r_recognition", 10
        )
        self._subscription = self.create_subscription(
            Recognition, "v2x/recognition", self._send_recognition, 10
        )
        self._receive_timer = self.create_timer(0.01, self._receive_packets)
        self._transport.open()
        self.get_logger().info(
            f"V2X UDP bridge listening on {local_port}, "
            f"sending to {remote_host}:{remote_port}"
        )

    def _send_recognition(self, message: Recognition) -> None:
        try:
            self._transport.send(self._codec.encode(message))
        except (OSError, TypeError, ValueError) as exc:
            self.get_logger().error(f"Failed to send Recognition: {exc}")

    def _receive_packets(self) -> None:
        # Bound work per timer call so a packet burst cannot starve ROS callbacks.
        for _ in range(100):
            try:
                data = self._transport.receive()
                if data is None:
                    return
                self._publisher.publish(self._codec.decode(data))
            except (OSError, PacketError, TypeError, ValueError) as exc:
                self.get_logger().error(f"Discarded invalid V2X packet: {exc}")

    def destroy_node(self):
        self._transport.close()
        return super().destroy_node()
