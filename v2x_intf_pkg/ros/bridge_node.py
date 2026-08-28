"""ROS-to-UDP bridge for V2X messages."""

from rclpy.node import Node
from v2x_intf_msg.msg import Recognition

from v2x_core.protocol import RecognitionCodec
from v2x_core.protocol.v2xmsg import V2XMessage
from v2x_core.transport import UdpTransport
from v2x_intf_pkg.ros.recognition_adapter import (
    recognition_from_ros,
    recognition_to_ros,
)


class V2XBridgeNode(Node):
    """Publish received V2X packets and transmit subscribed ROS messages."""

    def __init__(self, remote_host: str, remote_port: int, local_port: int):
        super().__init__("v2x_bridge")

        self._transport = UdpTransport(remote_host, remote_port, local_port)
        self._v2xmsg = V2XMessage()
        self._recog_codec = RecognitionCodec()
        self._recog_publisher = self.create_publisher(
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
            wave_message = self._recog_codec.encode(
                recognition_from_ros(message)
            )
            self._transport.send(self._v2xmsg.pack_ifm_message(wave_message))
        except (OSError, RuntimeError, TypeError, ValueError) as exc:
            self.get_logger().error(f"Failed to send Recognition: {exc}")

    def _proc_v2x_msg(self, msg_id: int, msg_name: str, data: bytes) -> None:
        if msg_id is None:
            self.get_logger().error("Received Custom(Non-SAE) V2X packet")
            return
        elif msg_id == 41:  # SDSM
            try:
                recognition_msg = recognition_to_ros(
                    self._recog_codec.decode(data)
                )
                self._recog_publisher.publish(recognition_msg)
                self.get_logger().info(
                    f"Published Recognition from V2X Msg ID {msg_id} ({msg_name})"
                )
            except (RuntimeError, TypeError, ValueError) as exc:
                self.get_logger().error(f"Failed to decode Recognition message: {exc}")


    def _receive_packets(self) -> None:
        # Bound work per timer call so a packet burst cannot starve ROS callbacks.
        for _ in range(100):
            try:
                data = self._transport.receive()
                if data is None:
                    return
                self._v2xmsg.on_message(data, self._proc_v2x_msg)

            except (OSError, RuntimeError, TypeError, ValueError) as exc:
                self.get_logger().error(f"Discarded invalid V2X packet: {exc}")

    def destroy_node(self):
        self._transport.close()
        return super().destroy_node()
