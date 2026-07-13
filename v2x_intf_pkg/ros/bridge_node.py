"""ROS-to-UDP bridge for V2X messages."""

from rclpy.node import Node
from v2x_intf_msg.msg import Recognition

from v2x_intf_pkg.protocol.packet import PacketError
from v2x_intf_pkg.protocol.recognition import RecognitionCodec
from v2x_intf_pkg.transport import UdpTransport
from v2x_intf_pkg.wave import WaveMessage


class V2XBridgeNode(Node):
    """Publish received V2X packets and transmit subscribed ROS messages."""

    def __init__(self, remote_host: str, remote_port: int, local_port: int):
        super().__init__("v2x_bridge")
    
        self._transport = UdpTransport(remote_host, remote_port, local_port)
        self._wave = WaveMessage()
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
            self._transport.send(self._recog_codec.encode(message))
        except (OSError, TypeError, ValueError) as exc:
            self.get_logger().error(f"Failed to send Recognition: {exc}")


    def _proc_wave_msg(self, msg_id: int, msg_name: str, data: bytes) -> None:
        if msg_id == 41 : # SDSM 
            try:
                recognition_msg = self._recog_codec.decode(data)
                self._recog_publisher.publish(recognition_msg)
                self.get_logger().info(f"Published Recognition message from WAVE message ID {msg_id} ({msg_name})")
            except (TypeError, ValueError) as exc:
                self.get_logger().error(f"Failed to decode Recognition message: {exc}")


    def _receive_packets(self) -> None:
        # Bound work per timer call so a packet burst cannot starve ROS callbacks.
        for _ in range(100):
            try:
                data = self._transport.receive() 
                msg_id, msg_name, parsed_msg, error = self._wave.on_message(data)

                if msg_id is not None :
                    if msg_name is not None :
                        self.get_logger().info(f"Received WAVE message with ID {msg_id} ({msg_name})")
                        self._proc_wave_msg(msg_id, msg_name, parsed_msg)
                    else: # non-standard message, message id is available but name is not, we can still process it if possible
                        self.get_logger().info(f"Received non-standard message with ID {msg_id} (unknown type)")
                        # self._proc_own_msg(msg_id, "Unknown", data)
                else :
                    if error:
                        self.get_logger().error(error)

            except (OSError, PacketError, TypeError, ValueError) as exc:
                self.get_logger().error(f"Discarded invalid V2X packet: {exc}")

    def destroy_node(self):
        self._transport.close()
        return super().destroy_node()
