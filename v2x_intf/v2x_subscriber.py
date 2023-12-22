import rclpy as rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message, deserialize_message

from case_msgs.msg import Recognition, Objects
import socket

obu_ip = '192.168.2.100' # TODO : MATCH IT!
obu_port = 1000 # TODO : MATCH IT!

# Create a TCP socket
# client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Connect to the server (establishing the connection)
# client_socket.connect((obu_ip, obu_port))

class RecognitionSubscriber(Node):
    def __init__(self):
        super().__init__('recognition_subscriber')
        self.subscription = self.create_subscription(
            Recognition,
            'caselab/recognition',
            self.recognition_callback,
            10)
        self.subscription  # prevent unused variable warning

    def recognition_callback(self, msg):
        print("I heard:", msg)
        serialized_data = serialize_message(msg)
        # Print serialized msg
        # client_socket.send() #* message.encode())
        deserialized_msg = deserialize_message(Recognition, serialized_data)
        print("serialize->deserialize : ", deserialize_message)


def main(args=None):
    rclpy.init(args=args)
    recognition_subscriber = RecognitionSubscriber()
    rclpy.spin(recognition_subscriber)

    # client_socket.close()
    recognition_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
