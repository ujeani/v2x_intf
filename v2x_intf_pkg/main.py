"""Executable entry point for the V2X ROS bridge."""

import argparse

import rclpy
from rclpy.utilities import remove_ros_args

from v2x_intf_pkg import config
from v2x_intf_pkg.ros import V2XBridgeNode


def _parse_arguments(args=None):
    parser = argparse.ArgumentParser(description="V2X OBU UDP interface")
    parser.add_argument("--obu-ip", default=config.DEFAULT_OBU_IP)
    parser.add_argument("--obu-port", type=int, default=config.DEFAULT_OBU_PORT)
    parser.add_argument(
        "--local-port", type=int, default=config.DEFAULT_LOCAL_PORT
    )
    application_args = remove_ros_args(args=args)
    if args is None:
        application_args = application_args[1:]
    return parser.parse_args(application_args)


def main(args=None):
    options = _parse_arguments(args)
    rclpy.init(args=args)
    node = None
    try:
        node = V2XBridgeNode(
            options.obu_ip,
            options.obu_port,
            options.local_port,
        )
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
