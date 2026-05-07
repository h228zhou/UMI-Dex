"""ROS2 node: publish raw CAN frames from SocketCAN as umi_dex_msgs/CanFrame.

No assembly, no filtering, no calibration — those happen offline
in the Python pipeline.
"""

import socket
import struct

import rclpy
from rclpy.node import Node

from umi_dex_msgs.msg import CanFrame

CAN_FRAME_FMT = "=IB3x8s"  # can_id (4), dlc (1), pad (3), data (8)
CAN_FRAME_SIZE = struct.calcsize(CAN_FRAME_FMT)


def open_can_socket(channel: str) -> socket.socket:
    sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    sock.settimeout(0.1)
    sock.bind((channel,))
    return sock


class CanRawNode(Node):
    def __init__(self) -> None:
        super().__init__("can_raw")
        self.declare_parameter("can_channel", "can0")
        self._channel = self.get_parameter("can_channel").value
        self._pub = self.create_publisher(CanFrame, "/hand/can_raw", 50)

    def run(self) -> None:
        self.get_logger().info("Opening SocketCAN on %s" % self._channel)
        try:
            sock = open_can_socket(self._channel)
        except OSError as e:
            self.get_logger().fatal(
                "Cannot open CAN socket on %s: %s" % (self._channel, e)
            )
            return

        self.get_logger().info("CAN raw publisher started on %s" % self._channel)
        try:
            while rclpy.ok():
                try:
                    raw = sock.recv(CAN_FRAME_SIZE)
                except socket.timeout:
                    continue
                if len(raw) < CAN_FRAME_SIZE:
                    continue

                can_id, dlc, data = struct.unpack(CAN_FRAME_FMT, raw)
                arb_id = can_id & 0x1FFFFFFF

                msg = CanFrame()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.arb_id = arb_id
                msg.dlc = dlc
                msg.data = list(data)
                self._pub.publish(msg)
        finally:
            sock.close()
            self.get_logger().info("CAN raw publisher stopped")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CanRawNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
