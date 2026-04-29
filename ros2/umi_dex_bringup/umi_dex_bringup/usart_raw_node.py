"""ROS2 node: publish raw USART encoder frames as umi_dex_msgs/UsartFrame.

Firmware emits a 16-byte framed packet per sample:
  55 AA | valid_mask(1) | 6x(lo, hi12) | checksum(1)

No in-node decoding to radians — that happens offline (or, eventually,
in firmware once calibration moves there).
"""

import time

import rclpy
import serial
from rclpy.node import Node

from umi_dex_msgs.msg import UsartFrame

SOF0 = 0x55
SOF1 = 0xAA
FRAME_LEN = 16
NUM_CHANNELS = 6


def _parse_frame(frame: bytes):
    """Return (raw_counts, valid_mask) if checksum ok, else None."""
    if (sum(frame[:-1]) & 0xFF) != frame[-1]:
        return None
    valid_mask = frame[2]
    raw = []
    p = 3
    for _ in range(NUM_CHANNELS):
        lo = frame[p]
        hi = frame[p + 1] & 0x0F
        raw.append(((hi << 8) | lo) & 0x0FFF)
        p += 2
    return raw, valid_mask


class UsartRawNode(Node):
    def __init__(self) -> None:
        super().__init__("usart_raw")
        self.declare_parameter("usart_port", "/dev/ttyUSB0")
        self.declare_parameter("usart_baud", 115200)
        self.declare_parameter("reconnect_backoff_s", 1.0)

        self._port: str = self.get_parameter("usart_port").value
        self._baud: int = int(self.get_parameter("usart_baud").value)
        self._backoff: float = float(
            self.get_parameter("reconnect_backoff_s").value
        )
        self._pub = self.create_publisher(UsartFrame, "/hand/usart_raw", 50)

    def run(self) -> None:
        while rclpy.ok():
            try:
                ser = serial.Serial(self._port, self._baud, timeout=0.05)
            except serial.SerialException as e:
                self.get_logger().error(
                    "Cannot open %s@%d: %s; retrying in %.1fs"
                    % (self._port, self._baud, e, self._backoff)
                )
                time.sleep(self._backoff)
                continue

            self.get_logger().info(
                "USART raw publisher on %s@%d" % (self._port, self._baud)
            )
            try:
                self._read_loop(ser)
            except serial.SerialException as e:
                self.get_logger().warning(
                    "Serial error on %s: %s; reconnecting"
                    % (self._port, e)
                )
                time.sleep(self._backoff)
            finally:
                try:
                    ser.close()
                except Exception:
                    pass

        self.get_logger().info("USART raw publisher stopped")

    def _read_loop(self, ser: serial.Serial) -> None:
        while rclpy.ok():
            b = ser.read(1)
            if not b:
                continue
            if b[0] != SOF0:
                continue

            # Stamp immediately: SOF0 has just arrived, frame starts here.
            stamp = self.get_clock().now().to_msg()

            b2 = ser.read(1)
            if not b2 or b2[0] != SOF1:
                continue

            rest = ser.read(FRAME_LEN - 2)
            if len(rest) != FRAME_LEN - 2:
                continue

            parsed = _parse_frame(bytes([SOF0, SOF1]) + rest)
            if parsed is None:
                continue

            raw, valid_mask = parsed
            msg = UsartFrame()
            msg.header.stamp = stamp
            msg.raw = raw
            msg.valid_mask = valid_mask
            self._pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UsartRawNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
