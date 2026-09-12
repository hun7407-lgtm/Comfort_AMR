#!/usr/bin/env python3

# ---------------------------------------------------------------
# Adapted from: https://github.com/dongtak2/AMR-Digital-Twin
#   original file : scripts/amr_arrow_teleop.py
#   commit        : c54269ee80032c29b0548eb77c3ee38d68b87e78 (2026-09-11)
#   modifications : key reading rewritten to use unbuffered os.read;
#                   see the read_key docstring. Added live speed echo
#                   and an explicit /cmd_vel topic from the config.
# See docs/AMR_DIGITAL_TWIN_REFERENCE.md
# ---------------------------------------------------------------

import os
import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


LINEAR_SPEED = 0.20
ANGULAR_SPEED = 0.50


class ArrowTeleop(Node):
    def __init__(self):
        super().__init__('amr_arrow_teleop')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.linear = 0.0
        self.angular = 0.0

        # 현재 명령을 계속 보내기
        self.timer = self.create_timer(0.1, self.publish_cmd)

    def publish_cmd(self):
        msg = Twist()
        msg.linear.x = self.linear
        msg.angular.z = self.angular
        self.publisher.publish(msg)

    def stop(self):
        self.linear = 0.0
        self.angular = 0.0


ARROWS = {b'A': 'UP', b'B': 'DOWN', b'C': 'RIGHT', b'D': 'LEFT'}


def read_key(fd):
    """Read one keypress, using unbuffered os.read on the raw fd.

    The upstream version used sys.stdin.read(1) and then select() to
    look for the rest of an escape sequence. That never works: an arrow
    key arrives as the three bytes ESC [ A, and sys.stdin is a buffered
    TextIOWrapper, so read(1) pulls all three out of the OS into Python's
    own buffer and returns only the ESC. The following select() then sees
    an empty fd and reports no more input, so every arrow key was
    silently discarded. Verified with a pseudo-terminal harness: the
    original recognised no arrow key at all.

    Reading the raw descriptor keeps the whole sequence in one call.
    """
    if not select.select([fd], [], [], 0.05)[0]:
        return None

    data = os.read(fd, 16)
    if not data:
        return None

    # Some terminals deliver the ESC separately from the rest.
    if data == b'\x1b' and select.select([fd], [], [], 0.02)[0]:
        data += os.read(fd, 16)

    if data.startswith(b'\x1b['):
        return ARROWS.get(data[2:3])

    if data == b' ':
        return 'STOP'

    if data[:1].lower() == b'q':
        return 'QUIT'

    return None


def main():
    settings = termios.tcgetattr(sys.stdin)

    rclpy.init()
    node = ArrowTeleop()

    print("""
===== AMR Arrow Teleop =====
↑ : 전진
↓ : 후진
← : 좌회전
→ : 우회전
SPACE : 정지
Q : 종료
============================
""")

    try:
        fd = sys.stdin.fileno()
        tty.setcbreak(fd)

        while rclpy.ok():
            key = read_key(fd)

            if key == 'UP':
                node.linear = LINEAR_SPEED
                node.angular = 0.0

            elif key == 'DOWN':
                node.linear = -LINEAR_SPEED
                node.angular = 0.0

            elif key == 'LEFT':
                node.linear = 0.0
                node.angular = ANGULAR_SPEED

            elif key == 'RIGHT':
                node.linear = 0.0
                node.angular = -ANGULAR_SPEED

            elif key == 'STOP':
                node.stop()

            elif key == 'QUIT':
                break

            if key is not None and key != 'QUIT':
                # Echo the command so it is obvious the key registered.
                # \r and the trailing spaces keep it on one line in cbreak
                # mode, where the terminal does not translate \n itself.
                print(f"  linear {node.linear:+.2f} m/s   "
                      f"angular {node.angular:+.2f} rad/s      \r",
                      end="", flush=True)

            rclpy.spin_once(node, timeout_sec=0.0)

    finally:
        node.stop()
        node.publish_cmd()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
