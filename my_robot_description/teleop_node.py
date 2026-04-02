#!/usr/bin/env python3
"""
teleop_node.py  —  Keyboard teleoperation for differential drive robot
ROS2 Humble

Controls:
  W : Move Forward
  S : Move Backward
  A : Turn Left
  D : Turn Right
  SPACE  : Hard Stop
  CTRL-C : Quit

Hold keys to move. Release to stop.
"""

import sys
import select
import termios
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


BANNER = """
---------------------------
  DIFF DRIVE TELEOP
---------------------------
  W : Forward
  S : Backward
  A : Turn Left
  D : Turn Right

  SPACE  : Hard Stop
  CTRL-C : Quit
---------------------------
Hold keys to move. Release to stop.
"""

# Tunable constants
LINEAR_SPEED  = 0.5   # m/s
ANGULAR_SPEED = 1.0   # rad/s


class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')
        self.vel_pub  = self.create_publisher(Twist, '/cmd_vel', 10)
        self.settings = termios.tcgetattr(sys.stdin)

    def get_key(self) -> str:
        tty.setraw(sys.stdin.fileno())
        try:
            rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
            return sys.stdin.read(1) if rlist else ''
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

    def publish(self, linear: float, angular: float) -> None:
        twist = Twist()
        twist.linear.x  = float(linear)
        twist.angular.z = float(angular)
        self.vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()

    print(BANNER)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0)

            key = node.get_key().lower()  # lowercase so Caps Lock doesn't break anything

            # Reset each loop — hold key to keep moving, release to stop
            linear  = 0.0
            angular = 0.0

            if   key == 'w':  linear  =  LINEAR_SPEED
            elif key == 's':  linear  = -LINEAR_SPEED
            elif key == 'a':  angular =  ANGULAR_SPEED
            elif key == 'd':  angular = -ANGULAR_SPEED
            elif key == ' ':  pass          # already zero — hard stop
            elif key == '\x03': break       # CTRL-C

            node.publish(linear, angular)

    except Exception as exc:
        print(f'[teleop] Exception: {exc}')

    finally:
        node.publish(0.0, 0.0)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, node.settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()