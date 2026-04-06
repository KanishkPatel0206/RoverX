#!/usr/bin/env python3
"""
obstacle_avoidance.py  —  RoverX Obstacle Avoidance Node
Package : my_robot_description
Robot   : diff_rover  (2-wheel diff drive + front caster)
Sensors : gpu_lidar → /scan  (360°, 10 Hz, 0.15–10.0 m)
Control : /cmd_vel  (geometry_msgs/Twist, bridged via ros_gz_bridge)

Algorithm: Sector-based VFH-lite with density voting
  FOV is split into 5 sectors:
    FRONT        : ±25°   — primary danger zone
    FRONT-LEFT   : 25–80°
    FRONT-RIGHT  : -80– -25°
    REAR-LEFT    : 80–180°  (used to choose reverse turn direction)
    REAR-RIGHT   : -180– -80°

Decision tree (evaluated every /scan callback):
  1. Emergency  (front < STOP_DIST)         → REVERSE + turn
  2. Front clear                             → FORWARD
  3. Front blocked, one side clearer         → arc-turn toward clear side
  4. All front sectors blocked               → SPIN away from denser side
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


# ── Tunable constants (also exposed as ROS 2 parameters) ─────────────────────

OBSTACLE_DIST  = 0.7    # [m]  threshold: reading < this → obstacle detected
STOP_DIST      = 0.28   # [m]  emergency stop / reverse  (> lidar min_range 0.15)

LINEAR_SPEED   = 0.22   # [m/s]   forward cruise speed
ARC_SPEED      = 0.10   # [m/s]   linear component while arc-turning
TURN_SPEED     = 0.6    # [rad/s] rotation speed
REVERSE_SPEED  = 0.14   # [m/s]   reverse speed

# Sector boundaries (degrees).  Lidar frame: 0° = forward, +CCW, -CW
# Matches URDF: min_angle=-π, max_angle=+π, 360 samples
FRONT_DEG      = 25     # ±25° = 50° front cone
SIDE_DEG       = 80     # ±80° total side cone

SCAN_TOPIC     = '/scan'
CMDVEL_TOPIC   = '/cmd_vel'

# ─────────────────────────────────────────────────────────────────────────────


class ObstacleAvoidance(Node):

    def __init__(self):
        super().__init__('obstacle_avoidance')

        # Declare & read parameters
        self.declare_parameter('obstacle_dist', OBSTACLE_DIST)
        self.declare_parameter('stop_dist',     STOP_DIST)
        self.declare_parameter('linear_speed',  LINEAR_SPEED)
        self.declare_parameter('arc_speed',     ARC_SPEED)
        self.declare_parameter('turn_speed',    TURN_SPEED)
        self.declare_parameter('reverse_speed', REVERSE_SPEED)

        self.obstacle_dist = self.get_parameter('obstacle_dist').value
        self.stop_dist     = self.get_parameter('stop_dist').value
        self.linear_speed  = self.get_parameter('linear_speed').value
        self.arc_speed     = self.get_parameter('arc_speed').value
        self.turn_speed    = self.get_parameter('turn_speed').value
        self.reverse_speed = self.get_parameter('reverse_speed').value

        self.sub = self.create_subscription(
            LaserScan, SCAN_TOPIC, self._scan_cb, 10)
        self.pub = self.create_publisher(Twist, CMDVEL_TOPIC, 10)

        self._state = 'FORWARD'
        self.get_logger().info(
            f'[ObstacleAvoidance] started  |  obstacle_dist={self.obstacle_dist} m'
            f'  stop_dist={self.stop_dist} m  linear={self.linear_speed} m/s')

    # ── Sector helpers ────────────────────────────────────────────────────────

    def _sector_min(self, ranges, a_min, a_inc, lo_deg, hi_deg):
        """Minimum valid range in the angular sector [lo_deg, hi_deg]."""
        lo = math.radians(lo_deg)
        hi = math.radians(hi_deg)
        best = float('inf')
        for i, r in enumerate(ranges):
            a = a_min + i * a_inc
            a = math.atan2(math.sin(a), math.cos(a))   # wrap to [-π, π]
            if lo <= a <= hi and math.isfinite(r) and r > 0.01:
                best = min(best, r)
        return best

    def _sector_density(self, ranges, a_min, a_inc, lo_deg, hi_deg):
        """Count of readings below obstacle_dist in sector."""
        lo = math.radians(lo_deg)
        hi = math.radians(hi_deg)
        count = 0
        for i, r in enumerate(ranges):
            a = a_min + i * a_inc
            a = math.atan2(math.sin(a), math.cos(a))
            if lo <= a <= hi and math.isfinite(r) and r < self.obstacle_dist:
                count += 1
        return count

    # ── Main callback ─────────────────────────────────────────────────────────

    def _scan_cb(self, msg: LaserScan):
        ranges  = msg.ranges
        a_min   = msg.angle_min       # -π  for this lidar
        a_inc   = msg.angle_increment # ~0.01745 rad (1°)

        # ── Sector distances ──────────────────────────────────────────────────
        front      = self._sector_min(ranges, a_min, a_inc, -FRONT_DEG,  FRONT_DEG)
        front_left = self._sector_min(ranges, a_min, a_inc,  FRONT_DEG,  SIDE_DEG)
        front_right= self._sector_min(ranges, a_min, a_inc, -SIDE_DEG,  -FRONT_DEG)

        # ── Density for turn direction voting ─────────────────────────────────
        left_density  = self._sector_density(ranges, a_min, a_inc,  FRONT_DEG, SIDE_DEG)
        right_density = self._sector_density(ranges, a_min, a_inc, -SIDE_DEG, -FRONT_DEG)

        # ── Boolean flags ─────────────────────────────────────────────────────
        emergency    = front < self.stop_dist
        front_clear  = front > self.obstacle_dist
        fleft_clear  = front_left  > self.obstacle_dist
        fright_clear = front_right > self.obstacle_dist

        # ── State machine ─────────────────────────────────────────────────────
        if emergency:
            self._state = 'REVERSE'
        elif front_clear:
            self._state = 'FORWARD'
        elif fleft_clear and not fright_clear:
            self._state = 'ARC_LEFT'
        elif fright_clear and not fleft_clear:
            self._state = 'ARC_RIGHT'
        elif fleft_clear and fright_clear:
            # Both sides open — choose by less-dense side
            self._state = 'ARC_LEFT' if left_density <= right_density else 'ARC_RIGHT'
        else:
            # All blocked — spin toward less-dense side
            self._state = 'SPIN_LEFT' if left_density <= right_density else 'SPIN_RIGHT'

        # ── Velocity output ───────────────────────────────────────────────────
        cmd = Twist()

        if self._state == 'FORWARD':
            cmd.linear.x  =  self.linear_speed
            cmd.angular.z =  0.0

        elif self._state == 'ARC_LEFT':
            cmd.linear.x  =  self.arc_speed
            cmd.angular.z =  self.turn_speed

        elif self._state == 'ARC_RIGHT':
            cmd.linear.x  =  self.arc_speed
            cmd.angular.z = -self.turn_speed

        elif self._state == 'SPIN_LEFT':
            cmd.linear.x  =  0.0
            cmd.angular.z =  self.turn_speed

        elif self._state == 'SPIN_RIGHT':
            cmd.linear.x  =  0.0
            cmd.angular.z = -self.turn_speed

        elif self._state == 'REVERSE':
            cmd.linear.x  = -self.reverse_speed
            # Reverse-turn away from denser side
            cmd.angular.z = self.turn_speed if left_density <= right_density \
                            else -self.turn_speed

        self.pub.publish(cmd)

        self.get_logger().debug(
            f'[{self._state:<12}]  F:{front:.2f}m  FL:{front_left:.2f}m'
            f'  FR:{front_right:.2f}m  Ld:{left_density}  Rd:{right_density}')


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Send zero velocity before shutting down
        stop = Twist()
        node.pub.publish(stop)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()