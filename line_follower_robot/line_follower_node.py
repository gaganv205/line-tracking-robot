#!/usr/bin/env python3
"""
Line Following Robot with Broken Line Detection and Obstacle Avoidance.
State machine:
  FOLLOWING  -> SEARCHING (line lost) | AVOIDING (obstacle close)
  SEARCHING  -> FOLLOWING (line found again)
  AVOIDING   -> FOLLOWING (obstacle cleared, line found)

FIXES APPLIED:
  - FIX 1: _just_avoided flag properly cleared on reacquire and line loss
  - FIX 2: Aligning threshold relaxed (40px) and grace period extended (30 frames)
  - FIX 3: Post-avoidance search retains short forward phase
  - FIX 4: Added move_to_line and turn_left_final phases for correct geometry
  - FIX 5: Enhanced debug logging for every phase transition
"""

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from enum import Enum, auto

from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge


class State(Enum):
    FOLLOWING = auto()
    SEARCHING = auto()
    AVOIDING  = auto()


class LineFollowerNode(Node):

    # --- tuneable constants ---
    YELLOW_LOWER_HSV     = np.array([20, 100, 100], dtype=np.uint8)
    YELLOW_UPPER_HSV     = np.array([35, 255, 255], dtype=np.uint8)
    LINE_MIN_AREA        = 500
    ROI_TOP_FRAC         = 0.5
    SEARCH_SPEED         = 0.5
    FOLLOW_SPEED         = 0.2
    OBSTACLE_DIST        = 0.8
    AVOID_CLEAR_DIST     = 1.0
    SEARCH_FORWARD_STEPS = 60
    MAX_SEARCH_ITERS     = 60
    KP                   = 0.010

    def __init__(self):
        super().__init__('line_follower_node')
        self.bridge = CvBridge()
        self.state  = State.FOLLOWING

        self._search_dir   = 1
        self._search_count = 0
        self._last_error   = 0

        self._obs_phase       = 'stop_and_back'
        self._obs_step_count  = 0
        self._following_grace = 0

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Image,     '/camera/image_raw', self.image_callback, 10)
        self.create_subscription(LaserScan, 'scan',              self.scan_callback,  10)

        self._min_front_dist = float('inf')
        self._image_received = False
        self._error          = 0
        self._line_found     = False
        self._just_avoided   = False

        self.timer = self.create_timer(0.1, self.control_loop)
        self._last_image_time = self.get_clock().now()

        self.get_logger().info('Line follower node started (Autonomous Mode).')

    # ------------------------------------------------------------------
    # Main Control Loop
    # ------------------------------------------------------------------

    def control_loop(self):
        if self._following_grace > 0:
            self._following_grace -= 1

        if self.state in [State.FOLLOWING, State.SEARCHING]:

            if self._min_front_dist < self.OBSTACLE_DIST:
                self.get_logger().info(
                    f'Obstacle detected at {self._min_front_dist:.2f}m — switching to AVOIDING')
                self.state           = State.AVOIDING
                self._obs_phase      = 'stop_and_back'
                self._obs_step_count = 0
                self._just_avoided   = True

            elif self.state == State.FOLLOWING and not self._line_found and self._following_grace == 0:
                self.get_logger().info('Line lost — switching to SEARCHING')
                self.state         = State.SEARCHING
                self._search_count = 0
                self._just_avoided = False   # FIX 1

            elif self.state == State.SEARCHING and self._line_found:
                self.get_logger().info('Line reacquired — switching to FOLLOWING')
                self.state         = State.FOLLOWING
                self._just_avoided = False   # FIX 1

        elif self.state == State.AVOIDING:
            pass

        if self.state == State.FOLLOWING:
            self._follow_step(self._error)
        elif self.state == State.SEARCHING:
            self._search_step()
        elif self.state == State.AVOIDING:
            self._avoid_step(self._line_found)

    # ------------------------------------------------------------------
    # Sensor Callbacks
    # ------------------------------------------------------------------

    def scan_callback(self, msg: LaserScan):
        ranges = np.array(msg.ranges)
        ranges = np.where(np.isfinite(ranges), ranges, float('inf'))
        n      = len(ranges)
        front  = ranges[n // 4 : 3 * n // 4]
        self._min_front_dist = float(np.min(front)) if front.size > 0 else float('inf')

    def image_callback(self, msg: Image):
        if not self._image_received:
            self.get_logger().info('First camera frame received.')
            self._image_received = True
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self._error, self._line_found = self._detect_line(frame)
        self._last_image_time = self.get_clock().now()

    # ------------------------------------------------------------------
    # Vision
    # ------------------------------------------------------------------

    def _detect_line(self, frame):
        h, w  = frame.shape[:2]
        roi   = frame[int(h * self.ROI_TOP_FRAC):, :]  # bottom portion
        hsv   = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        bw    = cv2.inRange(hsv, self.YELLOW_LOWER_HSV, self.YELLOW_UPPER_HSV)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        bw     = cv2.morphologyEx(bw, cv2.MORPH_CLOSE, kernel)
        bw     = cv2.morphologyEx(bw, cv2.MORPH_OPEN, kernel)
        M = cv2.moments(bw)
        if M['m00'] < self.LINE_MIN_AREA:
            return self._last_error, False
        cx    = int(M['m10'] / M['m00'])
        error = cx - w // 2
        self._last_error = error
        return error, True

    # ------------------------------------------------------------------
    # State Behaviours
    # ------------------------------------------------------------------

    def _follow_step(self, error: float):
        twist           = Twist()
        twist.linear.x  = self.FOLLOW_SPEED
        twist.angular.z = max(-1.0, min(1.0, -float(error) * self.KP))
        self.cmd_pub.publish(twist)

    def _search_step(self):
        self._search_count += 1
        twist = Twist()

        # FIX 3: short forward phase retained after avoidance
        if self._just_avoided:
            self._search_count = max(self._search_count, self.SEARCH_FORWARD_STEPS // 2)
            self._just_avoided = False
            self.get_logger().info('Search: post-avoidance short forward phase')

        if self._search_count < self.SEARCH_FORWARD_STEPS:
            twist.linear.x  = self.FOLLOW_SPEED
            twist.angular.z = 0.0
        else:
            iters_since_forward = self._search_count - self.SEARCH_FORWARD_STEPS
            if iters_since_forward > self.MAX_SEARCH_ITERS:
                self._search_dir   = -self._search_dir
                self._search_count = self.SEARCH_FORWARD_STEPS + 1
                self.get_logger().info(f'Search: flip direction → {self._search_dir}')
            twist.linear.x  = 0.0
            twist.angular.z = self._search_dir * self.SEARCH_SPEED

        self.cmd_pub.publish(twist)

    def _avoid_step(self, line_found: bool):
        """
        Complete 9-phase square box bypass matched to world file geometry.

        World file obstacle: x=2.5, y=0, size=0.3x0.3m
        Line at y=0, continues after x=3.0

        Path taken:
        ┌─────────────────────────────────┐
        │  Robot triggers at x≈1.7        │
        │  Backs up → turns left          │
        │  Moves sideways (y direction)   │
        │  Turns right → drives forward   │
        │  Turns right → moves to line    │
        │  Turns left → reacquires line   │
        └─────────────────────────────────┘

        At 10 Hz:
          0.8 rad/s × 20 steps × 0.1s = 1.6 rad ≈ 90°
          0.2 m/s  × N  steps × 0.1s = distance
        """
        twist = Twist()
        self._obs_step_count += 1

        TURN_90_STEPS  = 20   # ~90° at 0.8 rad/s
        BACKUP_STEPS   = 15   # ~0.22m backup
        SIDEWAYS_STEPS = 40   # ~0.8m sideways (clear obstacle)
        BYPASS_STEPS   = 80   # ~1.6m forward  (past obstacle length)
        RETURN_STEPS   = 40   # ~0.8m return toward line

        # Phase 0 — back up for turn clearance
        if self._obs_phase == 'stop_and_back':
            twist.linear.x = -0.15
            if self._obs_step_count == 1:
                self.get_logger().info('--- Avoidance Phase 0: stop_and_back ---')
            if self._obs_step_count >= BACKUP_STEPS:
                self._obs_phase      = 'turn_left'
                self._obs_step_count = 0

        # Phase 1 — turn left 90°
        elif self._obs_phase == 'turn_left':
            twist.angular.z = 0.8
            if self._obs_step_count == 1:
                self.get_logger().info('--- Avoidance Phase 1: turn_left_90 ---')
            if self._obs_step_count >= TURN_90_STEPS:
                self._obs_phase      = 'move_sideways'
                self._obs_step_count = 0

        # Phase 2 — move sideways to clear obstacle width
        elif self._obs_phase == 'move_sideways':
            twist.linear.x = 0.2
            if self._obs_step_count == 1:
                self.get_logger().info('--- Avoidance Phase 2: move_sideways ---')
            if self._obs_step_count >= SIDEWAYS_STEPS:
                self._obs_phase      = 'turn_right_1'
                self._obs_step_count = 0

        # Phase 3 — turn right 90° (now parallel to line)
        elif self._obs_phase == 'turn_right_1':
            twist.angular.z = -0.8
            if self._obs_step_count == 1:
                self.get_logger().info('--- Avoidance Phase 3: turn_right_parallel ---')
            if self._obs_step_count >= TURN_90_STEPS:
                self._obs_phase      = 'bypass_box'
                self._obs_step_count = 0

        # Phase 4 — drive forward past obstacle
        elif self._obs_phase == 'bypass_box':
            twist.linear.x = 0.2
            if self._obs_step_count == 1:
                self.get_logger().info('--- Avoidance Phase 4: bypass_box ---')
            if self._obs_step_count >= BYPASS_STEPS:
                self._obs_phase      = 'turn_right_2'
                self._obs_step_count = 0

        # Phase 5 — turn right 90° (face back toward line)
        elif self._obs_phase == 'turn_right_2':
            twist.angular.z = -0.8
            if self._obs_step_count == 1:
                self.get_logger().info('--- Avoidance Phase 5: turn_right_towards_line ---')
            if self._obs_step_count >= TURN_90_STEPS:
                self._obs_phase      = 'move_to_line'
                self._obs_step_count = 0

        # Phase 6 — FIX 4: move back toward the line
        elif self._obs_phase == 'move_to_line':
            twist.linear.x = 0.2
            if self._obs_step_count == 1:
                self.get_logger().info('--- Avoidance Phase 6: move_to_line ---')
            if self._obs_step_count >= RETURN_STEPS:
                self._obs_phase      = 'turn_left_final'
                self._obs_step_count = 0

        # Phase 7 — FIX 4: turn left 90° to face forward along line direction
        elif self._obs_phase == 'turn_left_final':
            twist.angular.z = 0.8
            if self._obs_step_count == 1:
                self.get_logger().info('--- Avoidance Phase 7: turn_left_face_forward ---')
            if self._obs_step_count >= TURN_90_STEPS:
                self._obs_phase      = 'reacquire'
                self._obs_step_count = 0

        # Phase 8 — creep forward until camera sees the line
        elif self._obs_phase == 'reacquire':
            twist.linear.x  = 0.1
            twist.angular.z = 0.0
            if self._obs_step_count == 1:
                self.get_logger().info('--- Avoidance Phase 8: reacquire_line ---')

            # Progress log every 2 seconds
            if self._obs_step_count % 20 == 0:
                self.get_logger().info(
                    f'Reacquire step={self._obs_step_count} | '
                    f'line_found={line_found} | error={self._error}px')

            if self._obs_step_count > 150:   # 15 second timeout
                self.get_logger().warn('Reacquire timed out — falling back to SEARCHING')
                self.state         = State.SEARCHING
                self._search_count = 0
                self._just_avoided = True
            elif line_found:
                self.get_logger().info(
                    f'Line found at step {self._obs_step_count} '
                    f'error={self._error}px — moving to aligning')
                self._obs_phase      = 'aligning'
                self._obs_step_count = 0

        # Phase 9 — proportional steering to centre on line
        elif self._obs_phase == 'aligning':
            twist.linear.x  = 0.08
            steer           = -float(self._error) * self.KP * 4.0
            twist.angular.z = max(-0.6, min(0.6, steer))

            if self._obs_step_count == 1:
                self.get_logger().info('--- Avoidance Phase 9: aligning ---')

            # Progress log every 1 second
            if self._obs_step_count % 10 == 0:
                self.get_logger().info(
                    f'Aligning: error={self._error}px | '
                    f'steer={twist.angular.z:.3f} | '
                    f'line_found={line_found}')

            # FIX 2: relaxed threshold (40px), longer timeout (4× TURN_90)
            centred   = line_found and abs(self._error) < 40
            timed_out = self._obs_step_count >= TURN_90_STEPS * 4

            if centred or timed_out:
                reason = 'centred' if centred else 'timed_out'
                self.get_logger().info(
                    f'Aligned ({reason}) error={self._error}px — resuming FOLLOWING')
                self.state            = State.FOLLOWING
                self._just_avoided    = False
                self._following_grace = 30   # FIX 2: 3 second grace period

        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = LineFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
