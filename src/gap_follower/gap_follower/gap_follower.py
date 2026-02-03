#
# Copyright (c) 2025 Composiv.ai
#
# This program and the accompanying materials are made available under the
# terms of the Eclipse Public License 2.0 which is available at
# http://www.eclipse.org/legal/epl-2.0.
#
# SPDX-License-Identifier: EPL-2.0
#
# Contributors:
#   Composiv.ai - initial API and implementation
#
# Follow The Gap algorithm with proper bicycle-model steering geometry
# Uses Pure Pursuit style steering conversion + hysteresis + rate limiting
# Optimized for F1TENTH racing on Spielberg (Red Bull Ring) track
#

import rclpy
import numpy as np
import math
import os
from typing import Optional, Tuple
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

# =============================================================================
# ENVIRONMENT PARAMETERS - Tune these for different driving styles
# =============================================================================

# Speed limits
MAX_SPEED = float(os.environ.get('GAP_FOLLOWER_MAX_SPEED', '3.0'))
MIN_SPEED = float(os.environ.get('GAP_FOLLOWER_MIN_SPEED', '1.0'))

# Gap detection
SAFE_GAP = float(os.environ.get('GAP_FOLLOWER_SAFE_GAP', '0.8'))
BUBBLE_RADIUS = float(os.environ.get('GAP_FOLLOWER_BUBBLE_RADIUS', '0.25'))
CAR_WIDTH = float(os.environ.get('GAP_FOLLOWER_CAR_WIDTH', '0.25'))

# Emergency thresholds
FRONT_DISTANCE_THRESHOLD = float(os.environ.get('GAP_FOLLOWER_FRONT_THRESHOLD', '0.4'))
SLOWDOWN_DISTANCE = float(os.environ.get('GAP_FOLLOWER_SLOWDOWN_DIST', '2.0'))

# Bicycle model parameters
WHEELBASE = float(os.environ.get('GAP_FOLLOWER_WHEELBASE', '0.33'))
LOOKAHEAD_MIN = float(os.environ.get('GAP_FOLLOWER_LOOKAHEAD_MIN', '0.8'))
LOOKAHEAD_MAX = float(os.environ.get('GAP_FOLLOWER_LOOKAHEAD_MAX', '3.0'))

# Steering control
ANGLE_GAIN = float(os.environ.get('GAP_FOLLOWER_ANGLE_GAIN', '1.0'))
STEER_SMOOTH = float(os.environ.get('GAP_FOLLOWER_STEER_SMOOTH', '0.3'))
STEER_RATE_LIMIT = float(os.environ.get('GAP_FOLLOWER_STEER_RATE', '4.0'))  # rad/s

# Target hysteresis - prevents target from jumping around
TARGET_JUMP_MAX_IDX = int(os.environ.get('GAP_FOLLOWER_TARGET_JUMP_MAX', '20'))
TARGET_HYSTERESIS_BONUS = float(os.environ.get('GAP_FOLLOWER_HYSTERESIS_BONUS', '0.15'))

# Physical limits
MAX_STEERING_ANGLE = 0.4189  # ~24 degrees, typical for F1TENTH

FOV = math.radians(270)  # lidar FOV
HALF_FOV = FOV / 2


# =============================================================================
# UTILITY FUNCTIONS
# =============================================================================

def clamp(v: float, lo: float, hi: float) -> float:
    """Clamp a value to [lo, hi]."""
    return max(lo, min(hi, v))


def preprocess_lidar(ranges) -> np.ndarray:
    """Clean up lidar data - replace invalid readings."""
    arr = np.array(ranges, dtype=np.float64)
    arr = np.where(np.isfinite(arr), arr, 30.0)
    arr = np.clip(arr, 0.0, 30.0)
    return arr


def apply_safety_bubble(ranges: np.ndarray, closest_idx: int,
                        bubble_radius: float, angle_inc: float) -> np.ndarray:
    """Zero out points around the closest obstacle."""
    if closest_idx < 0 or closest_idx >= len(ranges):
        return ranges

    result = ranges.copy()
    n = len(ranges)

    # Calculate bubble size based on distance
    dist = ranges[closest_idx]
    if dist > 0.01:
        bubble_indices = int(bubble_radius / (dist * angle_inc + 0.001)) + 1
        bubble_indices = min(bubble_indices, 50)

        start = max(0, closest_idx - bubble_indices)
        end = min(n, closest_idx + bubble_indices + 1)
        result[start:end] = 0.0

    return result


def apply_disparity_extender(ranges: np.ndarray, angle_inc: float,
                             car_width: float) -> np.ndarray:
    """Extend obstacles at disparity points to prevent corner clipping."""
    extended = ranges.copy()
    n = len(ranges)
    disparity_threshold = 0.3

    for i in range(1, n):
        diff = ranges[i] - ranges[i - 1]
        if abs(diff) > disparity_threshold:
            closer_dist = min(ranges[i], ranges[i - 1])
            if closer_dist > 0.1:
                extend_angle = math.atan2(car_width, closer_dist)
                extend_indices = int(extend_angle / angle_inc) + 1

                if diff > 0:
                    for j in range(i, min(n, i + extend_indices)):
                        extended[j] = min(extended[j], closer_dist)
                else:
                    for j in range(max(0, i - extend_indices), i):
                        extended[j] = min(extended[j], closer_dist)

    return extended


# =============================================================================
# GAP FINDING - Returns target INDEX (not angle) for hysteresis
# =============================================================================

def find_best_gap_index(ranges: np.ndarray, safe_gap: float) -> Tuple[int, float]:
    """
    Find the best gap and return target index + depth.
    Returns index instead of angle to enable hysteresis.
    """
    n = len(ranges)
    safe_mask = ranges > safe_gap

    if not np.any(safe_mask):
        # No safe gaps - head towards farthest point
        idx = int(np.argmax(ranges))
        return idx, float(ranges[idx])

    # Find contiguous safe regions
    padded = np.concatenate([[False], safe_mask, [False]])
    diff = np.diff(padded.astype(np.int32))
    starts = np.where(diff == 1)[0]
    ends = np.where(diff == -1)[0]

    if len(starts) == 0:
        idx = int(np.argmax(ranges))
        return idx, float(ranges[idx])

    # Score each gap: width * (1 + depth * 0.5)
    best_score = -1.0
    best_start = 0
    best_end = 0

    for s, e in zip(starts, ends):
        width = int(e - s)
        if width <= 0:
            continue
        seg = ranges[int(s):int(e)]
        max_depth = float(np.max(seg)) if seg.size > 0 else 0.0
        score = float(width) * (1.0 + max_depth * 0.5)

        if score > best_score:
            best_score = score
            best_start = int(s)
            best_end = int(e)

    if best_end <= best_start:
        idx = int(np.argmax(ranges))
        return idx, float(ranges[idx])

    # Blend between deepest point and center for stability
    seg = ranges[best_start:best_end]
    deepest_local = int(np.argmax(seg))
    deepest_idx = best_start + deepest_local
    center_idx = (best_start + best_end) // 2

    blend = 0.6
    idx = int(blend * deepest_idx + (1.0 - blend) * center_idx)
    idx = max(best_start, min(best_end - 1, idx))

    return idx, float(ranges[idx])


def get_front_distance(ranges: np.ndarray, angle_inc: float) -> float:
    """Get minimum distance in a narrow front cone (+/- 15 degrees)."""
    n = len(ranges)
    center = n // 2
    cone_angle = math.radians(15)
    cone_indices = int(cone_angle / angle_inc)

    start = max(0, center - cone_indices)
    end = min(n, center + cone_indices + 1)

    front = ranges[start:end]
    valid = front[front > 0.1]

    return float(np.min(valid)) if len(valid) > 0 else 30.0


def get_side_distances(ranges: np.ndarray, angle_inc: float) -> Tuple[float, float]:
    """Get average distances to left and right sides."""
    n = len(ranges)

    left_idx = int((HALF_FOV + math.radians(90)) / angle_inc)
    left_idx = min(n - 1, max(0, left_idx))

    right_idx = int((HALF_FOV - math.radians(90)) / angle_inc)
    right_idx = min(n - 1, max(0, right_idx))

    window = 20
    left_dist = np.mean(ranges[max(0, left_idx - window):min(n, left_idx + window)])
    right_dist = np.mean(ranges[max(0, right_idx - window):min(n, right_idx + window)])

    return float(left_dist), float(right_dist)


# =============================================================================
# STEERING GEOMETRY - Bicycle model Pure Pursuit
# =============================================================================

def steering_from_target(alpha_rad: float, lookahead_m: float,
                         wheelbase_m: float) -> float:
    """
    Compute Ackermann steering angle from a target direction using bicycle model.

    This is the Pure Pursuit formula:
        delta = atan2(2 * L * sin(alpha), Ld)

    Where:
        L = wheelbase
        alpha = target angle relative to vehicle forward
        Ld = lookahead distance

    This converts "point at that direction" into proper Ackermann steering.
    """
    ld = max(0.05, lookahead_m)
    return math.atan2(2.0 * wheelbase_m * math.sin(alpha_rad), ld)


def rate_limit(prev: float, target: float, max_rate: float, dt: float) -> float:
    """Rate-limit a signal to prevent sudden jumps."""
    if dt <= 0.0:
        return target
    max_step = max_rate * dt
    return prev + clamp(target - prev, -max_step, max_step)


# =============================================================================
# MAIN GAP FOLLOWER NODE
# =============================================================================

class GapFollower(Node):
    def __init__(self):
        self.hostname = os.uname()[1]
        super().__init__(f'{self.hostname}_gap_follower')

        self.get_logger().info(
            f'Gap Follower [BICYCLE MODEL] starting with: '
            f'MAX_SPEED={MAX_SPEED}, WHEELBASE={WHEELBASE}, '
            f'LOOKAHEAD=[{LOOKAHEAD_MIN},{LOOKAHEAD_MAX}], RATE_LIMIT={STEER_RATE_LIMIT}'
        )

        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, f'/{self.hostname}/drive', 1
        )
        self.create_subscription(LaserScan, f'/{self.hostname}/scan', self.cb, 1)

        # State for smoothing and rate limiting
        self.last_steer = 0.0
        self.last_speed = MIN_SPEED
        self.last_target_idx: Optional[int] = None
        self.last_stamp_sec: Optional[float] = None

    def cb(self, scan):
        # Calculate dt for rate limiting
        stamp_sec = float(scan.header.stamp.sec) + float(scan.header.stamp.nanosec) * 1e-9
        dt = 0.0
        if self.last_stamp_sec is not None:
            dt = max(0.0, stamp_sec - self.last_stamp_sec)
        self.last_stamp_sec = stamp_sec

        # Preprocess lidar
        ranges = preprocess_lidar(scan.ranges)
        angle_inc = scan.angle_increment
        angle_min = scan.angle_min

        # Find and bubble the closest point
        valid_mask = ranges > 0.1
        if np.any(valid_mask):
            closest_idx = int(np.argmin(np.where(valid_mask, ranges, 100.0)))
        else:
            closest_idx = len(ranges) // 2

        ranges = apply_safety_bubble(ranges, closest_idx, BUBBLE_RADIUS, angle_inc)
        ranges = apply_disparity_extender(ranges, angle_inc, CAR_WIDTH)

        # Get front distance for emergency handling
        front_dist = get_front_distance(ranges, angle_inc)

        # Emergency steering if too close
        if front_dist < FRONT_DISTANCE_THRESHOLD:
            left_dist, right_dist = get_side_distances(preprocess_lidar(scan.ranges), angle_inc)
            emergency_steer = 0.3 if left_dist > right_dist else -0.3
            self.publish_drive(scan.header.stamp, emergency_steer, MIN_SPEED * 0.5)
            return

        # Find best target INDEX (not angle yet)
        target_idx, gap_depth = find_best_gap_index(ranges, SAFE_GAP)

        # Apply target hysteresis - don't let target teleport
        if self.last_target_idx is not None:
            prev_idx = self.last_target_idx
            prev_depth = float(ranges[prev_idx]) if 0 <= prev_idx < len(ranges) else 0.0

            jump = abs(target_idx - prev_idx)
            # Only allow large jumps if new target is significantly better
            if jump > TARGET_JUMP_MAX_IDX:
                if gap_depth < prev_depth * (1.0 + TARGET_HYSTERESIS_BONUS):
                    target_idx = prev_idx
                    gap_depth = prev_depth

        self.last_target_idx = target_idx

        # Convert target index to target angle (relative to vehicle forward)
        # Note: angle_min is typically negative (left side of FOV)
        alpha = angle_min + target_idx * angle_inc

        # Determine lookahead distance based on gap depth
        lookahead = clamp(gap_depth, LOOKAHEAD_MIN, LOOKAHEAD_MAX)

        # BICYCLE MODEL: Convert target angle to Ackermann steering
        steer_cmd = steering_from_target(alpha, lookahead, WHEELBASE)

        # Apply gain and clamp to physical limits
        steer_cmd *= ANGLE_GAIN
        steer_cmd = clamp(steer_cmd, -MAX_STEERING_ANGLE, MAX_STEERING_ANGLE)

        # Smooth the steering command
        steer_smoothed = (1.0 - STEER_SMOOTH) * steer_cmd + STEER_SMOOTH * self.last_steer

        # Rate limit to prevent jerky steering
        steer_final = rate_limit(self.last_steer, steer_smoothed, STEER_RATE_LIMIT, dt)
        self.last_steer = steer_final

        # Calculate speed based on steering magnitude and front distance
        steer_magnitude = abs(steer_final)
        steer_factor = 1.0 - min(0.6, steer_magnitude / MAX_STEERING_ANGLE)

        if front_dist < SLOWDOWN_DISTANCE:
            dist_ratio = (front_dist - FRONT_DISTANCE_THRESHOLD) / \
                         (SLOWDOWN_DISTANCE - FRONT_DISTANCE_THRESHOLD)
            dist_factor = clamp(dist_ratio, 0.2, 1.0)
        else:
            dist_factor = 1.0

        # Depth confidence factor
        depth_factor = min(1.0, gap_depth / 2.0)

        speed_factor = min(steer_factor, dist_factor) * (0.7 + 0.3 * depth_factor)
        target_speed = MIN_SPEED + (MAX_SPEED - MIN_SPEED) * speed_factor

        # Smooth speed changes
        speed = 0.8 * target_speed + 0.2 * self.last_speed
        self.last_speed = speed

        self.publish_drive(scan.header.stamp, steer_final, speed)

    def publish_drive(self, stamp, steer: float, speed: float):
        msg = AckermannDriveStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = f"{self.hostname}/base_link"
        msg.drive.steering_angle = float(steer)
        msg.drive.speed = float(speed)
        self.drive_pub.publish(msg)


def main():
    rclpy.init()
    g = GapFollower()
    rclpy.spin(g)
    g.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
