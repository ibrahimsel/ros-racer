# MIT License

# Copyright (c) 2020 Hongrui Zheng
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

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


import os
import gym
import time
import json
import rclpy
import numpy as np
from collections import deque
from transforms3d import euler
from rclpy.node import Node
from std_msgs.msg import Bool, Int32, String
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Transform, PoseStamped
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


class GymBridge(Node):
    def __init__(self):
        super().__init__("gym_bridge")

        self.declare_parameter("racecar_namespace", "racecar")
        self.declare_parameter("racecar_odom_topic", "odom")
        self.declare_parameter("racecar_scan_topic", "scan")
        self.declare_parameter("racecar_drive_topic", "drive")
        self.declare_parameter("scan_distance_to_base_link", 0.0)
        self.declare_parameter("scan_fov", 4.7)
        self.declare_parameter("scan_beams", 1080)
        self.declare_parameter("map_path", "")
        self.declare_parameter("map_img_ext", "")
        self.declare_parameter("num_agent", 3)
        self.declare_parameter("auto_start", True)  # Auto-start simulation on launch
        default_map_path = os.path.join(get_package_share_directory("f1tenth_gym_ros"),
                                        "maps",
                                        "example_map")
        self.num_agents = self.get_parameter("num_agent").value
        self.start_subscriber = self.create_subscription(
            Bool, 'sim_start', self.start_callback, 10)
        self.pause_subscriber = self.create_subscription(
            Bool, 'sim_pause', self.pause_callback, 10)
        self.reset_subscriber = self.create_subscription(
            Bool, 'sim_reset', self.reset_callback, 10)
        self.choose_active_agent_subscriber = self.create_subscription(
            Int32, 'racecar_to_estimate_pose', self.choose_active_racecar_callback, 10)
        self.spawn_all_subscriber = self.create_subscription(
            Bool, 'spawn_all_mode', self.spawn_all_mode_callback, 10)

        # Additional attributes to manage the state of the simulation
        self.simulation_running = False
        self.simulation_paused = False
        self.spawn_all_mode = False  # When true, next pose estimate spawns all racecars
        self.agent_disqualified = [False] * self.num_agents

        # env backend
        try:
            self.map_path = self.get_parameter("map_path").value
            self.map_img_ext = self.get_parameter("map_img_ext").value
            self.env = gym.make(
                "f110_gym:f110-v0",
                map=self.map_path,
                map_ext=self.map_img_ext,
                num_agents=self.num_agents,
            )
        except Exception as e:
            self.get_logger().warn(
                f'Given map path can not be found. Defaulting to example_map.')
            self.env = gym.make(
                "f110_gym:f110-v0",
                map=default_map_path,
                map_ext='.png',
                num_agents=self.num_agents,
            )

        racecar_scan_topic = "/" + \
            self.get_parameter("racecar_scan_topic").value
        racecar_drive_topic = "/" + \
            self.get_parameter("racecar_drive_topic").value
        racecar_odom_topic = "/" + \
            self.get_parameter("racecar_odom_topic").value
        scan_fov = self.get_parameter("scan_fov").value
        scan_beams = self.get_parameter("scan_beams").value
        self.racecar_namespace = self.get_parameter("racecar_namespace").value
        self.scan_distance_to_base_link = self.get_parameter(
            "scan_distance_to_base_link"
        ).value

        self.angle_min = -scan_fov / 2.0
        self.angle_max = scan_fov / 2.0
        self.angle_inc = scan_fov / scan_beams
        self._num_beams = scan_beams

        self.pose_reset_arr = np.zeros((self.num_agents, 3))
        # Initial spawn position for racecar1 (others spawn behind it)
        initial_x = -29.224339
        initial_y = -2.775948
        initial_theta = 2.965379  # ~170° from quaternion (z=0.996, w=0.088)
        spacing = 1.5  # 1.5 meters between racecars
        # "Behind" direction is opposite of facing
        behind_theta = initial_theta + np.pi

        for i in range(self.num_agents):
            self.pose_reset_arr[i][0] = initial_x + i * spacing * np.cos(behind_theta)
            self.pose_reset_arr[i][1] = initial_y + i * spacing * np.sin(behind_theta)
            self.pose_reset_arr[i][2] = initial_theta  # All face same direction

        # Store initial start positions for race reset
        self.start_positions = self.pose_reset_arr.copy()

        self.obs, _, self.done, _ = self.env.reset(self.pose_reset_arr)

        # Adaptive sensor publishing rate based on agent count
        if self.num_agents <= 5:
            sensor_rate = 40  # Hz
        elif self.num_agents <= 10:
            sensor_rate = 25  # Hz
        else:
            sensor_rate = 20  # Hz
        sensor_period = 1.0 / sensor_rate

        # Timer rates optimized for smooth simulation:
        # - Physics: 100 Hz (10ms) - matches typical simulator timestep
        # - Sensor publish: adaptive based on agent count
        self.drive_timer = self.create_timer(0.01, self.drive_timer_callback)  # 100 Hz physics
        self.timer = self.create_timer(sensor_period, self.timer_callback)  # Adaptive sensor publish
        self.visualization_timer = self.create_timer(0.2, self._visualization_timer_callback)  # 5 Hz markers
        self.stats_timer = self.create_timer(0.5, self._stats_timer_callback)  # 2 Hz JSON stats
        self.get_logger().info(f"Sensor publish rate: {sensor_rate} Hz for {self.num_agents} agents")

        # transform broadcaster
        self.br = TransformBroadcaster(self)

        # Static transform broadcaster for laser frames (published once)
        self.static_br = StaticTransformBroadcaster(self)
        self._publish_static_laser_transforms()

        self.active_racecar_to_reset_pose = 0

        # topic names have to be unique for each car
        self.scan_topics = [
            f"{self.racecar_namespace}{i + 1}{racecar_scan_topic}" for i in range(self.num_agents)]

        self.drive_topics = [
            f"{self.racecar_namespace}{i + 1}{racecar_drive_topic}" for i in range(self.num_agents)]

        self.odom_topics = [
            f"{self.racecar_namespace}{i + 1}{racecar_odom_topic}" for i in range(self.num_agents)]

        # publishers and subscribers
        self.scan_publishers = []
        self.odom_publishers = []
        self.drive_subscribers = []
        self.drive_msgs = np.zeros(
            (self.num_agents, 2)
        )  # 2 for steering angle and speed

        # QoS profiles optimized to avoid backlog/latency under load
        # Best-effort + depth 1 for scan/odom (sensor data - latest is most important)
        sensor_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        # Depth 1 for drive (control commands - only latest matters)
        drive_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )

        for i in range(self.num_agents):
            racecar_scan_pub = self.create_publisher(
                LaserScan, self.scan_topics[i], qos_profile=sensor_qos
            )
            racecar_odom_pub = self.create_publisher(
                Odometry, self.odom_topics[i], qos_profile=sensor_qos
            )
            racecar_drive_sub = self.create_subscription(
                AckermannDriveStamped, self.drive_topics[i], self.drive_callback, qos_profile=drive_qos
            )
            self.scan_publishers.append(racecar_scan_pub)
            self.odom_publishers.append(racecar_odom_pub)
            self.drive_subscribers.append(racecar_drive_sub)

        self.marker_pub = self.create_publisher(MarkerArray, 'racecar_labels', 10)

        # Speed markers publisher (displays velocity above each car)
        self.speed_marker_pub = self.create_publisher(MarkerArray, 'speed_markers', 10)

        # Trajectory path publishers (one per agent)
        self.path_publishers = []
        self.trajectory_history = []
        for i in range(self.num_agents):
            path_pub = self.create_publisher(
                Path, f'{self.racecar_namespace}{i+1}/trajectory', 10)
            self.path_publishers.append(path_pub)
            self.trajectory_history.append(deque(maxlen=100))  # Reduced from 500 to 100

        # Trajectory publish timer (5 Hz - less frequent than main loop)
        self.trajectory_timer = self.create_timer(0.2, self._publish_trajectories)

        # Race stats publisher
        self.race_stats_pub = self.create_publisher(String, 'race_stats_json', 10)

        # Pre-allocate message objects for performance (avoid allocation in callbacks)
        self._scan_msgs = [LaserScan() for _ in range(self.num_agents)]
        self._odom_msgs = [Odometry() for _ in range(self.num_agents)]
        for i in range(self.num_agents):
            racecar_namespace = self.racecar_namespace + str(i + 1)
            # Pre-fill static scan fields
            self._scan_msgs[i].header.frame_id = racecar_namespace + "/laser"
            self._scan_msgs[i].angle_min = self.angle_min
            self._scan_msgs[i].angle_max = self.angle_max
            self._scan_msgs[i].angle_increment = self.angle_inc
            self._scan_msgs[i].range_min = 0.0
            self._scan_msgs[i].range_max = 30.0
            # Pre-fill static odom fields
            self._odom_msgs[i].header.frame_id = "map"
            self._odom_msgs[i].child_frame_id = racecar_namespace + "/base_link"

        # Per-tick cached values to avoid redundant computation across publishers
        self._cached_quats = np.zeros((self.num_agents, 4))  # [w, x, y, z] per agent
        self._cached_speeds = np.zeros(self.num_agents)

        self.racecar_drive_published = False
        self.last_log_time = time.time()
        self.pose_reset_sub = self.create_subscription(
            PoseWithCovarianceStamped, "/initialpose", self.pose_reset_callback, 10
        )

        # Track last drive message time for each racecar to detect stale commands
        self.last_drive_time = [time.time()] * self.num_agents
        self.drive_timeout = 0.1  # 100ms timeout - stop car if no command received

        # Pending pose updates for incremental world changes (instead of full reset)
        self._pending_pose_updates = {}  # {agent_idx: (x, y, theta)}

        # Auto-start simulation if enabled (removes need for Start button click)
        if self.get_parameter("auto_start").value:
            self.simulation_running = True
            self.get_logger().info("Simulation auto-started")

    def _publish_static_laser_transforms(self):
        """Publish static laser transforms once at startup."""
        transforms = []
        for i in range(self.num_agents):
            racecar_namespace = self.racecar_namespace + str(i + 1)
            racecar_scan_ts = TransformStamped()
            racecar_scan_ts.transform.translation.x = self.scan_distance_to_base_link
            racecar_scan_ts.transform.rotation.w = 1.0
            racecar_scan_ts.header.stamp = self.get_clock().now().to_msg()
            racecar_scan_ts.header.frame_id = racecar_namespace + "/base_link"
            racecar_scan_ts.child_frame_id = racecar_namespace + "/laser"
            transforms.append(racecar_scan_ts)
        self.static_br.sendTransform(transforms)

    def handle_collision(self):
        """Checks for collision and handles if there is any.

        Optimized: Uses vectorized numpy min instead of O(n * 1080) loop.
        """
        collision_threshold = 0.21
        min_scans = np.min(self.obs["scans"], axis=1)  # Vectorized - one op for all agents

        for i in range(self.num_agents):
            if self.agent_disqualified[i]:
                continue

            # Check wall collision (scan-based)
            if min_scans[i] <= collision_threshold:
                self.agent_disqualified[i] = True
                self.get_logger().info(f"Racecar {i + 1} is disqualified (wall collision)")

    def choose_active_racecar_callback(self, racecar_index):
        """Handle racecar selection from plugin numbered buttons."""
        self.active_racecar_to_reset_pose = racecar_index.data
        # Disable all special modes when selecting a racecar
        self.spawn_all_mode = False

    def spawn_all_mode_callback(self, msg):
        """Toggle spawn_all mode from plugin."""
        self.spawn_all_mode = msg.data
        if msg.data:
            self.get_logger().info("Spawn All mode activated - use 2D Pose Estimate")

    def start_callback(self, msg):
        if msg.data:
            self.simulation_running = True
            self.simulation_paused = False
            self.get_logger().info('Simulation started')

    def pause_callback(self, msg):
        if msg.data:
            self.simulation_paused = not self.simulation_paused
            if self.simulation_paused:
                self.get_logger().info('Simulation paused')
            else:
                self.get_logger().info('Simulation resumed')

    def reset_callback(self, msg):
        """Reset all agents to their starting positions."""
        if msg.data:
            self.get_logger().info('Resetting race...')
            # Reset positions to start
            self.pose_reset_arr = self.start_positions.copy()
            self.obs, _, self.done, _ = self.env.reset(np.array(self.pose_reset_arr))
            # Clear disqualifications
            self.agent_disqualified = [False] * self.num_agents
            self.get_logger().info('Race reset complete')

    def drive_callback(self, drive_msg):
        current_time = time.time()
        try:
            active_racecar = int(drive_msg.header.frame_id.split(
                self.racecar_namespace)[1].split("/")[0]) - 1  # extract the racecar to be driven from the header id

            if self.agent_disqualified[active_racecar]:
                self.drive_msgs[active_racecar][0] = 0.0
                self.drive_msgs[active_racecar][1] = 0.0
                return

            self.drive_msgs[active_racecar][0] = drive_msg.drive.steering_angle
            self.drive_msgs[active_racecar][1] = drive_msg.drive.speed
            self.last_drive_time[active_racecar] = current_time
            self.racecar_drive_published = True
        except IndexError as i:
            if abs(current_time - self.last_log_time) >= 5.0:
                self.get_logger().warn(f"Can't process frame_id: {drive_msg.header.frame_id}. Make sure you set the appropriate frame_id for your drive message! (e.g. racecar1/base_link)")
                self.last_log_time = current_time
        except ValueError as v:
            if abs(current_time - self.last_log_time) >= 5.0:
                self.get_logger().warn(f"Can't process frame_id: {drive_msg.header.frame_id}. Make sure you set the appropriate frame_id for your drive message! (e.g. racecar1/base_link)")
                self.last_log_time = current_time
        except Exception as e:
            if abs(current_time - self.last_log_time) >= 5.0:
                self.get_logger().error(f"Exception occurred at drive callback: {e}")
                self.last_log_time = current_time

    def drive_timer_callback(self):
        if self.simulation_running and not self.simulation_paused:
            current_time = time.time()

            # Check for stale drive commands and stop cars that haven't received commands
            for i in range(self.num_agents):
                if current_time - self.last_drive_time[i] > self.drive_timeout:
                    # No recent command - gradually slow down the car
                    self.drive_msgs[i][1] *= 0.9  # Reduce speed by 10%
                    if abs(self.drive_msgs[i][1]) < 0.1:
                        self.drive_msgs[i][1] = 0.0
                        self.drive_msgs[i][0] = 0.0

            # Apply any pending incremental pose updates
            if self._pending_pose_updates:
                self._apply_pending_pose_updates()

            # ALWAYS step physics - this is critical for smooth simulation!
            # Don't wait for drive messages - use last known commands
            self.obs, _, self.done, _ = self.env.step(self.drive_msgs)

            # Handle collision detection (once per step, not per agent)
            self.handle_collision()

    def _apply_pending_pose_updates(self):
        """Apply queued incremental pose updates at physics tick boundary."""
        if not self._pending_pose_updates:
            return

        # Get current poses
        for i in range(self.num_agents):
            if i in self._pending_pose_updates:
                x, y, theta = self._pending_pose_updates[i]
                self.pose_reset_arr[i][0] = x
                self.pose_reset_arr[i][1] = y
                self.pose_reset_arr[i][2] = theta
            else:
                self.pose_reset_arr[i][0] = self.obs["poses_x"][i]
                self.pose_reset_arr[i][1] = self.obs["poses_y"][i]
                self.pose_reset_arr[i][2] = self.obs["poses_theta"][i]

        # Reset env with new poses
        self.obs, _, self.done, _ = self.env.reset(np.array(self.pose_reset_arr))
        self._pending_pose_updates.clear()

    def _update_tick_cache(self):
        """Compute derived values once per tick, reused by multiple publishers.

        Caches quaternions and speeds to avoid redundant computation.
        """
        for i in range(self.num_agents):
            # Cache quaternion (transforms3d returns [w, x, y, z])
            self._cached_quats[i] = euler.euler2quat(
                0.0, 0.0, self.obs["poses_theta"][i], axes="sxyz"
            )
            # Cache speed
            self._cached_speeds[i] = np.sqrt(
                self.obs["linear_vels_x"][i]**2 +
                self.obs["linear_vels_y"][i]**2
            )

    def _visualization_timer_callback(self):
        """Publish visualization markers at 5 Hz (reduced from 20-40 Hz sensor rate)."""
        if not self.simulation_running or self.simulation_paused:
            return

        # Only publish if there are subscribers
        ts = self.get_clock().now().to_msg()
        if self.marker_pub.get_subscription_count() > 0:
            self._publish_name_markers(ts)
        if self.speed_marker_pub.get_subscription_count() > 0:
            self._publish_speed_markers(ts)

    def _stats_timer_callback(self):
        """Publish JSON race stats at 2 Hz (reduced from 20-40 Hz sensor rate)."""
        if not self.simulation_running or self.simulation_paused:
            return
        if self.race_stats_pub.get_subscription_count() > 0:
            self._publish_race_stats_json(self.get_clock().now().to_msg())

    def timer_callback(self):
        if self.simulation_running and not self.simulation_paused:
            ts = self.get_clock().now().to_msg()

            # Update cached derived values once for this tick
            self._update_tick_cache()

            # Publish scans for each agent (using pre-allocated messages)
            # Fast path: directly use numpy array when no obstacles
            for i in range(self.num_agents):
                self._scan_msgs[i].header.stamp = ts
                # Fast path: use array directly, avoid list conversion
                scan_data = self.obs["scans"][i]
                self._scan_msgs[i].ranges = scan_data.tolist()
                self.scan_publishers[i].publish(self._scan_msgs[i])

            # Publish transforms and odom (markers moved to 5 Hz visualization_timer)
            self._publish_odom(ts)
            self._publish_transforms(ts)
            self._publish_wheel_transforms(ts)

            # Update trajectory history (paths published at 5 Hz by separate timer)
            self._update_trajectory_history()

    def pose_reset_callback(self, pose_msg):
        rx = pose_msg.pose.pose.position.x
        ry = pose_msg.pose.pose.position.y
        rqx = pose_msg.pose.pose.orientation.x
        rqy = pose_msg.pose.pose.orientation.y
        rqz = pose_msg.pose.pose.orientation.z
        rqw = pose_msg.pose.pose.orientation.w
        _, _, rtheta = euler.quat2euler([rqw, rqx, rqy, rqz], axes="rxyz")

        # Handle Spawn All mode
        if self.spawn_all_mode:
            # Spawn all racecars in a vertical line (one behind the other)
            # First car at pose estimate, others behind it
            spacing = 1.5  # 1.5 meters between racecars (behind each other)
            behind_theta = rtheta + np.pi  # Direction behind the facing direction

            for i in range(self.num_agents):
                # First car at exact position, others offset behind
                x = rx + i * spacing * np.cos(behind_theta)
                y = ry + i * spacing * np.sin(behind_theta)
                self._pending_pose_updates[i] = (x, y, rtheta)
                # Clear disqualification for all racecars
                self.agent_disqualified[i] = False

            self.get_logger().info(f"Spawned all {self.num_agents} racecars in vertical line ({spacing}m apart)")
            self.spawn_all_mode = False
        else:
            # Single racecar pose reset (original behavior) - queue incremental update
            try:
                # When a pose reset is sent, racecar stops being disqualified
                if self.agent_disqualified[self.active_racecar_to_reset_pose]:
                    self.agent_disqualified[self.active_racecar_to_reset_pose] = False

                self._pending_pose_updates[self.active_racecar_to_reset_pose] = (rx, ry, rtheta)
            except IndexError:
                self.get_logger().warn("Chosen racecar does not exist")

    def _publish_odom(self, ts):
        """Publish odometry for all agents using pre-allocated messages."""
        for i in range(self.num_agents):
            odom = self._odom_msgs[i]
            odom.header.stamp = ts
            odom.pose.pose.position.x = self.obs["poses_x"][i]
            odom.pose.pose.position.y = self.obs["poses_y"][i]
            # Use cached quaternion (computed once per tick in _update_tick_cache)
            quat = self._cached_quats[i]
            odom.pose.pose.orientation.x = quat[1]
            odom.pose.pose.orientation.y = quat[2]
            odom.pose.pose.orientation.z = quat[3]
            odom.pose.pose.orientation.w = quat[0]
            odom.twist.twist.linear.x = self.obs["linear_vels_x"][i]
            odom.twist.twist.linear.y = self.obs["linear_vels_y"][i]
            odom.twist.twist.angular.z = self.obs["ang_vels_z"][i]
            self.odom_publishers[i].publish(odom)

    # Color palette for up to 10 agents (cycles if more)
    AGENT_COLORS = [
        (1.0, 0.0, 0.0),    # red
        (0.0, 1.0, 0.0),    # green
        (0.0, 0.0, 1.0),    # blue
        (1.0, 1.0, 0.0),    # yellow
        (1.0, 0.0, 1.0),    # magenta
        (0.0, 1.0, 1.0),    # cyan
        (1.0, 0.5, 0.0),    # orange
        (0.5, 0.0, 1.0),    # purple
        (1.0, 0.75, 0.8),   # pink
        (0.6, 0.3, 0.0),    # brown
    ]

    def _publish_name_markers(self, ts):
        arr = MarkerArray()
        for i in range(self.num_agents):
            ns  = f"{self.racecar_namespace}{i+1}"
            m           = Marker()
            m.header.stamp   = ts
            m.header.frame_id = f"{ns}/base_link"
            m.ns       = "labels"
            m.id       = i
            m.type     = Marker.TEXT_VIEW_FACING
            m.action   = Marker.ADD
            m.pose.position.z = 0.5      # half-meter above roof
            m.scale.z  = 0.3             # text height (m)
            r, g, b    = self.AGENT_COLORS[i % len(self.AGENT_COLORS)]
            m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, 1.0
            m.text = ns.replace("racecar", "agent")
            arr.markers.append(m)
        self.marker_pub.publish(arr)

    def _publish_speed_markers(self, ts):
        """Publish speed text markers above each racecar."""
        arr = MarkerArray()
        for i in range(self.num_agents):
            ns = f"{self.racecar_namespace}{i+1}"
            # Use cached speed (computed once per tick in _update_tick_cache)
            speed = self._cached_speeds[i]

            m = Marker()
            m.header.stamp = ts
            m.header.frame_id = f"{ns}/base_link"
            m.ns = "speeds"
            m.id = i
            m.type = Marker.TEXT_VIEW_FACING
            m.action = Marker.ADD
            m.pose.position.z = 0.8  # Above the name marker
            m.scale.z = 0.25  # Slightly smaller than name

            # Color gradient: green (slow) -> yellow (medium) -> red (fast)
            # Assuming max speed around 8 m/s
            speed_ratio = min(speed / 8.0, 1.0)
            if speed_ratio < 0.5:
                # Green to yellow
                m.color.r = speed_ratio * 2.0
                m.color.g = 1.0
            else:
                # Yellow to red
                m.color.r = 1.0
                m.color.g = 1.0 - (speed_ratio - 0.5) * 2.0
            m.color.b = 0.0
            m.color.a = 1.0

            m.text = f"{speed:.1f} m/s"
            arr.markers.append(m)
        self.speed_marker_pub.publish(arr)

    def _update_trajectory_history(self):
        """Record current positions for trajectory visualization."""
        ts = self.get_clock().now().to_msg()
        for i in range(self.num_agents):
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = ts  # Reuse single timestamp for all agents
            pose.pose.position.x = self.obs["poses_x"][i]
            pose.pose.position.y = self.obs["poses_y"][i]
            pose.pose.position.z = 0.0
            # Use cached quaternion (computed once per tick in _update_tick_cache)
            quat = self._cached_quats[i]
            pose.pose.orientation.x = quat[1]
            pose.pose.orientation.y = quat[2]
            pose.pose.orientation.z = quat[3]
            pose.pose.orientation.w = quat[0]
            self.trajectory_history[i].append(pose)

    def _publish_trajectories(self):
        """Publish trajectory paths for all agents (called at 5 Hz)."""
        if not self.simulation_running or self.simulation_paused:
            return

        ts = self.get_clock().now().to_msg()
        for i in range(self.num_agents):
            # Only publish if there are subscribers
            if self.path_publishers[i].get_subscription_count() == 0:
                continue
            if len(self.trajectory_history[i]) < 2:
                continue

            path = Path()
            path.header.stamp = ts
            path.header.frame_id = "map"
            path.poses = list(self.trajectory_history[i])
            self.path_publishers[i].publish(path)

    def _publish_race_stats_json(self, ts):
        """Publish comprehensive race stats as JSON for UI consumption."""
        stats = {
            "timestamp": ts.sec + ts.nanosec * 1e-9,
            "num_agents": self.num_agents,
            "simulation_running": self.simulation_running,
            "simulation_paused": self.simulation_paused,
            "agents": []
        }

        for i in range(self.num_agents):
            # Use cached values (computed once per tick in _update_tick_cache)
            speed = self._cached_speeds[i]

            agent_stats = {
                "id": i + 1,
                "name": f"{self.racecar_namespace}{i+1}",
                "position": {
                    "x": float(self.obs["poses_x"][i]),
                    "y": float(self.obs["poses_y"][i]),
                    "theta": float(self.obs["poses_theta"][i])
                },
                "speed": float(speed),
                "disqualified": self.agent_disqualified[i]
            }
            stats["agents"].append(agent_stats)

        msg = String()
        msg.data = json.dumps(stats)
        self.race_stats_pub.publish(msg)

    def _publish_transforms(self, ts):
        """Publish base_link transforms for all agents in a single batch."""
        transforms = []
        for i in range(self.num_agents):
            racecar_namespace = self.racecar_namespace + str(i + 1)
            racecar_t = Transform()
            racecar_t.translation.x = self.obs["poses_x"][i]
            racecar_t.translation.y = self.obs["poses_y"][i]
            racecar_t.translation.z = 0.0
            # Use cached quaternion (computed once per tick in _update_tick_cache)
            quat = self._cached_quats[i]
            racecar_t.rotation.x = quat[1]
            racecar_t.rotation.y = quat[2]
            racecar_t.rotation.z = quat[3]
            racecar_t.rotation.w = quat[0]
            racecar_ts = TransformStamped()
            racecar_ts.transform = racecar_t
            racecar_ts.header.stamp = ts
            racecar_ts.header.frame_id = "map"
            racecar_ts.child_frame_id = racecar_namespace + "/base_link"
            transforms.append(racecar_ts)
        self.br.sendTransform(transforms)  # Single batch call

    def _publish_wheel_transforms(self, ts):
        """Publish wheel transforms for all agents in a single batch."""
        transforms = []
        for i in range(self.num_agents):
            racecar_namespace = self.racecar_namespace + str(i + 1)
            # Use the current steering angle for wheel orientation
            steering_angle = self.drive_msgs[i][0]
            racecar_wheel_quat = euler.euler2quat(
                0.0, 0.0, steering_angle, axes="sxyz"
            )
            # Left wheel
            left_wheel_ts = TransformStamped()
            left_wheel_ts.transform.rotation.x = racecar_wheel_quat[1]
            left_wheel_ts.transform.rotation.y = racecar_wheel_quat[2]
            left_wheel_ts.transform.rotation.z = racecar_wheel_quat[3]
            left_wheel_ts.transform.rotation.w = racecar_wheel_quat[0]
            left_wheel_ts.header.stamp = ts
            left_wheel_ts.header.frame_id = racecar_namespace + "/front_left_hinge"
            left_wheel_ts.child_frame_id = racecar_namespace + "/front_left_wheel"
            transforms.append(left_wheel_ts)
            # Right wheel
            right_wheel_ts = TransformStamped()
            right_wheel_ts.transform.rotation.x = racecar_wheel_quat[1]
            right_wheel_ts.transform.rotation.y = racecar_wheel_quat[2]
            right_wheel_ts.transform.rotation.z = racecar_wheel_quat[3]
            right_wheel_ts.transform.rotation.w = racecar_wheel_quat[0]
            right_wheel_ts.header.stamp = ts
            right_wheel_ts.header.frame_id = racecar_namespace + "/front_right_hinge"
            right_wheel_ts.child_frame_id = racecar_namespace + "/front_right_wheel"
            transforms.append(right_wheel_ts)
        self.br.sendTransform(transforms)  # Single batch call


def main(args=None):
    rclpy.init(args=args)
    gym_bridge = GymBridge()
    try:
        rclpy.spin(gym_bridge)
    except KeyboardInterrupt:
        print("Interrupt signal detected... Killing node")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
