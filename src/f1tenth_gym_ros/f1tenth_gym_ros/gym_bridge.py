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
from geometry_msgs.msg import Transform, PoseStamped, PointStamped
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker, MarkerArray   
from tf2_ros import TransformBroadcaster
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


class LapTimeTracker:
    def __init__(self):
        self.start_time = None
        self.elapsed_time = 0
        self.running = False

    def start(self):
        if not self.running:
            self.start_time = time.time() - self.elapsed_time
            self.running = True

    def get_elapsed_time(self):
        if self.running:
            return time.time() - self.start_time
        return self.elapsed_time

    def reset(self, restart=True):
        self.start_time = None  # Reset to None
        self.elapsed_time = 0
        self.running = False
        if restart:
            self.start()

    def pause(self):
        if self.running:
            self.elapsed_time = time.time() - self.start_time
            self.running = False


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
        self.agent_lap_completed = [False] * self.num_agents
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

        self.pose_reset_arr = np.zeros((self.num_agents, 3))
        for i in range(len(self.pose_reset_arr)):
            # spawn racecars in a vertical column with 1.0 meters of distance between each
            self.pose_reset_arr[i][0] += i

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
        # - Lap times: 10 Hz (100ms) - UI update rate
        self.drive_timer = self.create_timer(0.01, self.drive_timer_callback)  # 100 Hz physics
        self.timer = self.create_timer(sensor_period, self.timer_callback)  # Adaptive sensor publish
        self.lap_time_timer = self.create_timer(0.1, self.publish_lap_times)  # 10 Hz lap times
        self.get_logger().info(f"Sensor publish rate: {sensor_rate} Hz for {self.num_agents} agents")
        self.lap_time_publisher = self.create_publisher(
            String, 'lap_times', 10
        )

        # transform broadcaster
        self.br = TransformBroadcaster(self)

        self.active_racecar_to_reset_pose = 0

        self.lap_time_trackers = [LapTimeTracker()
                                  for _ in range(self.num_agents)]
        self.best_lap_times = [0.0] * self.num_agents
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

        # Define QoS profile for some publishers
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )

        for i in range(self.num_agents):
            racecar_scan_pub = self.create_publisher(
                LaserScan, self.scan_topics[i], qos_profile=qos_profile
            )
            racecar_odom_pub = self.create_publisher(
                Odometry, self.odom_topics[i], qos_profile=qos_profile
            )
            racecar_drive_sub = self.create_subscription(
                AckermannDriveStamped, self.drive_topics[i], self.drive_callback, qos_profile=qos_profile
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
            self.trajectory_history.append(deque(maxlen=500))  # Last 500 positions
        
        # Trajectory publish timer (5 Hz - less frequent than main loop)
        self.trajectory_timer = self.create_timer(0.2, self._publish_trajectories)
        
        # Additional visualization publishers
        self.velocity_arrow_pub = self.create_publisher(MarkerArray, 'velocity_arrows', 10)
        self.safety_zone_pub = self.create_publisher(MarkerArray, 'safety_zones', 10)
        self.race_markers_pub = self.create_publisher(MarkerArray, 'race_markers', 10)
        self.race_stats_pub = self.create_publisher(String, 'race_stats_json', 10)
        
        # Publish static race markers (start/finish line) once
        self._start_finish_published = False

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

        self.racecar_drive_published = False
        self.last_log_time = time.time()
        self.pose_reset_sub = self.create_subscription(
            PoseWithCovarianceStamped, "/initialpose", self.pose_reset_callback, 10
        )
        
        # Obstacle spawning via clicked_point (from RViz Publish Point tool)
        self.clicked_point_sub = self.create_subscription(
            PointStamped, "/clicked_point", self.clicked_point_callback, 10
        )
        self.clear_obstacles_sub = self.create_subscription(
            Bool, "clear_obstacles", self.clear_obstacles_callback, 10
        )
        self.obstacles = []  # List of (x, y, radius) tuples
        self.obstacle_marker_pub = self.create_publisher(MarkerArray, 'obstacles', 10)
        
        # Lap point marker publisher
        self.lap_point_pub = self.create_publisher(MarkerArray, 'lap_point', 10)
        self.lap_point_position = (0.0, 0.0)  # Default lap point at origin
        self._lap_point_published = False

        # Track last drive message time for each racecar to detect stale commands
        self.last_drive_time = [time.time()] * self.num_agents
        self.drive_timeout = 0.1  # 100ms timeout - stop car if no command received

        # Auto-start simulation if enabled (removes need for Start button click)
        if self.get_parameter("auto_start").value:
            self.simulation_running = True
            for tracker in self.lap_time_trackers:
                tracker.start()
            self.get_logger().info("Simulation auto-started")

    def handle_collision(self):
        """Checks for collision and handles if there is any.
        
        Optimized: Uses vectorized numpy min instead of O(n × 1080) loop.
        """
        collision_threshold = 0.21
        min_scans = np.min(self.obs["scans"], axis=1)  # Vectorized - one op for all agents
        for i in range(self.num_agents):
            if not self.agent_disqualified[i] and min_scans[i] <= collision_threshold:
                self.agent_disqualified[i] = True
                self.lap_time_trackers[i].reset(restart=False)
                self.get_logger().info(f"Racecar {i + 1} is disqualified")

    def choose_active_racecar_callback(self, racecar_index):
        """Handle racecar selection from plugin dropdown."""
        self.active_racecar_to_reset_pose = racecar_index.data
        # Disable spawn_all mode when manually selecting a racecar
        self.spawn_all_mode = False

    def spawn_all_mode_callback(self, msg):
        """Toggle spawn_all mode from plugin."""
        self.spawn_all_mode = msg.data
        if msg.data:
            self.get_logger().info("Spawn All mode activated - next pose estimate will spawn all racecars")

    def clicked_point_callback(self, msg):
        """Add an obstacle at the clicked point location."""
        x = msg.point.x
        y = msg.point.y
        radius = 0.3  # Default obstacle radius
        
        self.obstacles.append((x, y, radius))
        self.get_logger().info(f"Added obstacle at ({x:.2f}, {y:.2f})")
        self._publish_obstacle_markers()

    def clear_obstacles_callback(self, msg):
        """Clear all obstacles when triggered."""
        if msg.data:
            self.obstacles.clear()
            self._publish_obstacle_markers()
            self.get_logger().info("Cleared all obstacles")

    def _publish_obstacle_markers(self):
        """Publish markers for all obstacles."""
        arr = MarkerArray()
        
        # First, delete all existing markers
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        arr.markers.append(delete_marker)
        
        # Add markers for each obstacle
        ts = self.get_clock().now().to_msg()
        for i, (x, y, radius) in enumerate(self.obstacles):
            m = Marker()
            m.header.stamp = ts
            m.header.frame_id = "map"
            m.ns = "obstacles"
            m.id = i
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.position.z = 0.15  # Half height
            m.scale.x = radius * 2
            m.scale.y = radius * 2
            m.scale.z = 0.3  # Height
            m.color.r = 0.8
            m.color.g = 0.2
            m.color.b = 0.2
            m.color.a = 0.9
            arr.markers.append(m)
        
        self.obstacle_marker_pub.publish(arr)

    def _apply_obstacles_to_scan(self, racecar_idx):
        """Apply obstacle intersections to laser scan data.
        
        For each scan ray, check if it intersects any obstacle closer than
        the original scan reading. If so, use the obstacle distance.
        """
        original_ranges = list(self.obs["scans"][racecar_idx])
        
        if not self.obstacles:
            return original_ranges
        
        # Get racecar position and orientation
        car_x = self.obs["poses_x"][racecar_idx]
        car_y = self.obs["poses_y"][racecar_idx]
        car_theta = self.obs["poses_theta"][racecar_idx]
        
        # Scan parameters
        angle_min = self.angle_min
        angle_inc = self.angle_inc
        num_rays = len(original_ranges)
        
        modified_ranges = original_ranges.copy()
        
        for ray_idx in range(num_rays):
            # Calculate ray angle in world frame
            ray_angle_local = angle_min + ray_idx * angle_inc
            ray_angle_world = car_theta + ray_angle_local
            
            # Ray direction
            ray_dx = np.cos(ray_angle_world)
            ray_dy = np.sin(ray_angle_world)
            
            # Check intersection with each obstacle
            for obs_x, obs_y, obs_radius in self.obstacles:
                # Vector from car to obstacle center
                to_obs_x = obs_x - car_x
                to_obs_y = obs_y - car_y
                
                # Project onto ray direction
                proj_dist = to_obs_x * ray_dx + to_obs_y * ray_dy
                
                if proj_dist <= 0:
                    continue  # Obstacle is behind the ray origin
                
                # Perpendicular distance from ray to obstacle center
                perp_dist = abs(to_obs_x * ray_dy - to_obs_y * ray_dx)
                
                if perp_dist < obs_radius:
                    # Ray intersects obstacle
                    # Calculate intersection distance (approximate - entry point)
                    intersect_dist = proj_dist - np.sqrt(max(0, obs_radius**2 - perp_dist**2))
                    
                    if intersect_dist > 0 and intersect_dist < modified_ranges[ray_idx]:
                        modified_ranges[ray_idx] = intersect_dist
        
        return modified_ranges

    def _publish_lap_point_marker(self, ts):
        """Publish the lap/finish line marker - checkered pattern like real races."""
        if self._lap_point_published:
            return
        
        arr = MarkerArray()
        
        # Create a checkered finish line pattern
        line_width = 4.0  # Total width of the line
        line_depth = 0.5  # Depth (thickness) of the line
        checker_size = 0.25  # Size of each checker square
        num_checkers = int(line_width / checker_size)
        
        marker_id = 0
        for i in range(num_checkers):
            for j in range(2):  # 2 rows of checkers
                m = Marker()
                m.header.stamp = ts
                m.header.frame_id = "map"
                m.ns = "lap_point"
                m.id = marker_id
                marker_id += 1
                m.type = Marker.CUBE
                m.action = Marker.ADD
                
                # Position checkers in a line perpendicular to track at origin
                m.pose.position.x = self.lap_point_position[0] + (j - 0.5) * checker_size
                m.pose.position.y = self.lap_point_position[1] + (i - num_checkers/2) * checker_size
                m.pose.position.z = 0.01
                
                m.scale.x = checker_size * 0.95
                m.scale.y = checker_size * 0.95
                m.scale.z = 0.02
                
                # Alternating black and white pattern
                if (i + j) % 2 == 0:
                    m.color.r = m.color.g = m.color.b = 1.0  # White
                else:
                    m.color.r = m.color.g = m.color.b = 0.1  # Black
                m.color.a = 1.0
                
                arr.markers.append(m)
        
        # Finish line text
        text = Marker()
        text.header.stamp = ts
        text.header.frame_id = "map"
        text.ns = "lap_point"
        text.id = marker_id
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = self.lap_point_position[0]
        text.pose.position.y = self.lap_point_position[1]
        text.pose.position.z = 0.5
        text.scale.z = 0.3
        text.color.r = 1.0
        text.color.g = 1.0
        text.color.b = 1.0
        text.color.a = 1.0
        text.text = "FINISH"
        arr.markers.append(text)
        
        self.lap_point_pub.publish(arr)
        self._lap_point_published = True

    def is_lap_completed(self, racecar_index):
        threshold_distance = 2.0
        current_x = self.obs["poses_x"][racecar_index]
        current_y = self.obs["poses_y"][racecar_index]
        distance_from_origin = np.sqrt(current_x**2 + current_y**2)

        if distance_from_origin <= threshold_distance:
            if not self.agent_lap_completed[racecar_index]:
                self.agent_lap_completed[racecar_index] = True
                completion_time = self.lap_time_trackers[racecar_index].get_elapsed_time()
                if completion_time > 5.0:
                    self.best_lap_times[racecar_index] = completion_time
                return True
            else:
                return False
        else:
            self.agent_lap_completed[racecar_index] = False
            return False

    def start_callback(self, msg):
        if msg.data:
            self.simulation_running = True
            self.simulation_paused = False
            self.get_logger().info('Simulation started')
            for tracker in self.lap_time_trackers:
                tracker.start()

    def pause_callback(self, msg):
        if msg.data:
            self.simulation_paused = not self.simulation_paused
            if self.simulation_paused:
                for tracker in self.lap_time_trackers:
                    tracker.pause()
                self.get_logger().info('Tracker and Simulation paused')
            else:
                for tracker in self.lap_time_trackers:
                    tracker.start()
                self.get_logger().info('Tracker and Simulation resumed')

    def reset_callback(self, msg):
        """Reset all agents to their starting positions."""
        if msg.data:
            self.get_logger().info('Resetting race...')
            # Reset positions to start
            self.pose_reset_arr = self.start_positions.copy()
            self.obs, _, self.done, _ = self.env.reset(np.array(self.pose_reset_arr))
            # Clear disqualifications
            self.agent_disqualified = [False] * self.num_agents
            # Reset lap trackers
            for tracker in self.lap_time_trackers:
                tracker.reset(restart=True)
            # Reset best lap times
            self.best_lap_times = [0.0] * self.num_agents
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

            # ALWAYS step physics - this is critical for smooth simulation!
            # Don't wait for drive messages - use last known commands
            self.obs, _, self.done, _ = self.env.step(self.drive_msgs)

            # Handle collision detection (once per step, not per agent)
            self.handle_collision()

            # Check lap completion for each agent
            for agent_idx in range(self.num_agents):
                if self.is_lap_completed(agent_idx):
                    self.lap_time_trackers[agent_idx].reset(restart=True)
                    completion_time = self.lap_time_trackers[agent_idx].get_elapsed_time()
                    if completion_time > 5.0:
                        self.get_logger().info(
                            f"Racecar {agent_idx + 1} completed a lap with time: {self.lap_time_trackers[agent_idx].get_elapsed_time():.4f}")

    def timer_callback(self):
        if self.simulation_running and not self.simulation_paused:
            ts = self.get_clock().now().to_msg()

            # Publish scans for each agent (using pre-allocated messages)
            for i in range(self.num_agents):
                self._scan_msgs[i].header.stamp = ts
                # Apply obstacle modifications to scan data
                ranges = self._apply_obstacles_to_scan(i)
                self._scan_msgs[i].ranges = ranges
                self.scan_publishers[i].publish(self._scan_msgs[i])

            # Publish transforms, odom, and markers ONCE (they loop internally)
            self._publish_odom(ts)
            self._publish_transforms(ts)
            self._publish_laser_transforms(ts)
            self._publish_wheel_transforms(ts)
            self._publish_name_markers(ts)
            self._publish_speed_markers(ts)
            self._publish_velocity_arrows(ts)
            self._publish_safety_zones(ts)
            self._publish_start_finish_line(ts)
            self._publish_lap_point_marker(ts)
            self._publish_race_stats_json(ts)
            
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

        if self.spawn_all_mode:
            # Spawn all racecars in a line from the pose estimate
            # Calculate perpendicular direction for line formation
            perp_theta = rtheta + np.pi / 2  # Perpendicular to facing direction
            spacing = 1.0  # 1 meter between racecars
            
            for i in range(self.num_agents):
                # Offset from center: 0 is center, others spread out
                offset = (i - (self.num_agents - 1) / 2) * spacing
                self.pose_reset_arr[i][0] = rx + offset * np.cos(perp_theta)
                self.pose_reset_arr[i][1] = ry + offset * np.sin(perp_theta)
                self.pose_reset_arr[i][2] = rtheta
                # Clear disqualification for all racecars
                self.agent_disqualified[i] = False
                self.lap_time_trackers[i].reset(restart=True)
            
            self.get_logger().info(f"Spawned all {self.num_agents} racecars in line formation")
            self.spawn_all_mode = False  # Reset spawn_all mode after use
        else:
            # Single racecar pose reset (original behavior)
            for i in range(self.num_agents):
                self.pose_reset_arr[i][0] = self.obs["poses_x"][i]
                self.pose_reset_arr[i][1] = self.obs["poses_y"][i]
                self.pose_reset_arr[i][2] = self.obs["poses_theta"][i]

            try:
                # When a pose reset is sent, racecar stops being disqualified
                if self.agent_disqualified[self.active_racecar_to_reset_pose]:
                    self.agent_disqualified[self.active_racecar_to_reset_pose] = False
                    self.lap_time_trackers[self.active_racecar_to_reset_pose].reset(restart=True)

                self.pose_reset_arr[self.active_racecar_to_reset_pose][0] = rx
                self.pose_reset_arr[self.active_racecar_to_reset_pose][1] = ry
                self.pose_reset_arr[self.active_racecar_to_reset_pose][2] = rtheta
            except IndexError:
                self.get_logger().warn("Chosen racecar does not exist")

        self.obs, _, self.done, _ = self.env.reset(
            np.array(self.pose_reset_arr))

    def _publish_odom(self, ts):
        """Publish odometry for all agents using pre-allocated messages."""
        for i in range(self.num_agents):
            odom = self._odom_msgs[i]
            odom.header.stamp = ts
            odom.pose.pose.position.x = self.obs["poses_x"][i]
            odom.pose.pose.position.y = self.obs["poses_y"][i]
            racecar_quat = euler.euler2quat(
                0.0, 0.0, self.obs["poses_theta"][i], axes="sxyz")
            odom.pose.pose.orientation.x = racecar_quat[1]
            odom.pose.pose.orientation.y = racecar_quat[2]
            odom.pose.pose.orientation.z = racecar_quat[3]
            odom.pose.pose.orientation.w = racecar_quat[0]
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
            m.pose.position.z = 0.5      # half‑meter above roof
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
            # Calculate speed from velocity components
            speed = np.sqrt(
                self.obs["linear_vels_x"][i]**2 + 
                self.obs["linear_vels_y"][i]**2
            )
            
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
        for i in range(self.num_agents):
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = self.obs["poses_x"][i]
            pose.pose.position.y = self.obs["poses_y"][i]
            pose.pose.position.z = 0.0
            quat = euler.euler2quat(0.0, 0.0, self.obs["poses_theta"][i], axes="sxyz")
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
            if len(self.trajectory_history[i]) < 2:
                continue
            
            path = Path()
            path.header.stamp = ts
            path.header.frame_id = "map"
            path.poses = list(self.trajectory_history[i])
            self.path_publishers[i].publish(path)

    def _publish_velocity_arrows(self, ts):
        """Publish arrow markers showing velocity direction and magnitude."""
        arr = MarkerArray()
        for i in range(self.num_agents):
            ns = f"{self.racecar_namespace}{i+1}"
            vx = self.obs["linear_vels_x"][i]
            vy = self.obs["linear_vels_y"][i]
            speed = np.sqrt(vx**2 + vy**2)
            
            m = Marker()
            m.header.stamp = ts
            m.header.frame_id = f"{ns}/base_link"
            m.ns = "velocity_arrows"
            m.id = i
            m.type = Marker.ARROW
            m.action = Marker.ADD
            
            # Arrow points in direction of velocity, length proportional to speed
            m.scale.x = max(0.1, speed * 0.3)  # Arrow length
            m.scale.y = 0.05  # Arrow width
            m.scale.z = 0.05  # Arrow height
            
            # Color gradient: green (slow) -> red (fast)
            speed_ratio = min(speed / 8.0, 1.0)
            if speed_ratio < 0.5:
                m.color.r = speed_ratio * 2.0
                m.color.g = 1.0
            else:
                m.color.r = 1.0
                m.color.g = 1.0 - (speed_ratio - 0.5) * 2.0
            m.color.b = 0.0
            m.color.a = 0.8
            
            arr.markers.append(m)
        self.velocity_arrow_pub.publish(arr)

    def _publish_safety_zones(self, ts):
        """Publish transparent circles around each agent showing safety zone."""
        arr = MarkerArray()
        collision_threshold = 0.21
        warning_threshold = 0.5
        
        for i in range(self.num_agents):
            ns = f"{self.racecar_namespace}{i+1}"
            min_scan = np.min(self.obs["scans"][i])
            
            m = Marker()
            m.header.stamp = ts
            m.header.frame_id = f"{ns}/base_link"
            m.ns = "safety_zones"
            m.id = i
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            
            m.pose.position.z = 0.01  # Slightly above ground
            m.scale.x = 0.6  # Diameter
            m.scale.y = 0.6
            m.scale.z = 0.02  # Very thin
            
            # Color based on proximity to obstacles
            if self.agent_disqualified[i]:
                m.color.r, m.color.g, m.color.b = 0.5, 0.5, 0.5  # Gray if disqualified
            elif min_scan <= collision_threshold:
                m.color.r, m.color.g, m.color.b = 1.0, 0.0, 0.0  # Red - collision!
            elif min_scan <= warning_threshold:
                m.color.r, m.color.g, m.color.b = 1.0, 1.0, 0.0  # Yellow - warning
            else:
                m.color.r, m.color.g, m.color.b = 0.0, 1.0, 0.0  # Green - safe
            m.color.a = 0.3
            
            arr.markers.append(m)
        self.safety_zone_pub.publish(arr)

    def _publish_start_finish_line(self, ts):
        """Publish start/finish line marker (once)."""
        if self._start_finish_published:
            return
        
        arr = MarkerArray()
        
        # Start/Finish line marker
        line = Marker()
        line.header.stamp = ts
        line.header.frame_id = "map"
        line.ns = "race_markers"
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = 0.1  # Line width
        line.color.r = 1.0
        line.color.g = 1.0
        line.color.b = 1.0
        line.color.a = 1.0
        
        # Line across start position (perpendicular to track at origin)
        from geometry_msgs.msg import Point
        p1 = Point()
        p1.x, p1.y, p1.z = -1.0, -2.0, 0.05
        p2 = Point()
        p2.x, p2.y, p2.z = -1.0, 2.0, 0.05
        line.points = [p1, p2]
        arr.markers.append(line)
        
        # START/FINISH text
        text = Marker()
        text.header.stamp = ts
        text.header.frame_id = "map"
        text.ns = "race_markers"
        text.id = 1
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = -1.0
        text.pose.position.y = 0.0
        text.pose.position.z = 0.5
        text.scale.z = 0.4
        text.color.r = 1.0
        text.color.g = 1.0
        text.color.b = 0.0
        text.color.a = 1.0
        text.text = "START / FINISH"
        arr.markers.append(text)
        
        self.race_markers_pub.publish(arr)
        self._start_finish_published = True

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
            speed = np.sqrt(
                self.obs["linear_vels_x"][i]**2 + 
                self.obs["linear_vels_y"][i]**2
            )
            min_scan = float(np.min(self.obs["scans"][i]))
            
            agent_stats = {
                "id": i + 1,
                "name": f"{self.racecar_namespace}{i+1}",
                "position": {
                    "x": float(self.obs["poses_x"][i]),
                    "y": float(self.obs["poses_y"][i]),
                    "theta": float(self.obs["poses_theta"][i])
                },
                "speed": float(speed),
                "min_scan_distance": min_scan,
                "current_lap_time": self.lap_time_trackers[i].get_elapsed_time(),
                "best_lap_time": self.best_lap_times[i],
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
            racecar_quat = euler.euler2quat(
                0.0, 0.0, self.obs["poses_theta"][i], axes="sxyz")
            racecar_t.rotation.x = racecar_quat[1]
            racecar_t.rotation.y = racecar_quat[2]
            racecar_t.rotation.z = racecar_quat[3]
            racecar_t.rotation.w = racecar_quat[0]
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

    def _publish_laser_transforms(self, ts):
        """Publish laser transforms for all agents in a single batch."""
        transforms = []
        for i in range(self.num_agents):
            racecar_namespace = self.racecar_namespace + str(i + 1)
            racecar_scan_ts = TransformStamped()
            racecar_scan_ts.transform.translation.x = self.scan_distance_to_base_link
            racecar_scan_ts.transform.rotation.w = 1.0
            racecar_scan_ts.header.stamp = ts
            racecar_scan_ts.header.frame_id = racecar_namespace + "/base_link"
            racecar_scan_ts.child_frame_id = racecar_namespace + "/laser"
            transforms.append(racecar_scan_ts)
        self.br.sendTransform(transforms)  # Single batch call

    def publish_lap_times(self):
        if not self.simulation_paused and self.simulation_running:
            lap_times_str = ""
            lap_times_msg = String()
            for agent_idx, tracker in enumerate(self.lap_time_trackers):
                elapsed_time = tracker.get_elapsed_time()
                lap_times_str += f"Racecar {agent_idx + 1}: Current Lap: {elapsed_time:.4f} Best Lap: {self.best_lap_times[agent_idx]:.4f}{' Disqualified' if self.agent_disqualified[agent_idx] else ''}\n"
            lap_times_msg.data = lap_times_str
            self.lap_time_publisher.publish(lap_times_msg)
        else:
            for tracker in self.lap_time_trackers:
                tracker.pause()


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
