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
import rclpy
import numpy as np
from transforms3d import euler
from rclpy.node import Node

from f1tenth_gym_ros.spawn_utils import get_spawn_positions
from std_msgs.msg import Bool, Int32, String
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Point
from geometry_msgs.msg import Transform
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker, MarkerArray   
from tf2_ros import TransformBroadcaster
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


class TrajectoryTracker:
    """Track agent trajectory history for visualization."""

    def __init__(self, max_points: int = 500, sample_distance: float = 0.1):
        self.max_points = max_points
        self.sample_distance = sample_distance
        self.points = []  # List of (x, y, theta) tuples
        self.last_point = None

    def add_point(self, x: float, y: float, theta: float):
        """Add a point if it's far enough from the last point."""
        if self.last_point is None:
            self.points.append((x, y, theta))
            self.last_point = (x, y)
        else:
            dist = np.sqrt((x - self.last_point[0])**2 + (y - self.last_point[1])**2)
            if dist >= self.sample_distance:
                self.points.append((x, y, theta))
                self.last_point = (x, y)
                if len(self.points) > self.max_points:
                    self.points.pop(0)

    def reset(self):
        """Clear trajectory history."""
        self.points.clear()
        self.last_point = None

    def get_points(self):
        """Return list of trajectory points."""
        return self.points


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
        self.declare_parameter("spawn_strategy", "auto")
        self.declare_parameter("spawn_min_distance", 1.5)
        self.declare_parameter("spawn_vehicle_radius", 0.22)
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
            Int32, 'racecar_to_estimate_pose', self.choose_active_agent_callback, 10)

        # Additional attributes to manage the state of the simulation
        self.simulation_running = False
        self.simulation_paused = False
        self.agent_lap_completed = [False] * self.num_agents
        self.agent_disqualified = [False] * self.num_agents

        # env backend
        try:
            self.map_path = self.get_parameter("map_path").value
            self.map_img_ext = self.get_parameter("map_img_ext").value
            if not self.map_path:
                raise ValueError("map_path is empty")
            self.env = gym.make(
                "f110_gym:f110-v0",
                map=self.map_path,
                map_ext=self.map_img_ext,
                num_agents=self.num_agents,
            )
        except Exception as e:
            self.get_logger().warn(
                f'Given map path can not be found. Defaulting to example_map.')
            self.map_path = default_map_path
            self.map_img_ext = '.png'
            self.env = gym.make(
                "f110_gym:f110-v0",
                map=self.map_path,
                map_ext=self.map_img_ext,
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

        # Get spawn configuration
        spawn_strategy = self.get_parameter("spawn_strategy").value
        spawn_min_distance = self.get_parameter("spawn_min_distance").value
        spawn_vehicle_radius = self.get_parameter("spawn_vehicle_radius").value

        # Auto-spawn: find valid positions based on map occupancy grid
        if spawn_strategy != "manual":
            try:
                map_yaml_path = self.map_path + ".yaml"
                self.pose_reset_arr = get_spawn_positions(
                    map_yaml_path,
                    self.num_agents,
                    min_distance=spawn_min_distance,
                    vehicle_radius=spawn_vehicle_radius,
                    strategy=spawn_strategy
                )
                self.get_logger().info(
                    f"Auto-spawned {self.num_agents} agents using '{spawn_strategy}' strategy"
                )
            except Exception as e:
                self.get_logger().warn(
                    f"Auto-spawn failed: {e}. Using fallback manual spawn."
                )
                # Fallback to manual spawn
                self.pose_reset_arr = np.zeros((self.num_agents, 3))
                for i in range(self.num_agents):
                    self.pose_reset_arr[i][0] = i
        else:
            # Manual spawn: hardcoded positions
            self.pose_reset_arr = np.zeros((self.num_agents, 3))
            for i in range(self.num_agents):
                self.pose_reset_arr[i][0] = i

        self.obs, _, self.done, _ = self.env.reset(self.pose_reset_arr)

        # Timer rates optimized for smooth simulation:
        # - Physics: 100 Hz (10ms) - matches typical simulator timestep
        # - Sensor publish: adaptive based on agent count
        # - Lap times: 10 Hz (100ms) - UI update rate
        self.drive_timer = self.create_timer(0.01, self.drive_timer_callback)  # 100 Hz physics

        # Adaptive sensor publish rate based on agent count (performance optimization)
        if self.num_agents <= 5:
            sensor_period = 0.025  # 40 Hz for small agent counts
        elif self.num_agents <= 10:
            sensor_period = 0.04   # 25 Hz for medium agent counts
        else:
            sensor_period = 0.05   # 20 Hz for large agent counts (20 agents)

        self.timer = self.create_timer(sensor_period, self.timer_callback)
        self.get_logger().info(
            f"Sensor publish rate: {1.0/sensor_period:.0f} Hz (adaptive for {self.num_agents} agents)"
        )
        self.lap_time_timer = self.create_timer(0.1, self.publish_lap_times)  # 10 Hz lap times
        self.lap_time_publisher = self.create_publisher(
            String, 'lap_times', 10
        )

        # transform broadcaster
        self.br = TransformBroadcaster(self)

        self.active_agent_to_reset_pose = 0

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

        # === Visualization Publishers ===
        # Trajectory paths for each agent
        self.trajectory_trackers = [TrajectoryTracker() for _ in range(self.num_agents)]
        self.trajectory_publishers = [
            self.create_publisher(Path, f"{self.racecar_namespace}{i+1}/trajectory", 10)
            for i in range(self.num_agents)
        ]

        # Velocity vectors visualization
        self.velocity_marker_pub = self.create_publisher(
            MarkerArray, 'velocity_vectors', 10)

        # Safety zones visualization
        self.safety_zone_pub = self.create_publisher(
            MarkerArray, 'safety_zones', 10)

        # Trajectory visualization timer (5 Hz - lower rate for paths)
        self.viz_timer = self.create_timer(0.2, self.publish_visualizations)

        self.racecar_drive_published = False
        self.last_log_time = time.time()
        self.pose_reset_sub = self.create_subscription(
            PoseWithCovarianceStamped, "/initialpose", self.pose_reset_callback, 10
        )

        # Track last drive message time for each agent to detect stale commands
        self.last_drive_time = [time.time()] * self.num_agents
        self.drive_timeout = 0.1  # 100ms timeout - stop car if no command received

    def handle_collision(self):
        """Checks for collision and handles if there is any.

        Optimized: Uses vectorized NumPy min() instead of iterating all 1080 beams.
        For 20 agents, this reduces from 21,600 comparisons to 20 min operations.
        """
        collision_threshold = 0.21
        # Vectorized: get minimum scan distance for each agent in one operation
        min_scans = np.min(self.obs["scans"], axis=1)

        for racecar_idx in range(self.num_agents):
            if not self.agent_disqualified[racecar_idx] and min_scans[racecar_idx] <= collision_threshold:
                self.agent_disqualified[racecar_idx] = True
                self.lap_time_trackers[racecar_idx].reset(restart=False)
                self.get_logger().info(
                    f"Agent {racecar_idx + 1} is disqualified")

    def choose_active_agent_callback(self, agent_index):
        self.active_agent_to_reset_pose = agent_index.data

    def is_lap_completed(self, agent_index):
        threshold_distance = 2.0
        current_x = self.obs["poses_x"][agent_index]
        current_y = self.obs["poses_y"][agent_index]
        distance_from_origin = np.sqrt(current_x**2 + current_y**2)

        if distance_from_origin <= threshold_distance:
            if not self.agent_lap_completed[agent_index]:
                self.agent_lap_completed[agent_index] = True
                completion_time = self.lap_time_trackers[agent_index].get_elapsed_time(
                )
                if completion_time > 5.0:
                    self.best_lap_times[agent_index] = completion_time
                return True
            else:
                return False
        else:
            self.agent_lap_completed[agent_index] = False
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
        """Reset simulation to initial state."""
        if msg.data:
            self.simulation_running = False
            self.simulation_paused = False

            # Reset all agents to starting positions
            # Re-run auto-spawn to get fresh positions
            spawn_strategy = self.get_parameter("spawn_strategy").value
            spawn_min_distance = self.get_parameter("spawn_min_distance").value
            spawn_vehicle_radius = self.get_parameter("spawn_vehicle_radius").value

            if spawn_strategy != "manual":
                try:
                    map_yaml_path = self.map_path + ".yaml"
                    self.pose_reset_arr = get_spawn_positions(
                        map_yaml_path,
                        self.num_agents,
                        min_distance=spawn_min_distance,
                        vehicle_radius=spawn_vehicle_radius,
                        strategy=spawn_strategy
                    )
                except Exception:
                    # Fallback to manual spawn
                    self.pose_reset_arr = np.zeros((self.num_agents, 3))
                    for i in range(self.num_agents):
                        self.pose_reset_arr[i][0] = i
            else:
                self.pose_reset_arr = np.zeros((self.num_agents, 3))
                for i in range(self.num_agents):
                    self.pose_reset_arr[i][0] = i

            # Reset environment
            self.obs, _, self.done, _ = self.env.reset(np.array(self.pose_reset_arr))

            # Reset lap trackers, trajectory trackers, and disqualification status
            for i in range(self.num_agents):
                self.lap_time_trackers[i].reset(restart=False)
                self.trajectory_trackers[i].reset()
                self.agent_disqualified[i] = False
                self.agent_lap_completed[i] = False
                self.best_lap_times[i] = 0.0

            self.get_logger().info('Simulation reset to initial state')

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
                            f"Agent {agent_idx + 1} completed a lap with time: {self.lap_time_trackers[agent_idx].get_elapsed_time():.4f}")

    def timer_callback(self):
        if self.simulation_running and not self.simulation_paused:
            ts = self.get_clock().now().to_msg()

            # Publish scans for each agent
            for i in range(self.num_agents):
                racecar_namespace = self.racecar_namespace + str(i + 1)
                scan = LaserScan()
                scan.header.stamp = ts
                scan.header.frame_id = racecar_namespace + "/laser"
                scan.angle_min = self.angle_min
                scan.angle_max = self.angle_max
                scan.angle_increment = self.angle_inc
                scan.range_min = 0.0
                scan.range_max = 30.0
                scan.ranges = list(self.obs["scans"][i])
                self.scan_publishers[i].publish(scan)

            # Publish transforms, odom, and markers ONCE (they loop internally)
            self._publish_odom(ts)
            self._publish_all_transforms_batched(ts)  # Batched TF broadcast (optimized)
            self._publish_name_markers(ts)

    def pose_reset_callback(self, pose_msg):
        rx = pose_msg.pose.pose.position.x
        ry = pose_msg.pose.pose.position.y
        rqx = pose_msg.pose.pose.orientation.x
        rqy = pose_msg.pose.pose.orientation.y
        rqz = pose_msg.pose.pose.orientation.z
        rqw = pose_msg.pose.pose.orientation.w
        _, _, rtheta = euler.quat2euler([rqw, rqx, rqy, rqz], axes="rxyz")

        for i in range(self.num_agents):
            self.pose_reset_arr[i][0] = self.obs["poses_x"][i]
            self.pose_reset_arr[i][1] = self.obs["poses_y"][i]
            self.pose_reset_arr[i][2] = self.obs["poses_theta"][i]

        try:
            # When a pose reset it sent, agent stops becoming disqualified
            if self.agent_disqualified[self.active_agent_to_reset_pose]:
                self.agent_disqualified[self.active_agent_to_reset_pose] = False
                self.lap_time_trackers[self.active_agent_to_reset_pose].reset(restart=True)

            self.pose_reset_arr[self.active_agent_to_reset_pose][0] = rx
            self.pose_reset_arr[self.active_agent_to_reset_pose][1] = ry
            self.pose_reset_arr[self.active_agent_to_reset_pose][2] = rtheta
        except IndexError:
            self.get_logger().warn("Chosen agent does not exist")

        self.obs, _, self.done, _ = self.env.reset(
            np.array(self.pose_reset_arr))

    def _publish_odom(self, ts):
        for i in range(self.num_agents):
            racecar_namespace = self.racecar_namespace + str(i + 1)
            racecar_odom = Odometry()
            racecar_odom.header.stamp = ts
            racecar_odom.header.frame_id = "map"
            racecar_odom.child_frame_id = racecar_namespace + "/base_link"
            racecar_odom.pose.pose.position.x = self.obs["poses_x"][i]
            racecar_odom.pose.pose.position.y = self.obs["poses_y"][i]
            racecar_quat = euler.euler2quat(
                0.0, 0.0, self.obs["poses_theta"][i], axes="sxyz")
            racecar_odom.pose.pose.orientation.x = racecar_quat[1]
            racecar_odom.pose.pose.orientation.y = racecar_quat[2]
            racecar_odom.pose.pose.orientation.z = racecar_quat[3]
            racecar_odom.pose.pose.orientation.w = racecar_quat[0]
            racecar_odom.twist.twist.linear.x = self.obs["linear_vels_x"][i]
            racecar_odom.twist.twist.linear.y = self.obs["linear_vels_y"][i]
            racecar_odom.twist.twist.angular.z = self.obs["ang_vels_z"][i]
            self.odom_publishers[i].publish(racecar_odom)

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
            r, g, b    = (1.0,0.0,0.0) if i==0 else (0.0,1.0,0.0) if i==1 else (0.0,0.0,1.0)
            m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, 1.0
            m.text = ns.replace("racecar", "agent")
            arr.markers.append(m)
        self.marker_pub.publish(arr)

    def publish_visualizations(self):
        """Publish all visualization markers (trajectories, velocity, safety zones)."""
        if not self.simulation_running or self.simulation_paused:
            return

        ts = self.get_clock().now().to_msg()

        # Update trajectory trackers with current positions
        for i in range(self.num_agents):
            self.trajectory_trackers[i].add_point(
                self.obs["poses_x"][i],
                self.obs["poses_y"][i],
                self.obs["poses_theta"][i]
            )

        # Publish trajectory paths
        self._publish_trajectories(ts)

        # Publish velocity vectors
        self._publish_velocity_markers(ts)

        # Publish safety zones
        self._publish_safety_zones(ts)

    def _publish_trajectories(self, ts):
        """Publish trajectory path for each agent."""
        for i in range(self.num_agents):
            path = Path()
            path.header.stamp = ts
            path.header.frame_id = "map"

            for x, y, theta in self.trajectory_trackers[i].get_points():
                pose = PoseStamped()
                pose.header = path.header
                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.position.z = 0.0
                quat = euler.euler2quat(0.0, 0.0, theta, axes="sxyz")
                pose.pose.orientation.w = quat[0]
                pose.pose.orientation.x = quat[1]
                pose.pose.orientation.y = quat[2]
                pose.pose.orientation.z = quat[3]
                path.poses.append(pose)

            self.trajectory_publishers[i].publish(path)

    def _publish_velocity_markers(self, ts):
        """Publish velocity arrow markers for all agents."""
        arr = MarkerArray()

        for i in range(self.num_agents):
            ns = f"{self.racecar_namespace}{i+1}"

            m = Marker()
            m.header.stamp = ts
            m.header.frame_id = f"{ns}/base_link"
            m.ns = "velocity_vectors"
            m.id = i
            m.type = Marker.ARROW
            m.action = Marker.ADD

            # Arrow from origin pointing forward
            m.pose.position.z = 0.15

            # Scale arrow by velocity magnitude
            vel_x = self.obs["linear_vels_x"][i]
            vel_y = self.obs["linear_vels_y"][i]
            speed = np.sqrt(vel_x**2 + vel_y**2)

            m.scale.x = max(speed * 0.3, 0.1)  # Arrow length
            m.scale.y = 0.05  # Arrow width
            m.scale.z = 0.05  # Arrow height

            # Color based on speed (green -> yellow -> red)
            max_speed = 8.0
            ratio = min(speed / max_speed, 1.0)
            m.color.r = ratio
            m.color.g = 1.0 - ratio * 0.5
            m.color.b = 0.0
            m.color.a = 0.8

            arr.markers.append(m)

        self.velocity_marker_pub.publish(arr)

    def _publish_safety_zones(self, ts):
        """Publish safety zone circles for each agent."""
        arr = MarkerArray()
        collision_threshold = 0.21

        for i in range(self.num_agents):
            ns = f"{self.racecar_namespace}{i+1}"

            m = Marker()
            m.header.stamp = ts
            m.header.frame_id = f"{ns}/base_link"
            m.ns = "safety_zones"
            m.id = i
            m.type = Marker.CYLINDER
            m.action = Marker.ADD

            m.pose.position.z = 0.01  # Just above ground
            m.scale.x = collision_threshold * 4  # Diameter
            m.scale.y = collision_threshold * 4
            m.scale.z = 0.02  # Very thin cylinder

            # Get minimum scan distance for this agent
            min_scan = np.min(self.obs["scans"][i])

            # Color: red if close to collision, green otherwise
            if min_scan < collision_threshold * 3:
                # Danger zone - red
                m.color.r = 1.0
                m.color.g = 0.2
                m.color.b = 0.0
            elif min_scan < collision_threshold * 5:
                # Warning zone - yellow
                m.color.r = 1.0
                m.color.g = 1.0
                m.color.b = 0.0
            else:
                # Safe - green
                m.color.r = 0.0
                m.color.g = 1.0
                m.color.b = 0.0
            m.color.a = 0.3

            arr.markers.append(m)

        self.safety_zone_pub.publish(arr)

    def _publish_all_transforms_batched(self, ts):
        """Batch publish all transforms in a single sendTransform call.

        Optimized: Collects all transforms (base_link, laser, wheels) and sends
        them in one call instead of 4*num_agents separate calls.
        For 20 agents, this reduces from 80 sendTransform calls to 1.
        """
        transforms = []

        for i in range(self.num_agents):
            racecar_namespace = self.racecar_namespace + str(i + 1)

            # Base link transform (map -> base_link)
            base_ts = TransformStamped()
            base_ts.header.stamp = ts
            base_ts.header.frame_id = "map"
            base_ts.child_frame_id = racecar_namespace + "/base_link"
            base_ts.transform.translation.x = self.obs["poses_x"][i]
            base_ts.transform.translation.y = self.obs["poses_y"][i]
            base_ts.transform.translation.z = 0.0
            base_quat = euler.euler2quat(
                0.0, 0.0, self.obs["poses_theta"][i], axes="sxyz")
            base_ts.transform.rotation.x = base_quat[1]
            base_ts.transform.rotation.y = base_quat[2]
            base_ts.transform.rotation.z = base_quat[3]
            base_ts.transform.rotation.w = base_quat[0]
            transforms.append(base_ts)

            # Laser transform (base_link -> laser)
            laser_ts = TransformStamped()
            laser_ts.header.stamp = ts
            laser_ts.header.frame_id = racecar_namespace + "/base_link"
            laser_ts.child_frame_id = racecar_namespace + "/laser"
            laser_ts.transform.translation.x = self.scan_distance_to_base_link
            laser_ts.transform.rotation.w = 1.0
            transforms.append(laser_ts)

            # Wheel transforms (hinge -> wheel)
            steering_angle = self.drive_msgs[i][0]
            wheel_quat = euler.euler2quat(0.0, 0.0, steering_angle, axes="sxyz")

            # Front left wheel
            left_wheel_ts = TransformStamped()
            left_wheel_ts.header.stamp = ts
            left_wheel_ts.header.frame_id = racecar_namespace + "/front_left_hinge"
            left_wheel_ts.child_frame_id = racecar_namespace + "/front_left_wheel"
            left_wheel_ts.transform.rotation.x = wheel_quat[1]
            left_wheel_ts.transform.rotation.y = wheel_quat[2]
            left_wheel_ts.transform.rotation.z = wheel_quat[3]
            left_wheel_ts.transform.rotation.w = wheel_quat[0]
            transforms.append(left_wheel_ts)

            # Front right wheel
            right_wheel_ts = TransformStamped()
            right_wheel_ts.header.stamp = ts
            right_wheel_ts.header.frame_id = racecar_namespace + "/front_right_hinge"
            right_wheel_ts.child_frame_id = racecar_namespace + "/front_right_wheel"
            right_wheel_ts.transform.rotation.x = wheel_quat[1]
            right_wheel_ts.transform.rotation.y = wheel_quat[2]
            right_wheel_ts.transform.rotation.z = wheel_quat[3]
            right_wheel_ts.transform.rotation.w = wheel_quat[0]
            transforms.append(right_wheel_ts)

        # Single batched broadcast
        self.br.sendTransform(transforms)

    # Keep old methods for backward compatibility but mark as deprecated
    def _publish_transforms(self, ts):
        """Deprecated: Use _publish_all_transforms_batched instead."""
        self._publish_all_transforms_batched(ts)

    def _publish_wheel_transforms(self, ts):
        """Deprecated: Handled by _publish_all_transforms_batched."""
        pass

    def _publish_laser_transforms(self, ts):
        """Deprecated: Handled by _publish_all_transforms_batched."""
        pass

    def publish_lap_times(self):
        if not self.simulation_paused and self.simulation_running:
            lap_times_str = ""
            lap_times_msg = String()
            for agent_idx, tracker in enumerate(self.lap_time_trackers):
                elapsed_time = tracker.get_elapsed_time()
                lap_times_str += f"Agent {agent_idx + 1}: Current Lap: {elapsed_time:.4f} Best Lap: {self.best_lap_times[agent_idx]:.4f}{' Disqualified' if self.agent_disqualified[agent_idx] else ''}\n"
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
