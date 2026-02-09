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
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def launch_setup(context):
    sim_config = os.path.join(
        get_package_share_directory("f1tenth_gym_ros"), "config", "sim.yaml"
    )

    config_dict = yaml.safe_load(open(sim_config, "r"))
    map_path = config_dict["gym_bridge"]["ros__parameters"]["map_path"]
    default_map_path = os.path.join(
        get_package_share_directory("f1tenth_gym_ros"), "maps", "example_map"
    )

    # Resolve num_agents from launch argument (which defaults to YAML value)
    num_agents = int(LaunchConfiguration("num_agents").perform(context))

    print(f"Starting with {num_agents} agents.")

    # Pass num_agent as parameter override so it takes precedence over YAML
    gym_bridge_node = Node(
        package="f1tenth_gym_ros",
        executable="gym_bridge",
        name="gym_bridge",
        parameters=[sim_config, {"num_agent": num_agents}],
    )

    # Color palette for agents (cycles if more agents than colors)
    colors = [
        "red", "green", "blue", "yellow", "magenta",
        "cyan", "orange", "purple", "pink", "brown",
    ]
    robot_state_publishers = []

    for i in range(num_agents):
        vehicle_name = f"racecar{i + 1}"
        color = colors[i % len(colors)]
        robot_state_publishers.append(
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name=f"{vehicle_name}_robot_description",
                parameters=[
                    {
                        "robot_description": Command(
                            [
                                "xacro ",
                                os.path.join(
                                    get_package_share_directory("f1tenth_gym_ros"),
                                    "launch",
                                    "ego_racecar.xacro",
                                ),
                                f" car:={vehicle_name}",
                                f" car_color:={color}",
                            ]
                        )
                    }
                ],
                remappings=[
                    ("/robot_description", f"/{vehicle_name}/robot_description")
                ],
            )
        )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        arguments=[
            "-d",
            os.path.join(
                get_package_share_directory("f1tenth_gym_ros"),
                "launch",
                "gym_bridge.rviz",
            ),
        ],
    )

    map_server_node = Node(
        package="nav2_map_server",
        executable="map_server",
        parameters=[
            {
                "yaml_filename": map_path + ".yaml"
                if map_path
                else default_map_path + ".yaml"
            },
            {"topic": "map"},
            {"frame_id": "map"},
            {"output": "screen"},
            {"use_sim_time": True},
        ],
    )

    nav_lifecycle_node = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"autostart": True},
            {"node_names": ["map_server"]},
        ],
    )

    return [
        rviz_node,
        gym_bridge_node,
        nav_lifecycle_node,
        map_server_node,
        *robot_state_publishers,
    ]


def generate_launch_description():
    sim_config = os.path.join(
        get_package_share_directory("f1tenth_gym_ros"), "config", "sim.yaml"
    )
    config_dict = yaml.safe_load(open(sim_config, "r"))
    yaml_num_agents = config_dict["gym_bridge"]["ros__parameters"]["num_agent"]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "num_agents",
                default_value=str(yaml_num_agents),
                description="Number of agents to spawn (overrides sim.yaml value)",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
