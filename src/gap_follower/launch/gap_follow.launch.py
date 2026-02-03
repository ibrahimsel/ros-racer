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
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()

    gap_follower_config = os.path.join(
        get_package_share_directory('gap_follower'),
        'config',
        'gap_follower.yaml'
    )


    ld.add_action(
        Node(
            package="gap_follower",
            executable="gap_follower",
            namespace="",
            parameters=[gap_follower_config],
        )
    )
    return ld
