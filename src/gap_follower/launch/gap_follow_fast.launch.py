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
# INTENTIONALLY FAILING LAUNCH FILE FOR ROLLBACK DEMO
# This launch file references a non-existent executable to trigger a failure.
#

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate a launch description that will fail intentionally."""
    ld = LaunchDescription()

    # This node references a non-existent executable to cause launch failure
    ld.add_action(
        Node(
            package="gap_follower",
            executable="gap_follower_fast_DOES_NOT_EXIST",  # Intentionally wrong
            namespace="",
            name="gap_follower_fast_failing",
        )
    )
    return ld
