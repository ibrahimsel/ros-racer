#!/usr/bin/env python3
"""
Deploy a stack to Muto by publishing to the stack topic.

Usage:
    # Deploy to all vehicles (broadcast)
    python3 deploy-stack.py stacks/gap_follower_conservative.json

    # Deploy to specific vehicle
    python3 deploy-stack.py stacks/gap_follower_balanced.json --vehicle racecar1

    # Stop a stack
    python3 deploy-stack.py stacks/gap_follower_balanced.json --method kill
"""
import rclpy
from rclpy.node import Node
from muto_msgs.msg import MutoAction
import json
import os
import sys
import argparse
import time


class StackDeployer(Node):
    # Dynamic fleet based on NUM_AGENTS environment variable
    _num_agents = int(os.environ.get('NUM_AGENTS', 3))
    FLEET_VEHICLES = [f'racecar{i}' for i in range(1, _num_agents + 1)]

    def __init__(self, stack_file, vehicle=None, method='start'):
        super().__init__('stack_deployer')

        # Determine topics based on vehicle
        if vehicle:
            # Single vehicle - publish to its specific topic
            self.topics = [f'/{vehicle}_muto/stack']
        else:
            # Broadcast to all vehicles in the fleet
            self.topics = [f'/{v}_muto/stack' for v in self.FLEET_VEHICLES]

        # Create a publisher for each topic
        self.stack_publishers = []
        for topic in self.topics:
            pub = self.create_publisher(MutoAction, topic, 10)
            self.stack_publishers.append((topic, pub))

        # Load stack definition
        with open(stack_file) as f:
            self.stack = json.load(f)

        self.method = method
        self.published = False

        self.get_logger().info(f"Stack deployer initialized")
        self.get_logger().info(f"  Stack: {self.stack.get('metadata', {}).get('name', 'unknown')}")
        self.get_logger().info(f"  Version: {self.stack.get('metadata', {}).get('version', 'unknown')}")
        self.get_logger().info(f"  Topics: {self.topics}")
        self.get_logger().info(f"  Method: {method}")

        # Create timer to publish (allow time for DDS discovery)
        # With many agents on CycloneDDS, discovery can take 3-5 seconds
        discovery_wait = float(os.environ.get('DEPLOY_DISCOVERY_WAIT', 3.0))
        self.get_logger().info(f"  Discovery wait: {discovery_wait}s")
        self.create_timer(discovery_wait, self.publish_stack)

    def publish_stack(self):
        if self.published:
            return

        msg = MutoAction()
        msg.method = self.method
        msg.payload = json.dumps(self.stack)

        # Publish to all topics
        for topic, publisher in self.stack_publishers:
            publisher.publish(msg)
            self.get_logger().info(f"Published stack to {topic}")

        self.published = True
        self.get_logger().info(f"Payload preview: {msg.payload[:100]}...")

        # Wait for reliable delivery before shutdown
        self.create_timer(1.0, lambda: self.shutdown())

    def shutdown(self):
        raise SystemExit(0)


def main():
    parser = argparse.ArgumentParser(
        description='Deploy Muto stack via ROS topic',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Deploy conservative variant to all vehicles
  %(prog)s ../stacks/gap_follower_conservative.json

  # Deploy to specific vehicle (racecar1, racecar2, or racecar3)
  %(prog)s ../stacks/gap_follower_balanced.json --vehicle racecar1

  # Stop current stack on a vehicle
  %(prog)s ../stacks/gap_follower_balanced.json --vehicle racecar1 --method kill
"""
    )
    parser.add_argument('stack_file', help='Path to stack JSON file')
    parser.add_argument(
        '--vehicle', '-v',
        help='Target vehicle name (racecar1, racecar2, racecar3). If omitted, publishes to /muto/stack'
    )
    parser.add_argument(
        '--method', '-m',
        default='start',
        choices=['start', 'apply', 'kill'],
        help='Action method (default: start)'
    )

    args = parser.parse_args()

    # Verify stack file exists
    try:
        with open(args.stack_file) as f:
            json.load(f)
    except FileNotFoundError:
        print(f"Error: Stack file not found: {args.stack_file}")
        sys.exit(1)
    except json.JSONDecodeError as e:
        print(f"Error: Invalid JSON in stack file: {e}")
        sys.exit(1)

    rclpy.init()
    node = StackDeployer(args.stack_file, args.vehicle, args.method)

    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
