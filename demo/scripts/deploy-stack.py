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
import sys
import argparse
import time


class StackDeployer(Node):
    def __init__(self, stack_file, vehicle=None, method='start'):
        super().__init__('stack_deployer')

        # Determine topic based on vehicle
        if vehicle:
            topic = f'/{vehicle}_muto/stack'
        else:
            # Broadcast to all vehicles by publishing to each known vehicle
            topic = '/muto/stack'

        self.topic = topic
        self.publisher = self.create_publisher(MutoAction, topic, 10)

        # Load stack definition
        with open(stack_file) as f:
            self.stack = json.load(f)

        self.method = method
        self.published = False

        self.get_logger().info(f"Stack deployer initialized")
        self.get_logger().info(f"  Stack: {self.stack.get('metadata', {}).get('name', 'unknown')}")
        self.get_logger().info(f"  Version: {self.stack.get('metadata', {}).get('version', 'unknown')}")
        self.get_logger().info(f"  Topic: {topic}")
        self.get_logger().info(f"  Method: {method}")

        # Create timer to publish (allow time for discovery)
        self.create_timer(1.0, self.publish_stack)

    def publish_stack(self):
        if self.published:
            return

        msg = MutoAction()
        msg.method = self.method
        msg.payload = json.dumps(self.stack)

        self.publisher.publish(msg)
        self.published = True

        self.get_logger().info(f"Published stack to {self.topic}")
        self.get_logger().info(f"Payload preview: {msg.payload[:100]}...")

        # Schedule shutdown
        self.create_timer(0.5, lambda: self.shutdown())

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
