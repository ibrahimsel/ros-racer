import os
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
import redis


class SpeedToRedis(Node):
    """
    Subscribes /speed (Float64) and publishes the value to Redis channel.
    """

    def __init__(self) -> None:
        super().__init__("speed_to_redis")
        self.redis = redis.Redis(
            host=os.getenv("REDIS_HOST", "redis"),
            port=int(os.getenv("REDIS_PORT", 6379)),
            decode_responses=True,  # strings instead of raw bytes
        )
        self.create_subscription(
            AckermannDriveStamped, "/racecar1/drive", self._on_speed, 10
        )
        self.get_logger().info(
            "Bridging /racecar1/drive → Redis channel 'vehicle_data'"
        )

    def _on_speed(self, msg: AckermannDriveStamped) -> None:
        # one line, gets the job done
        self.redis.publish("vehicle_data", msg.drive.speed)
        # self.get_logger().info(f"Received speed: {msg.drive.speed:.2f} m/s")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SpeedToRedis()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
