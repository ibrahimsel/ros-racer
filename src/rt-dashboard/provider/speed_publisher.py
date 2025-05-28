import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import random
from ackermann_msgs.msg import AckermannDriveStamped
class SpeedPublisher(Node):
    """
    A ROS2 node that publishes random float64 values to the /speed topic at 100Hz.

    Attributes:
        publisher_ (Publisher): Publishes Float64 messages.
        timer_ (Timer): Timer to trigger publishing at 100Hz.
    """

    def __init__(self) -> None:
        super().__init__('speed_publisher')
        self.publisher_ = self.create_publisher(Float64, '/speed', 10)
        self.timer_ = self.create_timer(0.01, self.publish_random_speed)  # 100Hz
        self.get_logger().info("SpeedPublisher is vibin' at 100Hz")

    def publish_random_speed(self) -> None:
        """
        Publishes a random Float64 message to the /speed topic.
        """
        msg = Float64()
        msg.data = random.uniform(0.0, 100.0)
        self.publisher_.publish(msg)
        self.get_logger().info(f"Ackermann: {AckermannDriveStamped()}")
        self.get_logger().debug(f"Published speed: {msg.data:.2f}")

def main(args=None) -> None:
    """
    Initializes the ROS2 system and spins the SpeedPublisher node.
    """
    rclpy.init(args=args)
    node = SpeedPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Ctrl+C hit — shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
