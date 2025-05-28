import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import redis


class SpeedToRedis(Node):
    def __init__(self) -> None:
        super().__init__("speed_to_redis")
        self.redis = redis.Redis(host="localhost", port=6379)
        self.sub = self.create_subscription(Float64, "/speed", self.callback, 10)

    def callback(self, msg: Float64) -> None:
        self.redis.publish("vehicle_data", str(msg.data))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SpeedToRedis()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
