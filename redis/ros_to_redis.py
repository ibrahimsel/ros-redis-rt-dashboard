import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import redis   # ← sync client, not redis.asyncio


class SpeedToRedis(Node):
    """Subscribes /speed (Float64) and publishes the value to Redis channel 'vehicle_data'."""

    def __init__(self) -> None:
        super().__init__("speed_to_redis")
        self.redis = redis.Redis(
            host=os.getenv("REDIS_HOST", "redis"),
            port=int(os.getenv("REDIS_PORT", 6379)),
            decode_responses=True,   # strings instead of raw bytes
        )
        self.create_subscription(Float64, "/speed", self._on_speed, 10)
        self.get_logger().info("Bridging /speed → Redis channel 'vehicle_data'")

    def _on_speed(self, msg: Float64) -> None:
        # one line, gets the job done
        self.redis.publish("vehicle_data", msg.data)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SpeedToRedis()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
