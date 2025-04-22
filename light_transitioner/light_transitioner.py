import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray, Int32
from geometry_msgs.msg import PoseStamped

class LightTransitioner(Node):
    def __init__(self):
        super().__init__('light_transitioner')

        # Light publisher
        self.light_publisher = self.create_publisher(UInt8MultiArray, '/set_lights', 10)

        # Subscribers
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.create_subscription(Int32, '/check_goal_proximity', self.proximity_callback, 10)

        self.publish_light([0, 2, 255, 255, 255)
        self.get_logger().info("Published initial light.")

    def publish_light(self, data):
        msg = UInt8MultiArray()
        msg.data = data
        self.light_publisher.publish(msg)
        self.get_logger().info(f"Published light data: {msg.data}")

    def goal_callback(self, msg):
        self.publish_light([1, 2, 50, 255, 0])  
        self.get_logger().info("Goal received → set lights to green.")

    def proximity_callback(self, msg):
        if msg.data == 1:
            self.publish_light([0, 2, 255, 255, 255])
            self.get_logger().info("Close to goal → set lights to white.")

def main(args=None):
    rclpy.init(args=args)
    node = LightTransitioner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
