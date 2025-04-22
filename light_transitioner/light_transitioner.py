import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray
from geometry_msgs.msg import PoseStamped

class LightTransitioner(Node):
    def __init__(self):
        super().__init__('light_transitioner')

        # Publisher to /set_lights
        self.publisher_ = self.create_publisher(UInt8MultiArray, '/set_lights', 10)

        # Initial message (set to white)
        init_msg = UInt8MultiArray()
        init_msg.data = [0, 2, 255, 0, 0]
        self.publisher_.publish(init_msg)
        self.get_logger().info(f"Initial light message published: {init_msg.data}")

        # Subscriber to /goal_pose
        self.subscription = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )
        self.subscription  

    # changes to green mode 1 when new goal is received (assumes moving)
    def goal_callback(self, msg):
        new_msg = UInt8MultiArray()
        new_msg.data = [1, 2, 50, 255, 0]
        self.publisher_.publish(new_msg)
        self.get_logger().info(f"Goal received. Light message published: {new_msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = LightTransitioner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
