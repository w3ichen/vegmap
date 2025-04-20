# robot_pose_tf_publisher.py
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage

class RobotPoseTFPublisher(Node):
    def __init__(self):
        super().__init__('robot_pose_tf_publisher')
        self.br = TransformBroadcaster(self)

        self.subscription = self.create_subscription(
            TFMessage,
            '/model/a200_0000/robot/pose',
            self.handle_pose,
            10
        )

    def handle_pose(self, msg):
        for transform in msg.transforms:
            if transform.header.frame_id == 'resistance_zones' and transform.child_frame_id == 'a200_0000/robot':
                t = TransformStamped()
                t.header.stamp = self.get_clock().now().to_msg()
                t.header.frame_id = 'map'
                t.child_frame_id = 'a200_0000/robot'
                t.transform = transform.transform
                self.br.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = RobotPoseTFPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
