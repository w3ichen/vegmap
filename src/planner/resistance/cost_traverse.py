import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from tf2_msgs.msg import TFMessage
import math
import numpy as np
from std_msgs.msg import Float32


class cost_calculator(Node):
    def __init__(self, name):
        """ Calculate cost of traverse based on input speed and actual speed """
        super().__init__(name)

        self.prev_position = None  # [x, y, z]
        self.prev_time = None      # seconds
        self.commanded_speed = 0.0 # Las commanded linear speed

        """ Create subscribers """
        # Ground truth pose subscription
        self.ground_truth_sub = self.create_subscription(
            TFMessage,
            '/model/a200_0000/robot/pose',
            self.ground_truth_callback,
            10  
        )

        # Commanded velocity subscription
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'a200_0000/cmd_vel',
            self.cmd_vel_callback,
            10  
        )

        """ Create publishers """
        # Cost publisher
        self.cost_pub = self.create_publisher(
            Float32,
            '/cost_traverse',
            10
        )


        # Timer for regulating cost publication
        self.timer = self.create_timer(0.1, self.publish_cost)
        self.get_logger().info("Cost calculator node initialized.")

    def ground_truth_callback(self, msg):
        """ Process ground truth position data """
        for transform in msg.tnrans



    """ ADD PROCESSING OF GROUND TRUTH POSE """

    """ ADD COST CALCULATION FROM GROUND TRUTH POSE """
        

def main (args=None):
    rclpy.init(args=args)
    node = cost_calculator("cost_calculator")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()