#!/usr/bin/env python3

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
        self.cost = 0.0            # traverse cost
        self.actual_speed = 0.0    # Actual speed of the robot
        

        """ Create subscribers """
        # Commanded velocity subscription
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'a200_0000/cmd_vel',
            self.cmd_vel_callback,
            10  
        )

        # Ground truth pose subscription
        self.ground_truth_sub = self.create_subscription(
            TFMessage,
            '/model/a200_0000/robot/pose',
            self.ground_truth_callback,
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
        for transform in msg.transforms:
            if (transform.header.frame_id == 'resistance_zones' and   # in world frame
                transform.child_frame_id == 'a200_0000/robot'):       # get robot pose

                # extract current position
                current_position = [
                    transform.transform.translation.x,
                    transform.transform.translation.y,
                    transform.transform.translation.z
                ]

                # extract current time
                current_time = transform.header.stamp.sec + transform.header.stamp.nanosec / 1e9

                # calculate actual speed
                if self.prev_position is not None and self.prev_time is not None:
                    # difference in time
                    dt = current_time - self.prev_time

                    if dt > 0:
                        dx = current_position[0] - self.prev_position[0]
                        dy = current_position[1] - self.prev_position[1]

                        distance = math.sqrt(dx**2 + dy**2)

                        self.actual_speed = distance / dt

                        if self.commanded_speed > 0:
                            temp_cost = 1.0 - (self.actual_speed / self.commanded_speed)
                            self.cost = max(0.0, temp_cost)
                            self.get_logger().info(f"current cost: {temp_cost:.2f}")
                            
                # update previous position and time
                self.prev_position = current_position
                self.prev_time = current_time
                break

    def cmd_vel_callback(self, msg):
        """ Process commanded velocity data """
        # extract commanded linear speed
        self.commanded_speed = msg.linear.x

    def publish_cost(self):
        """ Publish current grid cost """
        if hasattr(self, 'actual_speed') and self.commanded_speed > 0: 
            # calculate cost
            # cost = 1.0 - (self.actual_speed / self.commanded_speed)
            # cost = max(0.0, cost)
            cost_msg = Float32()
            cost_msg.data = float(self.cost)
            self.cost_pub.publish(cost_msg)
        
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