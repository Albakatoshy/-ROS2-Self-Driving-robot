#!/usr/bin/env python3 
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray  # Changed from Float32MultiArray
from geometry_msgs.msg import TwistStamped
import numpy as np


class SimpleController(Node): 
    def __init__(self):
        super().__init__('simple_controller')

        self.declare_parameter("wheel_radius", 0.033)
        self.declare_parameter("wheel_separation", 0.17)

        self.wheel_radius_= self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.wheel_separation_ = self.get_parameter("wheel_separation").get_parameter_value().double_value

        self.get_logger().info(f"wheel_radius: {self.wheel_radius_}")
        self.get_logger().info(f"wheel_separation: {self.wheel_separation_}")

        self.wheel_cmd_pub_ = self.create_publisher(
            Float64MultiArray , "simple_velocity_controller/commands" , 10  # Changed to Float64MultiArray
        )

        self.vel_sub_ = self.create_subscription(
            TwistStamped , "bumperbot_controller/cmd_vel" , self.velCallback , 10
        )

        self.speed_conversion_ = np.array([[self.wheel_radius_/2 , self.wheel_radius_/2] ,
                                          [self.wheel_radius_/self.wheel_separation_ , -self.wheel_radius_/self.wheel_separation_]])  # Fixed sign
        
        self.get_logger().info(f"the speed_conversion matrix : {self.speed_conversion_}")

    def velCallback(self , msg: TwistStamped):
        robot_speed = np.array([[msg.twist.linear.x] ,
                                [msg.twist.angular.z]]) 
        wheel_speed = np.matmul(np.linalg.inv(self.speed_conversion_) , robot_speed)

        wheel_speed_msg = Float64MultiArray()  # Changed to Float64MultiArray
        wheel_speed_msg.data = [wheel_speed[1 , 0] , wheel_speed[0 , 0]] # left , right
        self.wheel_cmd_pub_.publish(wheel_speed_msg)
        self.get_logger().info(f"publishing wheel speeds (rad/s): left: {wheel_speed[1 , 0]} , right: {wheel_speed[0 , 0]}")

def main(args=None):
    rclpy.init(args=args)
    node = SimpleController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()