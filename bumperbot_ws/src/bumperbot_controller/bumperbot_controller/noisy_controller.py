#!/usr/bin/env python3 
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import numpy as np
from sensor_msgs.msg import JointState
from rclpy.time import Time
from rclpy.constants import S_TO_NS # Seconds to Nanoseconds conversion factor
import math
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler , quaternion_multiply , quaternion_inverse
from tf2_ros import TransformBroadcaster 

class NoisyController(Node): 
    def __init__(self):
        super().__init__('simple_controller')

        self.declare_parameter("wheel_radius", 0.033)
        self.declare_parameter("wheel_separation", 0.17)

        self.wheel_radius_= self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.wheel_separation_ = self.get_parameter("wheel_separation").get_parameter_value().double_value

        self.get_logger().info(f"wheel_radius: {self.wheel_radius_}")
        self.get_logger().info(f"wheel_separation: {self.wheel_separation_}")

        self.left_wheel_prev_pos_ =  0.0
        self.right_wheel_prev_pos_ = 0.0
        self.prev_time_ = self.get_clock().now() # Initialize previous time with current time when node starts

        self.x_ = 0.0
        self.y_ = 0.0
        self.theta_ = 0.0

        self.joint_sub_ = self.create_subscription(
            JointState , "joint_states" , self.jointCallback , 10
        )

        self.odom_pub_ = self.create_publisher(
            Odometry , "bumperbot_controller/odom_noisy" , 10
        )

    
        #here we prepare the odom message by setting the frame id and child frame id once since they don't change
        self.odom_msg_ = Odometry()
        self.odom_msg_.header.frame_id = "odom"
        self.odom_msg_.child_frame_id = "base_footprint_ekf"
        #Initialize position and orientation to zero
        self.odom_msg_.pose.pose.position.x = 0.0
        self.odom_msg_.pose.pose.position.y = 0.0
        self.odom_msg_.pose.pose.position.z = 0.0
        self.odom_msg_.pose.pose.orientation.x = 0.0
        self.odom_msg_.pose.pose.orientation.y = 0.0
        self.odom_msg_.pose.pose.orientation.z = 0.0
        self.odom_msg_.pose.pose.orientation.w = 1.0

        # here we set up the transform broadcaster to publish the transform from odom to base_footprint
        self.broadcaster_ = TransformBroadcaster(self)
        self.transform_stamped_ = TransformStamped()
        self.transform_stamped_.header.frame_id = "odom"
        self.transform_stamped_.child_frame_id = "base_footprint_noisy"


    def jointCallback(self , msg: JointState):

        wheel_encoder_left = msg.position[1] + np.random.normal(0,0.005) # left wheel with noise
        wheel_encoder_right = msg.position[0] + np.random.normal(0,0.005) # right wheel with noise

        dp_left = wheel_encoder_left - self.left_wheel_prev_pos_  # left wheel
        dp_right = wheel_encoder_right - self.right_wheel_prev_pos_ # right
        dt = Time.from_msg( msg.header.stamp) - self.prev_time_ # Time difference The Time Is used to convert the ROS time to a rclpy Time object

        #Update left and right wheel previous positions and previous time
        self.left_wheel_prev_pos_ = msg.position[1]
        self.right_wheel_prev_pos_ = msg.position[0]
        self.prev_time_ = Time.from_msg( msg.header.stamp)

        rotational_speed_left = dp_left / (dt.nanoseconds / S_TO_NS)  # rad/s
        rotational_speed_right = dp_right / (dt.nanoseconds / S_TO_NS)  # rad/s

        linear_speed =  self.wheel_radius_ * (rotational_speed_right + rotational_speed_left) / 2
        angular_speed = self.wheel_radius_ * (rotational_speed_right - rotational_speed_left) / self.wheel_separation_

        pos = self.wheel_radius_ * (dp_right + dp_left) / 2 # linear position
        orientation = self.wheel_radius_ * (dp_right - dp_left) / self.wheel_separation_ # angular position
        self.theta_ += orientation
        self.x_ += pos * math.cos(self.theta_) # Update x based on current theta
        self.y_ += pos * math.sin(self.theta_) # Update y based on current theta

        quaternion = quaternion_from_euler(0, 0, self.theta_)  #here we convert the yaw (theta) to a quaternion
        self.odom_msg_.pose.pose.orientation.x = quaternion[0]
        self.odom_msg_.pose.pose.orientation.y = quaternion[1]
        self.odom_msg_.pose.pose.orientation.z = quaternion[2]
        self.odom_msg_.pose.pose.orientation.w = quaternion[3]
        self.odom_msg_.header.stamp = self.get_clock().now().to_msg() #set the header stamp to current time
        self.odom_msg_.pose.pose.position.x = self.x_  #here we set the position
        self.odom_msg_.pose.pose.position.y = self.y_
        self.odom_msg_.pose.pose.position.z = 0.0

        self.odom_msg_.twist.twist.linear.x = linear_speed #here we set the linear velocity
        self.odom_msg_.twist.twist.angular.z = angular_speed #here we set the angular velocity

        #now we prepare the transform to be broadcasted
        self.transform_stamped_.transform.translation.x = self.x_
        self.transform_stamped_.transform.translation.y = self.y_
        self.transform_stamped_.transform.translation.z = 0.0
        self.transform_stamped_.transform.rotation.x = quaternion[0]
        self.transform_stamped_.transform.rotation.y = quaternion[1]
        self.transform_stamped_.transform.rotation.z = quaternion[2]
        self.transform_stamped_.transform.rotation.w = quaternion[3]
        self.transform_stamped_.header.stamp = self.get_clock().now().to_msg()
        self.broadcaster_.sendTransform(self.transform_stamped_) #broadcast the transform from odom to base_footprint


        self.odom_pub_.publish(self.odom_msg_) #publish the odom message



def main(args=None):
    rclpy.init(args=args)
    node = NoisyController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()