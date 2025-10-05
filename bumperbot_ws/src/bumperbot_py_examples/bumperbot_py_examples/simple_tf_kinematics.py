import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster 
from tf2_ros import TransformBroadcaster , TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped
from tf_transformations import quaternion_from_euler , quaternion_multiply , quaternion_inverse
from bumperbot_msgs.srv import GetTransform




class SimpleTFKinematics(Node):
    def __init__(self):
        super().__init__('simple_tf_kinematics')

        # Create a static transform broadcaster and a TransformStamped message Then Publish the static transform once when the node starts
        self.static_tf_broadcaster_ = StaticTransformBroadcaster(self)

        # Create a dynamic transform broadcaster to publish transforms that may change over time
        self.dynamic_tf_broadcaster_ = TransformBroadcaster(self)


        self.static_transform_stamped_ = TransformStamped()
        self.dynamic_transform_stamped_ = TransformStamped()

        self.x_increment_ = 0.05  # Increment in x direction (1 cm per update)
        self.last_x = 0.0

        self.rotationts_counter = 0
        self.last_orientation = quaternion_from_euler(0,0,0)
        self.orientation_increment_ = quaternion_from_euler(0 , 0 , 0.05)

        self.tf_buffer_  = Buffer()
        self.tf_listener_ = TransformListener(self.tf_buffer_ , self)

        self.static_transform_stamped_.header.stamp    = self.get_clock().now().to_msg() #That line stamps the transform with the current ROS time, so when you publish it, other nodes know exactly when that transform is valid.
        self.static_transform_stamped_.header.frame_id = "bumperbot_base"  #Parent frame
        self.static_transform_stamped_.child_frame_id  = "bumperbot_top"
        # Set the translation and rotation of the transform
        self.static_transform_stamped_.transform.translation.x = 0.0
        self.static_transform_stamped_.transform.translation.y = 0.0
        self.static_transform_stamped_.transform.translation.z = 0.3 #0.3 meters above the base frame (30 cm)
        self.static_transform_stamped_.transform.rotation.x    = 0.0
        self.static_transform_stamped_.transform.rotation.y    = 0.0
        self.static_transform_stamped_.transform.rotation.z    = 0.0
        self.static_transform_stamped_.transform.rotation.w    = 1.0 #No rotation, identity quaternion


        # Initialize the dynamic transform (from odom to base_link)
        self.dynamic_transform_stamped_.header.stamp    = self.get_clock().now().to_msg() #That line stamps the transform with the current ROS time, so when you publish it, other nodes know exactly when that transform is valid.
        self.dynamic_transform_stamped_.header.frame_id = "odom"
        self.dynamic_transform_stamped_.child_frame_id  = "bumperbot_base"
        # Set the translation and rotation of the transform
        self.dynamic_transform_stamped_.transform.translation.x = 0.0
        self.dynamic_transform_stamped_.transform.translation.y = 0.0
        self.dynamic_transform_stamped_.transform.translation.z = 0.0
        self.dynamic_transform_stamped_.transform.rotation.x    = 0.0
        self.dynamic_transform_stamped_.transform.rotation.y    = 0.0
        self.dynamic_transform_stamped_.transform.rotation.z    = 0.0
        self.dynamic_transform_stamped_.transform.rotation.w    = 1.0 #No rotation, identity quaternion

        # Broadcast the static transform
        self.static_tf_broadcaster_.sendTransform(self.static_transform_stamped_)
        self.get_logger().info(
            f"Publishing static transform from {self.static_transform_stamped_.header.frame_id} to {self.static_transform_stamped_.child_frame_id}") 
        
        self.timer_ = self.create_timer(0.1, self.timer_callback) #10 Hz

        self.get_transform_srv_ = self.create_service(
            GetTransform , "get_transform" , self.getTransformCallback
        )





    def timer_callback(self):
        # Update the timestamp and broadcast the dynamic transform
        self.dynamic_transform_stamped_.header.stamp    = self.get_clock().now().to_msg() #That line stamps the transform with the current ROS time, so when you publish it, other nodes know exactly when that transform is valid.
        self.dynamic_transform_stamped_.header.frame_id = "odom"
        self.dynamic_transform_stamped_.child_frame_id  = "bumperbot_base"

        self.dynamic_transform_stamped_.transform.translation.x  = self.last_x + self.x_increment_
        self.dynamic_transform_stamped_.transform.translation.y += 0.0
        self.dynamic_transform_stamped_.transform.translation.z  = 0.0
        q = quaternion_multiply(self.last_orientation , self.orientation_increment_)
        self.dynamic_transform_stamped_.transform.rotation.x    = q[0]
        self.dynamic_transform_stamped_.transform.rotation.y    = q[1]
        self.dynamic_transform_stamped_.transform.rotation.z    = q[2]
        self.dynamic_transform_stamped_.transform.rotation.w    = q[3]  #No rotation, identity quaternion


        self.dynamic_tf_broadcaster_.sendTransform(self.dynamic_transform_stamped_)
        self.get_logger().info(
            f"Publishing dynamic transform from {self.dynamic_transform_stamped_.header.frame_id} to {self.dynamic_transform_stamped_.child_frame_id} at x={self.dynamic_transform_stamped_.transform.translation.x:.2f} m")
        
        self.last_x = self.dynamic_transform_stamped_.transform.translation.x
        self.rotationts_counter +=1
        self.last_orientation = q 
        if self.rotationts_counter >= 100:
            self.orientation_increment_ = quaternion_inverse(self.orientation_increment_)
            self.rotationts_counter = 0
             


    def getTransformCallback(self , request , response):
        self.get_logger().info(f"Requested Transform between {request.frame_id} ,{request.child_frame_id} ")
        requested_transform = TransformStamped()
        try:
            requested_transform = self.tf_buffer_.lookup_transform(request.frame_id , request.child_frame_id , rclpy.time.Time() )
        except TransformException as e:
            self.get_logger().error(f"An ERROR occured while transforming {request.frame_id} , {request.child_frame_id} ")
            response.success = False
            return response
        
        response.transform = requested_transform
        response.success = True
        return response
        

def main(args=None):
    rclpy.init(args=args)
    node = SimpleTFKinematics()
    rclpy.spin(node)
    rclpy.shutdown()
    node.destroy_node()

if __name__ == '__main__':
    main()
