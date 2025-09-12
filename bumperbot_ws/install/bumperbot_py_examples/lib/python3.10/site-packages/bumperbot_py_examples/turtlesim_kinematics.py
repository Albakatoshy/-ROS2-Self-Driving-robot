import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
import math

class simpleTurtlesimKinematics(Node):
    def __init__(self):
        super().__init__('simple_turtlesim_kinematics')

        self.turtle1_pose_sub_ = self.create_subscription(
            Pose , '/turtle1/pose' , self.turtle1_pose_callback , 10
        )

        self.turle2_pose_sub_ = self.create_subscription(
            Pose , '/turtle2/pose' , self.turtle2_pose_callback , 10
        )

        self.last_turtle1_pose_ = Pose()
        self.last_turtle2_pose_ = Pose()

    def turtle1_pose_callback(self, msg):
        self.last_turtle1_pose_ = msg

    def turtle2_pose_callback(self, msg):
        self.last_turtle2_pose_ = msg
        # Calculate distance between turtle1 and turtle2
        Tx = self.last_turtle2_pose_.x - self.last_turtle1_pose_.x
        Ty = self.last_turtle2_pose_.y - self.last_turtle1_pose_.y

        theta_rad = self.last_turtle2_pose_.theta - self.last_turtle1_pose_.theta
        theta_deg = theta_rad * (180.0 / 3.141592653589793)

        # Log the translation vector
        self.get_logger().info(f"""\n
        traslation vector from turtle1 to turtle2:\n
                              TX: {Tx}\n
                              TY: {Ty}\n
                              Rotation (degrees): {theta_deg}
                              Rotation (rad)    : {theta_rad}
--------------------------------------------------------------
                            rotation matrix from turtle1 to turtle2:
                            [r11 r12]  [{math.cos(theta_rad)}  {-math.sin(theta_rad)}] \n
                            [r21 r22]  [{math.sin(theta_rad)}   {math.cos(theta_rad)}] \n
--------------------------------------------------------------
                              """
                               )
        


def main(args=None):
    rclpy.init(args=args)
    node = simpleTurtlesimKinematics()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()