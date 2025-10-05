import rclpy
from rclpy.node import Node
from bumperbot_msgs.srv import AddTwoInts
import sys

class SimpleServiceClient(Node):
    def __init__(self , a , b):
        super().__init__('simple_service_client')

        self.client_ = self.create_client(
            AddTwoInts, 'add_two_ints' 
        )

        while not self.client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.request = AddTwoInts.Request()
        self.request.a = a
        self.request.b = b

        #this func returns a future object that bee returns when the service executes his function which is add_two_ints_callback
        self.future = self.client_.call_async(self.request)
        self.future.add_done_callback(self.responseCallback)        

    def responseCallback(self, future):
        self.get_logger().info(f"Result of add_two_ints: {future.result().sum}")      


def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) != 3:
        print("Usage: simple_service_client.py <int a> <int b>")
        return -1 
    
    a = int(sys.argv[1])
    b = int(sys.argv[2])

    node = SimpleServiceClient(a , b)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()                