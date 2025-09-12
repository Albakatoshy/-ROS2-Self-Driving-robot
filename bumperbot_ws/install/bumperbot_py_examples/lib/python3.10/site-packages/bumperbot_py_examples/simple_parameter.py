import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from rclpy.parameter import Parameter

class SimpleParameter(Node):
    def __init__(self):
        super().__init__('simple_parameter')
        self.declare_parameter("simple_int_parameter", 10)
        self.declare_parameter("simple_string_parameter", "Hello World")

        self.add_on_set_parameters_callback(self.set_parameters_callback)

    def set_parameters_callback(self, params):
        result = SetParametersResult()

        for param in params:
            if param.name == "simple_int_parameter" and param.type_ == Parameter.Type.INTEGER:
                self.get_logger().info(f"Setting {param.name} to {param.value}")
                result.successful = True

            if param.name == "simple_string_parameter" and param.type_ == Parameter.Type.STRING:
                self.get_logger().info(f"Setting {param.name} to {param.value}")
                result.successful = True

        return result

def main(args=None):
    rclpy.init(args=args)
    node = SimpleParameter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

    
if __name__ == '__main__':
    main()

        