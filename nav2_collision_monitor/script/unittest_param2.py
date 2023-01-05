import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters, SetParameters
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue

class TestGetGlobalParam(Node):
    def callback_set_param(self, future):
        try:
            result = future.result()
        except Exception as e:
            self.get_logger().warn("service call failed %r" % (e,))
        else:
            param = result.results
            if (param):
                self.get_logger().info("Set parameter successfully")
            else:
                 self.get_logger().warn("Setting parameter failed")

    def __init__(self):
        super().__init__("test_get_global_param")
        self.client = self.create_client(SetParameters, '/collision_monitor/set_parameters')

    def setParam(self, param_name='FootprintApproach.activate', activation=True):
        request = SetParameters.Request()
        
        new_param_value = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=activation)
        request.parameters = [Parameter(name=param_name, value=new_param_value)]
        
        self.client.wait_for_service()

        future = self.client.call_async(request)
        future.add_done_callback(self.callback_set_param)

def main(args=None):
    rclpy.init(args=args)
    node = TestGetGlobalParam()
    node.setParam('FootprintApproach.activate', True)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()