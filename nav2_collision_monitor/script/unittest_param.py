import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters

class TestGetGlobalParam(Node):
    def callback_global_param(self, future):
        try:
            result = future.result()
        except Exception as e:
            self.get_logger().warn("service call failed %r" % (e,))
        else:
            param = result.values[0]
            self.get_logger().info("Got global param: %s" % (param.bool_value,))

    def __init__(self):
        super().__init__("test_get_global_param")

        self.client = self.create_client(GetParameters,
                                         '/collision_monitor/get_parameters')

        request = GetParameters.Request()
        request.names = ['FootprintApproach.activate']
        from rcl_interfaces.msg import Parameter
        from collections.abc import Sequence
        from collections.abc import Set
        from collections import UserList
        from collections import UserString

        # ((isinstance(value, Sequence) or
        # isinstance(value, Set) or
        # isinstance(value, UserList)) and
        # not isinstance(value, str) and
        # not isinstance(value, UserString) and
        # all(isinstance(v, str) for v in value) and
        # True), 
        
        print (isinstance(request.names, Sequence) )
        print (isinstance(request.names, Set) )
        print (isinstance(request.names, UserList) )
        print (not isinstance(request.names, str) )
        print (not isinstance(request.names, UserString) )
        print (all(isinstance(v, str) for v in request.names))

        self.client.wait_for_service()

        future = self.client.call_async(request)
        future.add_done_callback(self.callback_global_param)

def main(args=None):
    rclpy.init(args=args)
    node = TestGetGlobalParam()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()