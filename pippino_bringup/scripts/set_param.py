from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters

import rclpy
from rclpy.node import Node


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(SetParameters, '/D455/D455/set_parameters')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = None

    def send_request(self):
        self.req = SetParameters.Request()

        param = Parameter()
        param.name = "enable_color"
        param.value.type = ParameterType.PARAMETER_BOOL
        param.value.bool_value = True
        self.req.parameters.append(param)

        self.future = self.cli.call_async(self.req)
        self.future.add_done_callback(lambda f: print('Future done!'))


def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()
    minimal_client.send_request()

    rclpy.spin(minimal_client)
    while rclpy.ok():
        if minimal_client.future.done():
            if minimal_client.future.result() is not None:
                response = minimal_client.future.result()
                minimal_client.get_logger().info(
                    'Result of set parameters: for %s' %
                    (str(response)))
            else:
                minimal_client.get_logger().info(
                    'Service call failed %r' % (minimal_client.future.exception(),))
            break

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()