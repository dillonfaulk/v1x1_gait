import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters
from std_srvs.srv import SetBool
from std_msgs.msg import Float64

class GaitCommander(Node):
    def __init__(self):
        super().__init__('gait_commander')

        # Explicitly list the leg node names
        self.leg_nodes = [
            '/front_left_leg',
            '/front_right_leg',
            '/rear_left_leg',
            '/rear_right_leg'
        ]

        # Service to flip direction (forward/reverse)
        self.srv = self.create_service(SetBool, 'set_direction', self.handle_set_direction)
        self.get_logger().info("GaitCommander ready. Call /set_direction with true=reverse, false=forward.")

        # Publisher for rotation angle
        self.rotation_pub = self.create_publisher(Float64, '/gait_rotation_angle', 10)
        self.get_logger().info("Publish to /gait_rotation_angle (std_msgs/Float64) to rotate gait trajectories.")

    def handle_set_direction(self, request, response):
        direction = "reverse" if request.data else "forward"
        self.get_logger().info(f"Setting all legs to {direction}")
        for leg in self.leg_nodes:
            self._set_param(leg, direction)
        response.success = True
        response.message = f"All legs set to {direction}"
        return response

    def _set_param(self, node_name: str, direction: str):
        client = self.create_client(SetParameters, f'{node_name}/set_parameters')
        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error(f'Service not available for {node_name}')
            return

        param = Parameter(name='gait_direction', value=direction)
        req = SetParameters.Request()
        req.parameters = [param.to_parameter_msg()]

        client.call_async(req)  # fire and forget
        self.get_logger().info(f'Sent gait_direction={direction} to {node_name}')


def main(args=None):
    rclpy.init(args=args)
    node = GaitCommander()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()