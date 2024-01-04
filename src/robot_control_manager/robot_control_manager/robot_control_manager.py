import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class RobotControlNode(Node):
    def __init__(self):
        super().__init__('robot_control_node')

        # Suscripción a comandos de velocidad
        self.left_wheel_cmd_sub = self.create_subscription(Float64, '/my_robot_control/left_wheel_joint/velocity_cmd', self.left_wheel_cmd_callback, 10)
        self.right_wheel_cmd_sub = self.create_subscription(Float64, '/my_robot_control/right_wheel_joint/velocity_cmd', self.right_wheel_cmd_callback, 10)

        # Publicación de estados de posición y velocidad
        self.left_wheel_state_pub = self.create_publisher(Float64, '/my_robot_control/left_wheel_joint/velocity_state', 10)
        self.right_wheel_state_pub = self.create_publisher(Float64, '/my_robot_control/right_wheel_joint/velocity_state', 10)

    # def left_wheel_cmd_callback(self, msg):
    #     # Lógica para manejar el comando de velocidad de la rueda izquierda
    #     # ...

    # def right_wheel_cmd_callback(self, msg):
    #     # Lógica para manejar el comando de velocidad de la rueda derecha
    #     # ...

def main(args=None):
    rclpy.init(args=args)
    node = RobotControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
