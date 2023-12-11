import rclpy
from geometry_msgs.msg import Twist

def callback(msg):
    print(f"Received: {msg}")

def main():
    rclpy.init()

    node = rclpy.create_node('simple_subscriber_node')

    # Cambia 'motor_cmd' al nombre del tópico al que quieres suscribirte
    subscription = node.create_subscription(Twist, '/cmd_vel', callback, 10)

    try:
        while rclpy.ok():
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()