import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
import serial
from gpiozero import AngularServo
import time
from std_msgs.msg import String

class ServoNode(Node):
    def __init__(self):
        super().__init__('servo_node')
        self.command_subscriber = self.create_subscription(String, 'door_control_command', self.action_porte ,10)
        self.car_coming_in = self.create_publisher(Empty, 'car_coming_in', 10)
        self.servo = AngularServo(18, min_pulse_width=0.0006, max_pulse_width=0.0023,initial_angle=90)


    def action_porte(self,msg):
        msgtof=Empty()
        if msg.data=="open":
            self.get_logger().info('Door opening')
            self.servo.angle = 90
            self.car_coming_in.publish(msgtof)
        elif msg.data=="close":
            self.get_logger().info('Closing the door')
            self.servo.angle = 0

def main(args=None):
    rclpy.init(args=args)
    node = ServoNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
