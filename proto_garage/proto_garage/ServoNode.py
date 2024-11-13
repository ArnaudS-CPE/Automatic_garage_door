import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
import serial
from gpiozero import AngularServo
from time import sleep
from std_msgs.msg import String

class ServoNode(Node):
    def __init__(self):
        super().__init__('servo_node')
        self.command_subscriber = self.create_subscription(String, 'door_control_command', self.action_porte ,10)
        self.car_coming_in = self.create_publisher(Empty, 'car_coming_in', 10)
        self.servogauche = AngularServo(4, min_pulse_width=0.0006, max_pulse_width=0.0023)
        self.servodroit = AngularServo(18, min_pulse_width=0.0006, max_pulse_width=0.0023)


    def action_porte(self,msg):
        if msg=="open":
            print('Ouverture de la porte')
            self.servogauche.angle = 90
            self.servodroit.angle = -90
            self.car_coming_in.publish(msg)
        elif msg=="close":
            print('Fermeture de la porte')
            self.servogauche.angle = 0
            self.servodroit.angle = 0

def main(args=None):
    rclpy.init(args=args)
    node = ServoNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
