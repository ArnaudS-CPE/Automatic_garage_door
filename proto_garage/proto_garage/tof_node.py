import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
import serial
from std_msgs.msg import String

class ToFNode(Node):
    def __init__(self):
        super().__init__('tof_node')


        self.car_coming_in = self.create_subscription(Empty, 'car_coming_in', self.start, 10)
        self.car_parked = self.create_publisher(String, 'door_control_command', 10)
        self.msg = String()
        self.msg.data = "close"


    def start(self,msg):
        parked=False
        while(parked==False):
            ser = serial.Serial(port='/dev/ttyUSB0',baudrate=9600)
            print('Voiture detectée')
            ser.write(str.encode('90'))
            value= ser.read(1)
            if(value!=None):
                print("Véhicule dans le garage")
                self.car_parked.publish(self.msg)
                parked=True

def main(args=None):
    rclpy.init(args=args)
    node = ToFNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
