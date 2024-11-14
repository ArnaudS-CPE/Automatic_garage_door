import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
import serial

class ToFNode(Node):
    def __init__(self):
        super().__init__('tof_node')


        self.car_coming_in = self.create_subscription(Empty, 'car_coming_in', self.start, 10)


    def start(self,msg):
        parked=False
        while(parked==False):
            ser = serial.Serial(port='/dev/ttyUSB0',baudrate=9600)
            print('Voiture detectée')
            ser.write(str.encode('90'))
            value= ser.read(1)
            if(value!=None):
                print("Véhicule dans le garage")
                parked=True

def main(args=None):
    rclpy.init(args=args)
    node = ToFNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
