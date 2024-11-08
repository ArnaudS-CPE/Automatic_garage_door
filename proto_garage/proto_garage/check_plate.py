import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import String

#import RPi.GPIO as GPIO
import cv2
from cv_bridge import CvBridge

import easyocr


class plate_reader(Node):

    def __init__(self):

        super().__init__('minimal_publisher')

        self.publisher_plate = self.create_publisher(String, 'plate', 10)

        self.image_sub = self.create_subscription(Image, 'car_detected', self.image_callback, 10)

        timer_period = 0.5  # seconds

        self.bridge = CvBridge()

        #init GPIO


        self.timer = self.create_timer(timer_period, self.timer_callback)

    def image_callback(self, msg):
        if self.flight_mode == 2:
            # Convert the image message to OpenCV format
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            frame_center_x = frame.shape[1] // 2  
            frame_center_y = frame.shape[0] // 2 
            dist_center = 70
            horizontal_position = "center"
            vertical_position = "center"
            position_y = "center"


            self.publisher_control.publish(self.twist)
            

def main(args=None):
    rclpy.init(args=args)
    node = plate_reader()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
