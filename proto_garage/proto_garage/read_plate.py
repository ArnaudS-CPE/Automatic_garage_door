import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import String

import cv2
from cv_bridge import CvBridge

import easyocr


class plate_reader(Node):

    def __init__(self):

        super().__init__('minimal_publisher')

        self.publisher_plate = self.create_publisher(String, 'plate', 10)

        self.image_sub = self.create_subscription(Image, 'car_detected', self.image_callback, 10)

        self.bridge = CvBridge()

        reader = easyocr.Reader(['en'])



    def image_callback(self, msg):

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')




        if self.flight_mode == 2:
            # Convert the image message to OpenCV format
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv2.imshow('image',frame)
            cv2.waitKey(0)


            self.publisher_control.publish(self.twist)
            

def main(args=None):
    rclpy.init(args=args)
    node = plate_reader()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
