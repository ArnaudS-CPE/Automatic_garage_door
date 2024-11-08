import rclpy
from rclpy.node import Node

from std_msgs.msg import Empty
from sensor_msgs.msg import Image

#import RPi.GPIO as GPIO
import cv2
from cv_bridge import CvBridge


class car_detection(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_car_detection = self.create_publisher(Image, 'car_detected', 10)
        timer_period = 0.5  # seconds

        self.bridge = CvBridge()

        #init GPIO


        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):

        if True: 
            cam = cv2.VideoCapture(0)
            result, image = cam.read()

            image_message = self.bridge.cv2_to_imgmsg(image, "passthrough")

            self.publisher_car_detection.publish(image_message)

            #publie sur le topic une image
            #alume les leds
            #attend x secondes
            #eteint les leds

            


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = car_detection()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()