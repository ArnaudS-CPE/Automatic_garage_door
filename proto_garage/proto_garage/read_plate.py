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

        self.reader = easyocr.Reader(['en'])



    def image_callback(self, msg):

        plate = String()
        plate_text = ""

        # Convert the image message to OpenCV format
        frame = self.bridge.imgmsg_to_cv2(msg)
        # cv2.imshow('image',frame)
        # cv2.waitKey(0)

        result = self.reader.readtext(frame)

        for (bbox, text, prob) in result:
            #(top_left, top_right, bottom_right, bottom_left) = bbox
            print(f'Text: {text}, Probability: {prob}')
            if (len(text) >= 7) and (len(text) <= 10) and (prob > 0.2):
                plate_text = text


        if len(plate_text) >= 7:
            plate_text = plate_text.replace("-", "")
            plate_text = plate_text.replace(" ", "")
            plate_text = plate_text.replace(",", "")
            plate_text = plate_text.replace(";", "")
            plate_text = plate_text.replace(":", "")
            plate_text = plate_text.replace(".", "")
            plate_text = plate_text[2:5]
            plate.data = plate_text
            self.publisher_plate.publish(plate)
            

def main(args=None):
    rclpy.init(args=args)
    node = plate_reader()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
