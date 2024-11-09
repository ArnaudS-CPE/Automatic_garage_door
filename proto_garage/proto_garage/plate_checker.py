import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class PlateChecker(Node):
    def __init__(self):
        super().__init__('plate_checker')

        # Publisher to send open commands to the door controller
        self.command_publisher = self.create_publisher(String, 'door_control_command', 10)
        
        # Subscriber to receive license plate numbers
        self.plate_subscriber = self.create_subscription(
            String,
            'license_plate_input',
            self.plate_callback,
            10
        )

        # Load allowed plates from the JSON file
        self.allowed_plates = self.load_allowed_plates('/home/tototime/ros2_ws/src/Automatic_garage_door/proto_garage/proto_garage/plate_list.json')

    def load_allowed_plates(self, filename):
        """Load allowed plates from the JSON file with the structure specified."""
        try:
            with open(filename, 'r') as f:
                data = json.load(f)
                # Extract plate values into a list
                return [plate['value'] for plate in data.get("plate_list", [])]
        except FileNotFoundError:
            self.get_logger().error(f"File '{filename}' not found.")
            return []
        except json.JSONDecodeError:
            self.get_logger().error(f"Error decoding JSON in '{filename}'.")
            return []

    def plate_callback(self, msg):
        """Callback to process received license plate number and publish command if allowed."""
        plate_number = msg.data
        if plate_number in self.allowed_plates:
            self.get_logger().info(f"Plate {plate_number} is allowed. Sending open command.")
            self.command_publisher.publish(String(data="open"))
        else:
            self.get_logger().info(f"Plate {plate_number} is NOT allowed. No action taken.")

def main(args=None):
    rclpy.init(args=args)
    plate_checker = PlateChecker()
    rclpy.spin(plate_checker)

    # Cleanup
    plate_checker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
