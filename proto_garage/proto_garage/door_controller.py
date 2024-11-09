import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class DoorController(Node):
    def __init__(self):
        super().__init__('door_controller')
        
        
        # Initialize the door's state
        self.door_state = 'CLOSED'  # Possible states: CLOSED, OPEN

        # Create a publisher for the door state
        self.state_publisher = self.create_publisher(String, 'door_state', 10)
        
        # Set a timer to publish the door state at 1Hz
        self.timer = self.create_timer(0.5, self.publish_door_state)

        # Subscriber to receive commands
        self.command_subscriber = self.create_subscription(String, 'door_control_command', self.command_callback, 10)

    def publish_door_state(self):
        # Create and publish the door state message
        msg = String()
        msg.data = self.door_state
        self.state_publisher.publish(msg)
        

    def command_callback(self, msg):
        """Callback to receive open/close commands and execute based on the state machine logic."""
        command = msg.data
        self.get_logger().info(f"Received command: {command}")
        self.execute_command(command)

    def open_door(self):
        """Send a command to open the door if the state allows it."""
        if self.door_state == 'CLOSED':
            self.get_logger().info("Sending command to open the door.")
            self.door_state = 'OPEN'
        else:
            self.get_logger().info("Door is already open or opening, no action taken.")

    def close_door(self):
        """Send a command to close the door if the state allows it."""
        if self.door_state == 'OPEN':
            self.get_logger().info("Sending command to close the door.")
            self.door_state = 'CLOSED'
        else:
            self.get_logger().info("Door is already closed or closing, no action taken.")

    def execute_command(self, command):
        """Check command and execute based on the state machine logic."""
        if command == 'open':
            self.open_door()
        elif command == 'close':
            self.close_door()
        else:
            self.get_logger().warning("Unknown command received!")

def main(args=None):
    rclpy.init(args=args)
    door_controller = DoorController()
    # Spin the node to keep it alive and responsive to incoming commands
    rclpy.spin(door_controller)
    # Cleanup
    door_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
