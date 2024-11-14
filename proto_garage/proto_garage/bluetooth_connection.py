import rclpy
from rclpy.node import Node
import bluetooth
import json
import os
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory

class BluetoothManager(Node):
    def __init__(self):
        super().__init__('bluetooth_manager')
        
        # Publisher to send open commands to the door controller
        self.command_publisher = self.create_publisher(String, 'door_control_command', 10)

        # Load JSON file path dynamically
        package_path = get_package_share_directory('proto_garage')
        database_path = os.path.join(package_path, 'database')
        self.json_file_path = "/home/prototype/ws_proto/src/Automatic_garage_door/proto_garage/database/plate_list.json"

        # Bluetooth setup
        self.server_sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
        self.server_sock.bind(("", 22))

        # Set up a timer to periodically check for connections
        self.create_timer(1.0, self.listen_for_connections)  # Calls listen_for_connections every second

    def listen_for_connections(self):
        self.server_sock.listen(1)
        self.get_logger().info("Waiting for Bluetooth connection...")
        
        try:
            client_sock, address = self.server_sock.accept()
            self.get_logger().info(f"Connection established with: {address}")
            
            # Start reading from the client socket
            self.handle_client_connection(client_sock)
        
        except Exception as e:
            self.get_logger().error(f"Connection error: {e}")

    def handle_client_connection(self, client_sock):
        try:
            while True:
                recvdata = client_sock.recv(1024).decode("utf-8").strip()
                self.get_logger().info(f"Received: {recvdata}")

                # Check for "open" command to publish to the door controller
                if recvdata == "open":
                    status = self.send_open_command()
                elif recvdata == "0":
                    self.get_logger().info("Resetting connection.")
                    break
                elif recvdata.startswith("w:"):
                    plate_number = recvdata[2:]
                    status = self.modify_plate_in_json("w", plate_number)
                elif recvdata.startswith("d:"):
                    plate_number = recvdata[2:]
                    status = self.modify_plate_in_json("d", plate_number)
                else:
                    status = "invalid_command"

                # Send status back to Bluetooth client
                self.get_logger().info(f"Status: {status}")
                client_sock.send(f"{status}\n")
        
        except Exception as e:
            self.get_logger().error(f"Error: {e}")
        
        finally:
            client_sock.close()
            self.get_logger().info("Connection closed.")

    def send_open_command(self):
        """Publishes an open command to the door control topic."""
        try:
            self.command_publisher.publish(String(data="open"))
            self.get_logger().info("Sent open command to door controller.")
            return "door_open"
        except Exception as e:
            self.get_logger().error(f"Error: {e}")
            return "invalid_command"

    def modify_plate_in_json(self, command, plate_number):
        # Check if the JSON file exists
        if not os.path.exists(self.json_file_path):
            data = {"plate_list": []}
        else:
            with open(self.json_file_path, "r") as f:
                data = json.load(f)

        if command == "w":
            if not any(plate["value"] == plate_number for plate in data["plate_list"]):
                data["plate_list"].append({"value": plate_number})
                with open(self.json_file_path, "w") as f:
                    json.dump(data, f, indent=4)
                return "written"
            else:
                return "already_exists"
        
        elif command == "d":
            updated_list = [plate for plate in data["plate_list"] if plate["value"] != plate_number]
            if len(updated_list) < len(data["plate_list"]):
                data["plate_list"] = updated_list
                with open(self.json_file_path, "w") as f:
                    json.dump(data, f, indent=4)
                return "deleted"
            else:
                return "not_found"

        return "invalid_command"

def main(args=None):
    rclpy.init(args=args)
    bluetooth_plate_manager = BluetoothManager()
    rclpy.spin(bluetooth_plate_manager)
    bluetooth_plate_manager.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()