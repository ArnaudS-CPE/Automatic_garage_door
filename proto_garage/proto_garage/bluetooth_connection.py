import rclpy
from rclpy.node import Node
import bluetooth
import json
import os
from ament_index_python.packages import get_package_share_directory

class BluetoothManager(Node):
    def __init__(self):
        super().__init__('bluetooth_manager')
        
        # Path setup
        package_path = get_package_share_directory('proto_garage')
        database_path = os.path.join(package_path, 'database')
        self.json_file_path = os.path.join(database_path, 'plate_list.json')
        
        # Start Bluetooth server
        self.start_bluetooth_server()

    def start_bluetooth_server(self):
        """Initialize and listen for Bluetooth connections."""
        self.server_sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
        port = 22
        self.server_sock.bind(("", port))
        self.server_sock.listen(1)
        self.get_logger().info("Bluetooth Plate Manager is waiting for Bluetooth connections...")
        
        while rclpy.ok():
            try:
                client_sock, address = self.server_sock.accept()
                self.get_logger().info(f"Connection established with: {address}")
                self.handle_client(client_sock)
            except bluetooth.BluetoothError as e:
                self.get_logger().error(f"Bluetooth error: {e}")
                # Close and restart the server socket to clear any bad state
                self.server_sock.close()
                self.start_bluetooth_server()
                break

    def handle_client(self, client_sock):
        """Handle incoming client connection and process commands."""
        try:
            while rclpy.ok():
                recvdata = client_sock.recv(1024).decode("utf-8").strip()
                self.get_logger().info(f"Received: {recvdata}")

                if recvdata == "0":
                    self.get_logger().info("Resetting connection.")
                    break  # Disconnect and reset for the next client

                # Handle write and delete commands
                if recvdata.startswith("w:"):
                    plate_number = recvdata[2:]
                    status = self.modify_plate_in_json("w", plate_number)
                elif recvdata.startswith("d:"):
                    plate_number = recvdata[2:]
                    status = self.modify_plate_in_json("d", plate_number)
                else:
                    status = "invalid_command"

                self.get_logger().info(f"Status: {status}")
                client_sock.send(f"{status}\n")

        except bluetooth.BluetoothError as e:
            self.get_logger().error(f"Client connection error: {e}")
        finally:
            client_sock.close()
            self.get_logger().info("Client connection closed.")

    def modify_plate_in_json(self, command, plate_number):
        """Modify the JSON file to add or remove license plates."""
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

def main(args=None):
    rclpy.init(args=args)
    node = BluetoothManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Bluetooth Plate Manager node interrupted.")
    finally:
        node.server_sock.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
