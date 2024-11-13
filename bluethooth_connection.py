import bluetooth
import json
import os

# Function to load the JSON file, add or remove a license plate
def modify_plate_in_json(command, plate_number, json_file_path):
    # Check if the JSON file exists
    if not os.path.exists(json_file_path):
        # Initialize file structure if it does not exist
        data = {"plate_list": []}
    else:
        # Load existing data from the JSON file
        with open(json_file_path, "r") as f:
            data = json.load(f)
    
    # Handle write command
    if command == "w":
        if not any(plate["value"] == plate_number for plate in data["plate_list"]):
            data["plate_list"].append({"value": plate_number})
            with open(json_file_path, "w") as f:
                json.dump(data, f, indent=4)
            return "written"
        else:
            return "already_exists"

    # Handle delete command
    elif command == "d":
        updated_list = [plate for plate in data["plate_list"] if plate["value"] != plate_number]
        if len(updated_list) < len(data["plate_list"]):  # If any entry was removed
            data["plate_list"] = updated_list
            with open(json_file_path, "w") as f:
                json.dump(data, f, indent=4)
            return "deleted"
        else:
            return "not_found"

# Path to the JSON file
json_file_path = "/path/to/database/plate_list.json"  # Update with the actual path

# Initialize Bluetooth server
server_sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
port = 22
server_sock.bind(("", port))

while True:
    server_sock.listen(1)
    print("Waiting for Bluetooth connection...")
    client_sock, address = server_sock.accept()
    print("Connection established with:", address)

    try:
        while True:
            # Receive data from Bluetooth client
            recvdata = client_sock.recv(1024).decode("utf-8").strip()
            print("Received:", recvdata)

            # Check for reset command to wait for a new connection
            if recvdata == "0":
                print("Resetting connection.")
                break  # Go back to waiting for a new connection

            # Process commands based on the received data format
            if recvdata.startswith("w:"):
                plate_number = recvdata[2:]
                status = modify_plate_in_json("w", plate_number, json_file_path)
            elif recvdata.startswith("d:"):
                plate_number = recvdata[2:]
                status = modify_plate_in_json("d", plate_number, json_file_path)
            else:
                status = "invalid_command"

            # Send status back to Bluetooth client
            print("Status:", status)
            client_sock.send(f"{status}\n")

    except Exception as e:
        print("Error:", e)
    finally:
        # Close the client connection and go back to listening for a new one
        client_sock.close()
        print("Connection closed.")
