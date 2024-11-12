import bluetooth
import json
import os

# Function to load the JSON file and add a new license plate
def add_plate_to_json(plate_number, json_file_path):
    # Check if the JSON file exists
    if not os.path.exists(json_file_path):
        # If it doesn't exist, create an initial structure
        data = {"plate_list": []}
    else:
        # Load existing data from the JSON file
        with open(json_file_path, "r") as f:
            data = json.load(f)
    
    # Add the new plate to the list if not already present
    if not any(plate["value"] == plate_number for plate in data["plate_list"]):
        data["plate_list"].append({"value": plate_number})
        
        # Write the updated data back to the JSON file
        with open(json_file_path, "w") as f:
            json.dump(data, f, indent=4)
        return "written"
    else:
        return "already_exists"

# Initialize Bluetooth server
server_sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
port = 22
server_sock.bind(("", port))
server_sock.listen(1)
print("Waiting for Bluetooth connection...")
client_sock, address = server_sock.accept()
print("Connection established with:", address)

# Path to the JSON file
json_file_path = "proto_garage/database/plate_list.json"  # Update with the actual path

try:
    while True:
        # Receive data from Bluetooth client
        recvdata = client_sock.recv(1024).decode("utf-8").strip()
        print("Received:", recvdata)

        # If the received data is "0", end the loop
        if recvdata == "0":
            print("Ending connection.")
            break

        # Write the license plate to JSON and send confirmation
        status = add_plate_to_json(recvdata, json_file_path)
        print("Status:", status)
        client_sock.send(f"{status}\n")
        
except Exception as e:
    print("Error:", e)
finally:
    # Close the Bluetooth sockets
    client_sock.close()
    server_sock.close()
    print("Connection closed.")
