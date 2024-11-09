# Ensure the script is executable
# Run `chmod +x scenario.sh` before executing this script

# Function to publish a license plate number and display the door's status
publish_plate() {
    local plate_number=$1
    echo "Publishing license plate: $plate_number"
    ros2 topic pub --once /license_plate_input std_msgs/msg/String "data: '$plate_number'"

    # Check if the door opened
    echo "Checking door status..."
    door_status=$(ros2 topic echo /door_state --once | grep "data: 'OPEN'")

    
    if [ -n "$door_status" ]; then
        echo "Door opened for plate $plate_number. Closing the door..."
        ros2 topic pub --once /door_control_command std_msgs/msg/String "data: 'close'"
    else
        echo "Door did not open for plate $plate_number."
    fi
}

# Start the door_controller and plate_checker nodes if they aren't already running
echo "Starting door controller and plate checker nodes..."
# Replace `<your_package_name>` with the actual package name
ros2 run proto_garage door_controller &
ros2 run proto_garage plate_checker &

# Wait a few seconds for nodes to start up
sleep 3

# List of test license plates
plates=("DD001CJ" "AB123CD" "ZZ999ZZ")

# Publish each plate number and check the door status
for plate in "${plates[@]}"; do
    publish_plate "$plate"
done

# Kill background processes (nodes) after testing
kill %1 %2
echo "Scenario complete. Door controller and plate checker nodes stopped."
