#!/bin/bash

# Test angles to try (in degrees)
angles=(0 30 60 90)

echo "Starting servo test..."
echo "Publishing to /sroi_gripper/command topic"

for angle in "${angles[@]}"; do
    echo "Testing angle: $angle degrees"
    # Publish at 50Hz for 1 second
    rostopic pub /sroi_gripper/command std_msgs/UInt16 "data: $angle" -r 50 &
    # Wait for 1 second to see the movement
    sleep 1
    # Kill the previous rostopic process
    pkill -f "rostopic pub.*$angle"
    echo "Completed angle: $angle degrees"
done

echo "Test completed" 