#!/bin/bash

# Getting the apriltag family passed in while running the script, with default value "tag36h11".
apriltag_family="${1:-tag36h11}"


# Check if the argument is provided
if [ -z "$1" ]; then
    # Argument not provided, use the default value
    arg="$apriltag_family"
else
    # Argument provided, use it
    arg="$1"
fi

gnome-terminal --window --title="Project Shell" -- bash -c "source ~/amazon-robotics-p1/install/setup.bash; ros2 launch arm_mobility robot_bringup.launch.py"
gnome-terminal --window --title="Project Shell" -- bash -c "source ~/amazon-robotics-p1/install/setup.bash; ros2 launch arm_mobility robot_pick_place.launch.py apriltag_family:=$arg"
gnome-terminal --window --title="Project Shell" -- bash -c "source ~/amazon-robotics-p1/install/setup.bash; ros2 launch kinova_vision kinova_vision.launch.py"