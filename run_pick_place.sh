#!/bin/bash

gnome-terminal --tab --title="Loading robot in RViz" -- bash -c "source ~/amazon-robotics-p1/install/setup.bash; ros2 launch arm_mobility robot_bringup.launch.py"
gnome-terminal --tab --title="Loading Pick and Place" -- bash -c "source ~/amazon-robotics-p1/install/setup.bash; ros2 launch arm_mobility robot_pick_place.launch.py"
gnome-terminal --tab --title="Loading robot camera" -- bash -c "source ~/amazon-robotics-p1/install/setup.bash; ros2 launch kinova_vision kinova_vision.launch.py"