# The Helping Hand
## Robot Arm for Medicine Transportation

##### By Adebola Ajayi, Chinonso Okafor, Farouk Balogun, Oluwasubomi Bashorun and Yoofi Williams for Howard University Senior Capstone Project.

## Overview
### Problem Statement
Amidst the challenges posed by the COVID-19 outbreak, the medical sector faces a pressing issue - the heightened risk of healthcare professionals contracting the virus during patient interactions. This concern is substantiated by a 2021 survey conducted by the Kaiser Family Foundation and The Washington Post, emphasizing the urgent need for change.
Specifically, 21% of the frontline health care workers surveyed said the hardest part of working during the COVID-19 pandemic was worrying about exposure, getting sick, exposing family. The adjustments in hospital protocols during this period further highlight the crucial demand for a safer and more efficient method of delivering medications and essential supplies to patients, particularly those susceptible to highly contagious diseases.



### Project Description
Our project centers on configuring a robust robotic arm with precise object manipulation capabilities, specifically designed to revolutionize medicine dispensing in hospitals. The primary goal is to enhance both the quality of life for patients and the overall efficiency of medical processes. By utilizing this innovative robotic arm, we aim to minimize unnecessary interactions between patients and healthcare professionals during the medication dispensing process.
The robotic arm, equipped with a camera, will be capable of identifying medicines left for a patient and facilitating a seamless handoff to the intended recipient. This approach eliminates the necessity for physical contact between the person delivering the medicine and the patient, especially if the patient is in an isolation room. By reducing direct contact, we mitigate the risk of spreading highly contagious diseases such as COVID-19, making the medication delivery process safer and more efficient.


## Dependencies
The code in this repository was written for the **Kinova Gen3-7dof Robot Arm** with the **Robotiq 2F-85 Adaptive Gripper** attached; developed and tested on **Ubuntu 22.04** with **ROS2 Iron**. See below for a list of dependencies and links to installation instructions.
| Resource | Instructions |
|----------|----------|
| Ubuntu 22.04 | See installation instructions [here](https://releases.ubuntu.com/jammy/).|
| ROS2 Iron | See installation instructions [here](https://docs.ros.org/en/iron/Installation/Ubuntu-Install-Debians.html).|
| MoveIt2! | See installation instructions [here](https://moveit.ros.org/install-moveit2/binary/). Select ROS2 Iron.|
| MoveIt2! Tutorials | See setup instructions [here](https://moveit.picknik.ai/main/doc/tutorials/getting_started/getting_started.html). Follow the "Getting Started" guide to set up MoveIt.|
| Kinova Robotics ROS2_Kortex | See installation instructions [here](https://github.com/Kinovarobotics/ros2_kortex). Follow the instructions for "7dof arm".
| Apriltags (Python Bindings) | See installation instructions [here](https://github.com/duckietown/lib-dt-apriltags#installation).
| Xdotool | See installation instructions [here](https://github.com/jordansissel/xdotool).|



## Instructions
If you have set up all the dependencies listed above, go ahead and clone this repo.

On every new shell you open, run this command to have access to the ROS2 commands, or add it to your shell startup script (`.bashrc`).
```
# Replace ".bash" with your shell if you're not using bash
# Possible values are: setup.bash, setup.sh, setup.zsh
source /opt/ros/iron/setup.bash 
```

Then from the `amazon-robotics-p1` directory, run this command to resolve package dependencies.
```
rosdep install -i --from-path src --rosdistro iron -y
```

Then build the packages using `colcon`.
```
colcon build
```

### Pick and Place on the Physical Robot
From the `amazon-robotics-p1` directory, source your overlay using the following command:
```
source install/setup.bash
```
You should source your overlay in any new terminal shell you open up.

After sourcing, run the `robot_bringup.launch.py` launch file from the `arm_mobility` package:
```
ros2 launch arm_mobility robot_bringup.launch.py
```

Optionally, run `initial_position_node` in another terminal to bring the robot to a predefined initial pose. The node should terminate when the robot is at the pose:
```
ros2 run arm_mobility initial_position_node
```

In another terminal, run the `kinova_vision.launch.py` launch file from the `kinova_vision` package to launch the robot camera nodes:
```
ros2 launch kinova_vision kinova_vision.launch.py
```

Then in another terminal, run the `robot_pick_place.launch.py` launch file from the `arm_mobility` package to start the task:
```
ros2 launch arm_mobility robot_pick_place.launch.py
```

At this point, RViz is loaded and should show the state of the robot and the detected medicine bottle (the bottle is detected with the help of `tag36h11` apriltags). Select any of the planned solutions for `pick_place_task` and execute.

We have provided a script `run_pick_place.sh` that you can use in place of entering the launch commands manually, assuming the robot is already in an appropriate initial pose where the object is in the camera frame:
```
# from the amazon-robotics-p1, run
./run_pick_place.sh
```

### Pick and Place in Simulation
From the `amazon-robotics-p1` directory, source your overlay using the following command:
```
source install/setup.bash
```
You should source your overlay in any new terminal shell you open up.

After sourcing, run the `robot_bringup.launch.py` launch file from the `arm_mobility` package with the specified argument:
```
ros2 launch arm_mobility robot_bringup.launch.py use_fake_hardware:=true
```

In another terminal, run the `robot_pick_place.launch.py` launch file from the `arm_mobility` package to start the task:
```
ros2 launch arm_mobility robot_pick_place.launch.py
```

The process will be waiting until the object pose is received on the `/object_3d_pose` topic. You can publish the object pose directly in a new terminal shell like this:
```
ros2 topic pub --once /object_3d_pose geometry_msgs/msg/Pose '{position: {x: 0.36, y: -0.04}}'
```

At this point, RViz is loaded and should show the state of the robot and the medicine bottle at the pose specified. Select any of the planned solutions for `pick_place_task` and execute.


### Optional UI with Flask
This section includes an optional user interface (UI) powered by Flask, allowing users to interact with the robot through a web-based interface.

#### Setup Instructions
To use the optional UI, follow these steps:

1. Install the required dependencies by running the command below:
```
pip install -r requirements.txt
```

2. Run the Flask application:
```
flask --app user_interface/operate run
```

3. Open your web browser and navigate to 'http://localhost:5000' to access the UI.


## References
- Michael Görner*, Robert Haschke*, Helge Ritter, and Jianwei Zhang, "MoveIt! Task Constructor for Task-Level Motion Planning", International Conference on Robotics and Automation (ICRA), 2019, Montreal, Canada.  [DOI](https://doi.org/10.1109/ICRA.2019.8793898) [PDF](https://pub.uni-bielefeld.de/download/2918864/2933599/paper.pdf).