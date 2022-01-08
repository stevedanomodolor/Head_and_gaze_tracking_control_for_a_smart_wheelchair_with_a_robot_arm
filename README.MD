# Head and gaze tracking control for a smart wheelchair with a robot arm

# Table of Contents
* [General Info](#General-info)
* [Dependencies](#Dependencies)
* [Build](#Build)
* [Usage](#Usage)
* [Contributing authors](#Contributing-authors)


## General Info
The code contains an implementation of the article head and gaze tracking control for a smart wheelchair with robotic arm. Check the following article for an in depth information. **TODO add link to readme**. It is part of a project of the human  robot interaction & operation course of the master's degree in automatic control and robotic UPC ETSEIB


![Output sample](files/final_demo.gif)


## Dependencies
1. In order to execute the python code with ros, you need to have a python3 environment compatible with ros. In order to do so, follow the instructions in the following video(only install necessay dependecies!!!!) https://www.youtube.com/watch?v=oxK4ykVh1EE
2. Download the husky package, this can be seen in the following link
http://wiki.ros.org/husky_gazebo/Tutorials/Simulating%20Husky
3. Download the robotiq gripper package in your src folder
```
git clone https://github.com/utecrobotics/robotiq
```
## Build
1. To build, create a new catkin_ws workspace(xxxxx_ws->name the worksapce as you wish)
```
mkdir -p ~/xxxxx_ws/src
```
2. git clone this repository to the src folder
```
cd ~/xxxxx_ws/src
git clone https://github.com/stevedanomodolor/robot_learning.git
```
3. go back to the xxxxx_ws folder and build
```
cd ~/xxxxx_ws
catkin_make
```

## Usage
To run the code, open 3 terminals
1. In the first terminal, launch the gazebo simulation.
```
cd ~/xxxxx_ws
source ~/xxxxx_ws/devel/setup.bash
roslaunch husky_ur5_robotiq_85_moveit_config demo_gazebo.launch
```
2. In the second terminal, activate the python3 virtual environment,
 ```
 cd ~/xxxxx_ws
 source ~/python3_ws/py3venv/bin/activate
 source ~/xxxxx_ws/devel/setup.bash
 cd ~/xxxxx_ws/src/mobile_manipulation_alg/scripts/tracker
 python eye_tracker_node.py
 ```
3. In the last terminal, run the following command to run the husky controller node
```
cd ~/xxxxx_ws
source ~/xxxxx_ws/devel/setup.bash
rosrun mobile_manipulation_alg wheel_chair_head_control_manager_node
```

## Contributing authors
1. stevedan.ogochu.omodolor@estudiantat.upc.edu;
2. pablo.manas@estudiantat.upc.edu;
3. javier.pedrosa@estudiantat.upc.edu;
These authors contributed equally to this work.
