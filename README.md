# Description 

This project consists of two Python scripts - a REST server and a REST client - designed to monitor the status of a robot controlled by the Robot Operating System (ROS). The REST server exposes an endpoint to provide the current status of the robot, while the REST client continuously queries this endpoint to display the status in real-time.

We are using flask API for a lightweight web framework 

Flask-RESTful is an extension for Flask that adds support for quickly building REST APIs. It is a lightweight abstraction that works with your existing ORM/libraries.

The REST API server serves the GET endpoint /api/robot/status locally on port 7201.
The API server communicates with ROS and pulls the status information from the /move_base/status topic.
When called, the REST API endpoint provides a response containing the status and text fields from the /move_base/status topic. If the topic is not available, the REST API calls an error out with a 400 code and a Bad request message.

## Prerequisites
* Linux

* ROS

* Flask

* Flask-RESTful

* requests

Install the prerequisites and packages before proceeding. This code has been run on Ubuntu 20.04 LTS ROS Noetic.

## Installing Turtlebot3

```bash
sudo apt-get update
sudo apt-get upgrade
cd ~/catkin_ws/src/
git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git -b noetic-devel
git clone https://github.com/ROBOTIS-GIT/turtlebot3.git -b noetic-devel
cd ~/catkin_ws && catkin_make
```

## Turtlebot3 simulator 

```bash
~/catkin_ws/src/
git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
cd ~/catkin_ws && catkin_make
```

## Turtlebot3 packages

```bash 
sudo apt install ros-noetic-dynamixel-sdk
sudo apt install ros-noetic-turtlebot3-msgs
sudo apt install ros-noetic-turtlebot3 
```

## seting up bashrc 

```bash
cd
gedit .bashrc

ONCE BASHRC FILE OPENS PASTE THE FOLLOWING LINES IN A NEW LINE

alias burger='export TURTLEBOT3_MODEL=burger'
alias waffle='export TURTLEBOT3_MODEL=waffle'
alias tb3fake='roslaunch turtlebot3_fake turtlebot3_fake.launch'
alias tb3teleop='roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch'
alias tb3='roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch'
alias tb3maze='roslaunch turtlebot3_gazebo turtlebot3_world.launch'
alias tb3house='roslaunch turtlebot3_fake turtlebot3_house.launch'
alias tb3rviz='roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch'

PASTE THESE AT THE BOTTOM

source /opt/ros/noetic/setup.bash
source /home/*Your pc name*/catkin_ws/devel/setup.bash
export TURTLEBOT_MODEL=waffle
export SVGA_VGPU10=0
```


## dependecies 

```bash
 sudo apt-get install ros-noetic-joy ros-noetic-teleop-twist-joy \
  ros-noetic-teleop-twist-keyboard ros-noetic-laser-proc \
  ros-noetic-rgbd-launch ros-noetic-rosserial-arduino \
  ros-noetic-rosserial-python ros-noetic-rosserial-client \
  ros-noetic-rosserial-msgs ros-noetic-amcl ros-noetic-map-server \
  ros-noetic-move-base ros-noetic-urdf ros-noetic-xacro \
  ros-noetic-compressed-image-transport ros-noetic-rqt* ros-noetic-rviz \
  ros-noetic-gmapping ros-noetic-navigation ros-noetic-interactive-markers
```
# Steps

## Running the simulator 
```bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```


## Running the teleop(to control the bot)
```bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```



Check if the bot is moving using the 'waxd s ' keys

## Running rviz
We need to run rviz to map the world generated in gazeboo 
```bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
```



* after the world is mapped,save the map 
* It will be saved in two files with extensions .yaml and .pgm

## map 



* Once SLAM node is successfully up and running, TurtleBot3 will be exploring unknown area of the map using teleoperation. It is important to avoid vigorous movements such as changing the linear and angular speed too quickly. When building a map using the TurtleBot3, it is a good practice to scan every corner of the map.

# Robot Navigation Instructions

## Estimating Initial Pose

1. **Initiate Pose Estimation:**
   - Click on the "2D Pose Estimate" button available in the RViz menu.
   - On the displayed map, select the approximate location of the robot.
   - Adjust the large green arrow to indicate the robot's orientation.

2. **Refine Robot's Position:**
   - Launch the keyboard teleoperation node to precisely position the robot.
   - Use keyboard controls to move the robot and align it accurately on the map.
   - Move the robot back and forth to gather surrounding environment data for better localization.
   - To avoid conflicting commands, terminate the keyboard teleoperation node by entering Ctrl + C in the terminal.

## Setting Navigation Goal

1. **Define Navigation Goal:**
   - Access the RViz menu and choose the "2D Nav Goal" option.
   - Select the desired destination on the map.
   - Adjust the green arrow to indicate the direction the robot will face upon reaching the destination.


# running Navigation

run the same code for starting the gazeboo world 

run the following code to open rviz with the map made.

```bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml
```

* Click on 2D Pose estimation and click and drag to match the positon and direction of the bot located in the gazeboo simulation
 

* Click on the 2D Nav Goal nd drag the final destination point to where the bot must reach



The bot will move to the final point without hitting any obstacles as mapped 

# Running the Scripts 

## Running the Server
* Go to your catkin_workspace, eg catkin_ws/src
```bash
git clone https://github.com/Aravind-Sethu/robot-rest-api.git
cd ..
catkin_make
roslaunch robot-rest-api server.launch
```


* the luanch file contains the code to run the server side 

## Running of clients 
```bash
cd robot-rest-api
cd src
python read.py
```

## Test cases 

* case of accepting the Goal
  


* case of reaching the Goal
  

* Case of invalid message

## Refrences 

[SLAM](https://emanual.robotis.com/docs/en/platform/turtlebot3/slam/#slam)

[Simulation](https://emanual.robotis.com/docs/en/platform/turtlebot3/nav_simulation/)
