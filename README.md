
# robot-rest-api
This project consists of two Python scripts - a REST server and a REST client - designed to monitor the status of a robot controlled by the Robot Operating System (ROS). The REST server exposes an endpoint to provide the current status of the robot, while the REST client continuously queries this endpoint to display the status in real-time. We are using flask API for a lightweight web framework

# Prerequisites
* Linux

* ROS

* Flask

* Flask-RESTful

* requests

Install the prerequisites and packages before proceeding. This code has been run on Ubuntu 20.04 LTS ROS Noetic.

## Turtlebot3 packages

bash 
sudo apt install ros-noetic-dynamixel-sdk
sudo apt install ros-noetic-turtlebot3-msgs
sudo apt install ros-noetic-turtlebot3 


