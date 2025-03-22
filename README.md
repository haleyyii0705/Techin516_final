# Techin516_final
This is a script that controls the TurtleBot to move a certain distance, then the Kinova arm places a cube from the table into the robot, and finally the TurtleBot returns to a specified location.

## 🚀 Features

- Turtlebot moving forward certain distance, wait and moving backwards using .py.
- Kinova grab a cube on the table and put it on turtlebot using setting points provided by csv file.
- TurtleBot navigation in a maze using .py file.

## 📦 How to use
## 1. Connecting to the turtlebot and Kinova
- Connect laptop with turtlebot using the following command:
  ```bash
  ssh <username>@<ip_address>
- Edit .bashrc file of turtlebot to include following the line if needed
  ```export ROS_DOMAIN_ID=30```
- On you laptop, start the discovery server with:
  ```bash
  ssh <username>@<ip_address
- Start the bringup launch file on the Turtlebot:
  ```bash
  ros2 launch turtlebot3_bringup robot.launch.py
  
- Connect your laptop's ROS environment to the robot arm:
  ```bash
  ros2 launch kortex_bringup gen3_lite.launch.py \
  robot_ip:=<ip.of.the.arm> \
  launch_rviz:=false
- Launch MoveIt's planning server and Rviz
```bash
  ros2 launch kinova_gen3_lite_moveit_config robot.launch.py \
  robot_ip:=<ip.of.the.arm>
```
## 2. Build a package in ros2 workspace
- 
