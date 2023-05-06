# Dependecies
 
```
sudo apt-get install ros-noetic-joy ros-noetic-teleop-twist-joy \
  ros-noetic-teleop-twist-keyboard ros-noetic-laser-proc \
  ros-noetic-rgbd-launch ros-noetic-rosserial-arduino \
  ros-noetic-rosserial-python ros-noetic-rosserial-client \
  ros-noetic-rosserial-msgs ros-noetic-amcl ros-noetic-map-server \
  ros-noetic-move-base ros-noetic-urdf ros-noetic-xacro \
  ros-noetic-compressed-image-transport ros-noetic-rqt* ros-noetic-rviz \
  ros-noetic-gmapping ros-noetic-navigation ros-noetic-interactive-markers
```
```
sudo apt install ros-noetic-dynamixel-sdk
sudo apt install ros-noetic-turtlebot3-msgs
sudo apt install ros-noetic-turtlebot3
```
# How to operate the Robot
* before any thing add the following 2 lines in your bashrc
```
#export ROS_MASTER_URI=http://your_device_ip:11311
#export ROS_IP=your_device_IP
```
save the file then out and close all the terminal  

## Terminal 1
1. ```roscore```
## Terminal 2
```
ssh ubuntu@<ip_of_the_robot>
```
**note**
* The password of the robot is **turtlebot**
* Enter your IP of the computer getting it from ```ifconfig``` in the MasterIP
* ```roslaunch turtlebot3_bringup turtlebot3_robot.launch```
## terminal 3
1. ```cd DEBI_competition```
2. ```roslaunch turtlebot3_manipulation_bringup turtlebot3_manipulation_bringup.launch```
## terminal 4
1. ```roslaunch turtlebot3_manipulation_moveit_config move_group.launch```


# Todo
## Day 1 Todo
- [x]  Calibrate camera
- [x]  tune parameter contours and Hough
- [x]  topics of Real Robot
- [x]  control robot
- [x]  mapping (by knowing starting position)
## Day 2 Todo
- [x] Make map from starting position (3)
- [x] Collect bag after maping (4)
- [x] Ask enna ezay nzbot el latency (5) --------> m4 han3rf n3ml 7aga hanm4e el robot bera7a awwwy
- [x] test el arm of the robot (1) 
- [x] move el robot o howa 4ayf el balls (2) 
## Day 3 Todo
-[] test amcl on real life
-[] test bashscript bta3 el connection 
-[] test perception  
-[] test PID
-[] 2dem el camera 
# Topics of Real life
```
/battery_state
/camera/camera_info
/camera/image
/cmd_vel
/cmd_vel_rc100
/diagnostics
/firmware_version
/gripper_move_time
/gripper_position
/imu
/joint_move_time
/joint_states
/joint_trajectory_point
/magnetic_field
/motor_power
/odom
/reset
/rosout
/rosout_agg
/rpms
/scan
/sensor_state
/sound
/tf
/version_info

```
# Output of meeting 4/5/2023
 * Slightly tune Hough 
 * t3rfo ezay 45al transform 
# 
