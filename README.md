# Team ROBIT2

Introduction
-
The Team ROBIT is a robot team of Kwangwoon University in Seoul, Republic of Korea. Team ROBIT has been established in November 2006. We have studied hardware and software of robot system. We also has participated in several domestic and international robot competitions.

Overview
-
This document describes the team ROBIT2's hardware and software for 2018 R-Biz challenge Turtlebot3 autonomous race.

Hardware platform
-
+ [Turtlebot3](http://emanual.robotis.com/docs/en/platform/turtlebot3/overview/) 
+ Usb camera: Logitech Webcam HD Pro C930e
+ PC: Intel NUC7i5BNK (RAM 4GB)

# User's Guide

Uploading the modifed openCR firmware
-
This 'turtlebot3_core' package has been modified for the autonomous race. So you should upload this arduino sketch.

Setting the permission
-
You should set the permission of some devices. To set commands are:

  `$ sudo chmod 777 /dev/video0`
  `$ sudo chmod 777 /dev/ttyUSB0`
  `$ sudo chmod 777 /dev/ttyACM0`

**ROBIT2** team TurtlebotAutoRace Source File

1. Download ROS_QT_PLUGIN 4.5.1(for ROS KINETIC KAME)

2. Download USB_CAM PACKAGE to Get Camera message on Vision node

3. write roscore on the terminal 

4. roslaunch usb_cam usb_cam-test.launch on the terminal

5. rosrun ROBIT_vision2 ROBIT_vision2 on the terminal 

6. rosrun ROBIT_master ROBIT_master on the terminal
