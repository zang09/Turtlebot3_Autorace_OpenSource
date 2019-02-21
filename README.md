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

      $ sudo chmod 777 /dev/video0
      $ sudo chmod 777 /dev/ttyUSB0
      $ sudo chmod 777 /dev/ttyACM0

Run ROBIT_vision2 Node
-
We use ROS usb_cam package. Install the kinetic usb_cam package. 
  Reference link: http://wiki.ros.org/usb_cam
       
       $ sudo apt-get install ros-kinetic-usb-cam
  
  You should edit launch file. 
      
      $ cd /opt/ros/kinetic/share/usb_cam/launch/
      $ sudo gedit usb_cam-test.launch 
    
  Fill in this contents:
      
      <launch>
        <node name="usb_cam" pkg="usb_cam" type="usb_cam_node" output="screen" >
          <param name="video_device" value="/dev/video0" />
          <param name="image_width" value="320" />
          <param name="image_height" value="240" />
          <param name="framerate" value="30" />
          <param name="pixel_format" value="mjpeg" />
          <param name="camera_frame_id" value="usb_cam" />
          <param name="autoexposure" value="false" />
          <param name="exposure" value="130" />
          <param name="focus" value="1" />
          <param name="brightness" value="130" />
          <param name="contrast" value="100" />
          <param name="saturation" value="100" />
          <param name="auto_white_balance" value="false" />
          <param name="white_balance" value="4800" />
          <param name="io_method" value="mmap"/>
        </node>
      </launch>
  
  Execute usb_cam node after "catkin_make".
   
      $ roslaunch usb_cam usb_cam-test.launch 
    
  The turtlevision node use ROS qt UI. You should install ros-kinetic-qt. To install command is:
  
      $ sudo apt-get install ros-kinetic-qt-*
    
  Execute Robit_vision2 node.  
  
      $ rosrun Robit_vision2 Robit_vision2     
  
  You can threshold the range of several color in turtlevision node UI and press the "save parameter" to save the range of color. If you press the "run" button, robot will start.
  
Run ROBIT_vision2 Node
-
 To execute master node commands are:  
  
      $ rosrun Robit_master Robit_master   

  ## Run turtlebot3 tunnel node
   Bring up basic packages to start TurtleBot3 applications.
    
        $ roslaunch turtlebot3_bringup turtlebot3_robot.launch

   If you want to launch Lidar sensor and core separately, please use below commands.
        
        $ roslaunch turtlebot3_bringup turtlebot3_lidar.launch
        $ roslaunch turtlebot3_bringup turtlebot3_core.launch
        
   Execute turtlebot3 tunnel node.
    
        $ roslaunch hls_lfcd_lds_driver goal_pulisher
        
