#include <ros/ros.h>
#include "hls_lfcd_lds_driver/goal_publisher.h"
#include <hls_lfcd_lds_driver/lfcd_laser.h>
#include <sensor_msgs/Imu.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#define RAD2DEG (180.0 / M_PI)
void imuMsgCallback(const sensor_msgs::Imu::ConstPtr &imu);
void planMsgCallback(const nav_msgs::Path &plan);
void locationMsgCallback(const geometry_msgs::PolygonStamped &location);
void secCallback(const ros::TimerEvent&);
void ScanMsgCallBack(const sensor_msgs::LaserScan::ConstPtr &scan);
void motorVel(double x, double z);
void Turn(int start, int end, int gain);
int sec = 0;
int laserdata[10] = {0,};
int frontlaserdata = 0;
double direction_angle = 0.0;
bool firstObstacle = 0;
bool secondObstacle = 0;
bool thirdObstacle = 0;
bool zz = 0;
int motiontime = 0;
double speeeeeed = 0;
// publish motor vel


int main(int argc, char **argv)
{

  ros::init(argc, argv, "o4jang_publisher");
  ros::NodeHandle nh;
  ros::Publisher motor_vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel",1);
  ros::NodeHandle n;
  ros::NodeHandle action_nh("move_base");
  ros::Subscriber imu_sub_        = n.subscribe("/imu",10,&imuMsgCallback);
  ros::Subscriber scan_sub_       = n.subscribe("/scan",10,&ScanMsgCallBack);
  ros::Timer sec_timer_           = n.createTimer(ros::Duration(0.1),&secCallback);
  geometry_msgs::Twist motor_vel;
  ros::Rate loop_rate(200);

  while (ros::ok())
  {

    ROS_INFO("ANGLE : %lf", direction_angle);
    ROS_INFO("First :  %d", firstObstacle);
    ROS_INFO("Second : %d", secondObstacle);
    ROS_INFO("Third : %d", thirdObstacle);
    ROS_INFO("zz : %d", zz);
    ROS_INFO("sec : %d",sec);

    motor_vel_pub_.publish(motor_vel);

    if(direction_angle <= 95 && direction_angle >= 85)


    //Turn(90,180,20);


/*
    if(firstObstacle == 0 && secondObstacle == 0)
    {
        if(frontlaserdata >= 35)                              //go straight
        {
            motor_vel.linear.x = 0.10;
            motor_vel_pub_.publish(motor_vel);
        }
        else if(frontlaserdata < 35 && frontlaserdata != 0)  //stop
        {
            motor_vel.linear.x = 0.0;
            motor_vel_pub_.publish(motor_vel);
            firstObstacle = 1;
        }
    }
    else if(firstObstacle == 1 && secondObstacle == 0)
    {
      if(thirdObstacle == 0)                                //turn left
      {
          if(direction_angle <= 95 && direction_angle >= 85)
          {
            motor_vel.linear.x = 0.0;
            motor_vel.linear.y = 0.0;
            motor_vel.angular.z = 0.0;
            motor_vel_pub_.publish(motor_vel);
            secondObstacle = 1;
            motiontime = sec;
          }
          else
          {
            motor_vel.angular.z = 1.0;
            motor_vel_pub_.publish(motor_vel);
          }
      }
      else if(thirdObstacle == 1)                          //turn right
      {
          if(direction_angle <= 275 && direction_angle >= 265)
          {
            motor_vel.linear.x = 0.0;
            motor_vel.angular.z = 0.0;
            motor_vel_pub_.publish(motor_vel);
            zz = 1;
            secondObstacle = 1;
            motiontime = sec;
          }
          else
          {
            motor_vel.angular.z = -1.0;
            motor_vel_pub_.publish(motor_vel);
          }
       }
    }
    else if(secondObstacle == 1)
    {
      if(zz == 0)
      {
        if(sec - motiontime > 38)
        {
          if(direction_angle <= 5 || direction_angle >= 355)  //turn right
          {
            motor_vel.linear.x = 0.0;
            motor_vel.angular.z = 0.0;
            motor_vel_pub_.publish(motor_vel);
            thirdObstacle = 1;
            firstObstacle = 0;
            secondObstacle = 0;
            zz =1;
          }
          else
          {
            motor_vel.linear.x = 0.0;
            motor_vel.angular.z = -1.0;
            motor_vel_pub_.publish(motor_vel);
          }
        }
        else
        {
          motor_vel.linear.x = 0.1;
          motor_vel_pub_.publish(motor_vel);
        }
      }
      else
      {
        if(sec - motiontime > 38)
        {
          if(direction_angle <= 5 || direction_angle >= 355)  //turn right
          {
            motor_vel.linear.x = 0.0;
            motor_vel.angular.z = 0.0;
            motor_vel_pub_.publish(motor_vel);
          }
          else
          {
            motor_vel.linear.x = 0.0;
            motor_vel.angular.z = 1.0;
            motor_vel_pub_.publish(motor_vel);
          }
        }
        else
        {
          motor_vel.linear.x = 0.1;
          motor_vel_pub_.publish(motor_vel);
        }
      }
    }
*/
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
void imuMsgCallback(const sensor_msgs::Imu::ConstPtr &imu)
{
    double imu_x = imu->orientation.x;
    double imu_y = imu->orientation.y;
    double imu_z = imu->orientation.z;
    double imu_w = imu->orientation.w;

    double yaw = atan2(2*imu_x*imu_y + 2*imu_w*imu_z, imu_w*imu_w + imu_x*imu_x - imu_y*imu_y - imu_z*imu_z);
    double yaw_angle = yaw * RAD2DEG;


    if(yaw_angle > 0.0 && yaw_angle <= 180.0)
      direction_angle = yaw_angle;
    else if(yaw_angle <= 0.0 && yaw_angle > -180.0)
      direction_angle = yaw_angle + 360.0;
}
void secCallback(const ros::TimerEvent&)
{
    sec++;
}
void ScanMsgCallBack(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    int count = 0, sumfrontlaserdata = 0;
    for(int i = 0, count = 0; i < 5; i++)
    {
      laserdata[i] = scan->ranges[i] * 100;
      if(laserdata[i] != 0)
      {
        sumfrontlaserdata += laserdata[i];
        count ++;
      }
    }
    for(int i = 355; i <360; i ++)
    {
      laserdata[i] = scan->ranges[i] * 100;
      if(laserdata[i] != 0)
      {
        sumfrontlaserdata += laserdata[i];
        count ++;
      }
    }
    if(count != 0)
      frontlaserdata =  sumfrontlaserdata / count;
}
