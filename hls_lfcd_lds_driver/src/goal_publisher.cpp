#include <ros/ros.h>
#include "hls_lfcd_lds_driver/goal_publisher.h"
#include <hls_lfcd_lds_driver/lfcd_laser.h>
#include <std_msgs/UInt32.h>
#include <sensor_msgs/Imu.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Bool.h>
// define
#define RAD2DEG (180.0 / M_PI)
#define XTARGET 1.52
#define YTARGET 1.55
#define MotorValue 0.001

//function
void imuMsgCallback(const sensor_msgs::Imu::ConstPtr &imu);
void locationMsgCallback(const geometry_msgs::PolygonStamped &location);
void secCallback(const ros::TimerEvent&);
void ScanMsgCallBack(const sensor_msgs::LaserScan::ConstPtr &scan);
void masterDataCallback(const std_msgs::Bool::ConstPtr &master_data);
//timer & motion
int sec = 0;
int motiontime = 0;

//distance (laser)
int laserdata[10] = {0,};
int frontlaserdata = 0;


//footprint
double x_final = 0.0;
double y_final = 0.0;
//master -> tunnel
bool tunnelState = 0;

//motorspeed
double x_before = 0.0;
double y_before = 0.0;
double x_delta = 0.0;
double y_delta =0.0;

//for escape variable
double current_location = 0.0;
// imu angle
double direction_angle = 0.0;

int main(int argc, char **argv)
{
    //algorithm flag
    int reachflag = 0;
    bool stopflag = 0;
    bool distanceflag = 0;
    bool motionflag = 0;
    ros::init(argc, argv, "goal_publisher");
    ros::NodeHandle n;
    ros::NodeHandle action_nh("move_base");

    ros::Subscriber location_sub_   = n.subscribe("/move_base/global_costmap/footprint",10,&locationMsgCallback);
    ros::Subscriber imu_sub_        = n.subscribe("/imu",10,&imuMsgCallback);
    ros::Subscriber scan_sub_       = n.subscribe("/scan",10,&ScanMsgCallBack);
    ros::Subscriber tunnel_sub_     = n.subscribe("tunnel_state",1,&masterDataCallback);
    ros::Publisher robot_init_pub_  = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose",1);
    ros::Publisher action_goal_pub_ = action_nh.advertise<move_base_msgs::MoveBaseActionGoal>("goal",1);
    ros::Publisher motor_vel_pub_   = n.advertise<geometry_msgs::Twist>("cmd_vel",1);
    ros::Publisher tunnel_escpae_pub= n.advertise<std_msgs::UInt32>("tunnel_escape",1);
    ros::Timer sec_timer_           = n.createTimer(ros::Duration(1,0),&secCallback);

    geometry_msgs::PoseWithCovarianceStamped robot_init;
    move_base_msgs::MoveBaseActionGoal action_goal;
    geometry_msgs::Twist motor_vel;

    ros::Rate loop_rate(200);

    while (ros::ok())
    {
        if(tunnelState == 1)
        {
            if(stopflag == 0)
            {
                motor_vel.linear.x = 0.0;
                motor_vel.angular.z = 0.0;
                motor_vel_pub_.publish(motor_vel);
                stopflag =1;
            }
            if(sec == 1)      // Setting init(back 0,0)
            {
                robot_init.header.seq = 0;
                robot_init.header.stamp = ros::Time::now();
                robot_init.header.frame_id = "map";
                robot_init.pose.pose.position.x = 0.08;
                robot_init.pose.pose.position.y = 0.0;
                robot_init.pose.pose.position.z = 0.0;
                robot_init.pose.pose.orientation.x = 0.0;
                robot_init.pose.pose.orientation.y = 0.0;
                robot_init.pose.pose.orientation.z = 0.0 ;
                robot_init.pose.pose.orientation.w = 1.0 ;
                robot_init_pub_.publish(robot_init);
            }
            else if(sec == 3) // Setting target
            {
                action_goal.header.seq = 0;
                action_goal.header.stamp = ros::Time::now();
                action_goal.goal.target_pose.header.stamp = ros::Time::now();
                action_goal.goal.target_pose.header.frame_id = "map";
                action_goal.goal.target_pose.pose.position.x = 1.52; //XTARGET
                action_goal.goal.target_pose.pose.position.y = 1.55; //YTARGET
                action_goal.goal.target_pose.pose.position.z = 0.0;
                action_goal.goal.target_pose.pose.orientation.x = 0.0;
                action_goal.goal.target_pose.pose.orientation.y = 0.0;
                action_goal.goal.target_pose.pose.orientation.z = 0.0;
                action_goal.goal.target_pose.pose.orientation.w = 1.0;
                action_goal_pub_.publish(action_goal);
            }
            else if(sec > 3)  // Escape tunnel
            {
                //ROS_INFO("========================================");
                //ROS_INFO("x : %lf, y : %lf, current_location : %lf",x_final,y_final,current_location);
                //ROS_INFO("imu : %lf",direction_angle);
                //ROS_INFO("Motor x : %lf, Motor y : %lf",x_delta,y_delta);

                if(reachflag == 0)
                {
                    if(current_location <= 0.01)                         //this is circle equation, error is 0.05m
                    {
                        if((x_delta <= MotorValue && x_delta >= -MotorValue)||(y_delta <= MotorValue && y_delta >= -MotorValue))  // Motor on off
                        {
                            if(direction_angle <= 275 && direction_angle >= 265) //arrive target
                            {
                                motor_vel.angular.x = 0.0;
                                motor_vel.angular.z = 0.0;
                                motor_vel_pub_.publish(motor_vel);
                                ROS_INFO("GOAL");
                                ROS_INFO("distnace : %d", frontlaserdata);

                                if(frontlaserdata < 33)        //near
                                    distanceflag = 1;
                                else if(frontlaserdata > 37)   //far
                                    distanceflag = 0;
                                reachflag = 1;
                            }
                            else if(direction_angle > 275 || direction_angle < 265)
                            {
                                motor_vel.angular.z = ((270 - direction_angle) / 20.0);
                                motor_vel_pub_.publish(motor_vel);
                            }
                        }
                    }
                }
                else if(reachflag == 1)
                {
                    if(distanceflag == 1) //near
                    {
                        if(frontlaserdata >= 35)
                        {
                            motor_vel.linear.x = 0.0;
                            motor_vel_pub_.publish(motor_vel);
                            reachflag = 2;
                            motiontime = sec;
                            ROS_INFO("Distance 1OK");
                        }
                        else
                        {
                            motor_vel.linear.x = -0.08;
                            motor_vel_pub_.publish(motor_vel);
                        }
                    }
                    else // far
                    {
                        if(frontlaserdata <= 35)
                        {
                            motor_vel.linear.x = 0.0;
                            motor_vel_pub_.publish(motor_vel);
                            reachflag = 2;
                            motiontime = sec;
                            ROS_INFO("Distance 2OK");
                        }
                        else
                        {
                            motor_vel.linear.x = 0.08;
                            motor_vel_pub_.publish(motor_vel);
                        }
                    }
                }
                else if(reachflag == 2 && motionflag == 0)
                {
                    if(direction_angle >= 355.0 || direction_angle <= 5.0)
                    {
                        if(sec - motiontime < 2)
                        {
                            motor_vel.linear.x = 0.22;
                            motor_vel.angular.z = 0.0;
                            motor_vel_pub_.publish(motor_vel);
                        }
                        else if(sec - motiontime >= 2)
                        {
                            if(motionflag == 0)
                            {
                                motor_vel.linear.x = 0.0;
                                motor_vel.angular.z = 0.0;
                                motor_vel_pub_.publish(motor_vel);
                                motionflag = 1;
                            }
                        }

                    }
                    else if(direction_angle < 355.0 && direction_angle > 5.0)
                    {
                        if(direction_angle > 0.0 && direction_angle < 90.0)
                        {
                            motor_vel.angular.z = 0.0;
                            motor_vel_pub_.publish(motor_vel);
                        }
                        else
                        {
                            motor_vel.angular.z = ((360 - direction_angle) / 20.0);
                            motor_vel_pub_.publish(motor_vel);
                        }
                    }

                }
                if(motionflag == 1)
                    tunnel_escpae_pub.publish(reachflag);
            }
        }
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
void locationMsgCallback(const geometry_msgs::PolygonStamped &location)
{
    double x_sum = location.polygon.points[0].x + location.polygon.points[1].x + location.polygon.points[2].x + location.polygon.points[3].x;
    double y_sum = location.polygon.points[0].y + location.polygon.points[1].y + location.polygon.points[2].y + location.polygon.points[3].y;
    x_final = x_sum / 4.0;
    y_final = y_sum / 4.0;
    current_location = (x_final - XTARGET)*(x_final - XTARGET) + (y_final - YTARGET)*(y_final - YTARGET); //by circle equation
}
void secCallback(const ros::TimerEvent&)
{
    if(tunnelState == 1)
    {
        sec++;
    }
    x_delta = x_before - x_final;
    y_delta = y_before - y_final;

    x_before = x_final;
    y_before = y_final;
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
void masterDataCallback(const std_msgs::Bool::ConstPtr &master_data)
{
    tunnelState = master_data->data;
}
