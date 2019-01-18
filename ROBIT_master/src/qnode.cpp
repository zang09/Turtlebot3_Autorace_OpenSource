/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include "../include/ROBIT_master/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/
bool ROBIT_driving::confirm_gatebar = false, ROBIT_driving::state = false;
int ROBIT_driving::sensor_data[3]={0,}, ROBIT_driving::direction_angle = 0, ROBIT_driving::tunnel_escape = 0;
geometry_msgs::Twist ROBIT_driving::motor_value;

namespace ROBIT_master {

bool isRecved = false;
bool button_clicked = false;
using namespace  std;
ROBIT_driving Driving;

/*****************************************************************************
** Implementation
*****************************************************************************/
QNode::QNode(int argc, char** argv ) :
    init_argc(argc),
    init_argv(argv), tunnel_detect(0)
{}

void QNode::TunDataCallback(const std_msgs::UInt32::ConstPtr &tunnel_msg)
{
    ROBIT_driving::tunnel_escape = tunnel_msg->data;
}

void QNode::ImgDataCallback(const ROBIT_vision2::vision_msg2::ConstPtr &vision_msg)
{
    if(vision_msg != NULL && !isRecved)
    {
        isRecved = true;
        Driving.update_parameter(vision_msg, button_clicked);
        Driving.Go();
        if(ROBIT_driving::state == 1)
        {
            //ROS_INFO("2");
            Tun_Data_pub.publish(ROBIT_driving::state);
        }
        else if(ROBIT_driving::state == 0)
        {
            //ROS_INFO("3");
            Mot_Data_pub.publish(ROBIT_driving::motor_value);
        }
        Q_EMIT Recv_Data();
    }
}

void QNode::CdsDataCallback(const std_msgs::UInt32::ConstPtr &cds_msg)
{
    ROBIT_driving::sensor_data[cds] = cds_msg->data;
    if(ROBIT_driving::sensor_data[cds] >= 800) //tunnel
    {
        ROS_INFO("%d",ROBIT_driving::tunnel_escape);
        ROS_INFO("%d",tunnel_detect);
        ROS_INFO("%d",ROBIT_driving::confirm_gatebar);
        ROS_INFO("%d",ROBIT_driving::direction_angle);
        if(ROBIT_driving::tunnel_escape == 2)
        {
            ROBIT_driving::tunnel_escape = 0;
            ROBIT_driving::state = 0;
        }
        else
        {
            if(!tunnel_detect /*&& ROBIT_driving::confirm_gatebar*/ && (ROBIT_driving::direction_angle <= 290 && ROBIT_driving::direction_angle >= 250))
            {
                ROBIT_driving::state = 1;
                tunnel_detect = true;
            }
        }
    }
    else
    {
        if(ROBIT_driving::tunnel_escape == 2)
        {
            ROBIT_driving::tunnel_escape = 0;
            ROBIT_driving::state = 0;
        }
    }
}
void QNode::SwDataCallback(const std_msgs::Bool::ConstPtr &sw_data)
{
    if(ROBIT_driving::state == 1 && sw_data->data == true)
        ROBIT_driving::state = 0;
}
void QNode::imuMsgCallback(const sensor_msgs::Imu::ConstPtr &imu)
{
    double imu_x = imu->orientation.x;
    double imu_y = imu->orientation.y;
    double imu_z = imu->orientation.z;
    double imu_w = imu->orientation.w;

    double yaw = atan2(2*imu_x*imu_y + 2*imu_w*imu_z, imu_w*imu_w + imu_x*imu_x - imu_y*imu_y - imu_z*imu_z);
    double yaw_angle = yaw * RAD2DEG;

    if(yaw_angle > 0.0 && yaw_angle <= 180.0)
      ROBIT_driving::direction_angle = yaw_angle;
    else if(yaw_angle <= 0.0 && yaw_angle > -180.0)
      ROBIT_driving::direction_angle = yaw_angle + 360.0;
}

bool QNode::init()
{
    ros::init(init_argc,init_argv,"ROBIT_master");
    if ( ! ros::master::check() )
        return false;
    ros::start();
    ros::NodeHandle nh;

    Img_Data_sub = nh.subscribe("turtle_vision",1,&QNode::ImgDataCallback,this);
    Cds_Data_sub = nh.subscribe("cds_data",1,&QNode::CdsDataCallback,this);
    Tun_Data_sub = nh.subscribe("tunnel_escape",1,&QNode::TunDataCallback,this);
    Tun_Data_pub = nh.advertise<std_msgs::Bool>("tunnel_state",1);
    Mot_Data_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel",1);
    Imu_Data_sub = nh.subscribe("/imu",10,&QNode::imuMsgCallback,this);
    Sw_Data_sub = nh.subscribe("switch_data1",1,&QNode::SwDataCallback,this);
    start();
    return true;
}

void QNode::run()
{
    ros::Rate loop_rate(100);
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

}
