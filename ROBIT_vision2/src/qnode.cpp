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

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/ROBIT_vision2/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/
using namespace std;
int vision::sensor_data[3]={0,};
int vision::direction_angle = 0;
namespace ROBIT_vision2 {

/*****************************************************************************
** Implementation
*****************************************************************************/

bool isRecved = false;

QNode::QNode(int argc, char** argv ) :
    init_argc(argc),
    init_argv(argv)
{}

QNode::~QNode() {}

void QNode::imageCallback(const sensor_msgs::ImageConstPtr &msg_img)
{
    if(img_qnode == NULL && !isRecved)
    {
        img_qnode = new cv::Mat(cv_bridge::toCvCopy(msg_img, sensor_msgs::image_encodings::BGR8)->image);
        if(img_qnode != NULL)
        {
            robit_vision.START(*img_qnode);
            Record.prepare_Record();
            Record.do_Record(robit_vision.Raw_image);
            vision_pub.publish(robit_vision.Vision_msg);
            isRecved = true;
            Q_EMIT recvImg();
        }
    }
}

bool QNode::init() {
    ros::init(init_argc,init_argv,"tledak_vision2");
    if ( ! ros::master::check() )
        return false;    
    ros::start();
    ros::NodeHandle n;
    image_transport::ImageTransport it(n);

    imu_sub = n.subscribe("/imu",10,&QNode::imuMsgCallback,this);
    image_sub = it.subscribe("/usb_cam/image_raw",1,&QNode::imageCallback,this);
    Psd_Data_sub = n.subscribe("psd_data1",1,&QNode::PsdDataCallback,this);
    Psd_Data_sub2 = n.subscribe("psd_data2",1,&QNode::Psd2DataCallback,this);
    Psd_Data_sub3 = n.subscribe("psd_data3",1,&QNode::Psd3DataCallback,this);
    vision_pub = n.advertise<ROBIT_vision2::vision_msg2>("turtle_vision", 1);
    start();
    return true;
}

void QNode::PsdDataCallback(const std_msgs::UInt32::ConstPtr &psd_data1)
{
    vision::sensor_data[psd] = psd_data1->data;
}

void QNode::Psd2DataCallback(const std_msgs::UInt32::ConstPtr &psd_data2)
{
    vision::sensor_data[psd2] = psd_data2->data;
}

void QNode::Psd3DataCallback(const std_msgs::UInt32::ConstPtr &psd_data3)
{
    vision::sensor_data[psd3] = psd_data3->data;
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
      vision::direction_angle = yaw_angle;
    else if(yaw_angle <= 0.0 && yaw_angle > -180.0)
      vision::direction_angle = yaw_angle + 360.0;

}
void QNode::run() {
    ros::Rate loop_rate(300);
    while ( ros::ok() )
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

}  // namespace ROBIT_vision2
