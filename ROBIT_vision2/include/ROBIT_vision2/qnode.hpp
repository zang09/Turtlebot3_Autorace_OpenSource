/**
 * @file /include/tledak_vision2/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/

#ifndef ROBIT_vision2_QNODE_HPP_
#define ROBIT_vision2_QNODE_HPP_

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <QThread>
#include <opencv2/opencv.hpp>
#include "opencv2/features2d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/core/core.hpp"
#include <std_msgs/UInt32.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include "vision.hpp"
#include "iostream"
#include <string>
#include <sensor_msgs/Imu.h>
#endif
#define RAD2DEG (180.0 / M_PI)
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ROBIT_vision2 {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
    QNode(int argc, char** argv );
    virtual ~QNode();
    bool init();
    void run();
    cv::Mat *img_qnode;
    vision robit_vision;

Q_SIGNALS:
    void recvImg();
    void rosShutdown();

private:
    vision_record Record;
    int init_argc;
    char** init_argv;
    void imageCallback(const sensor_msgs::ImageConstPtr& msg_img);
    void PsdDataCallback(const std_msgs::UInt32::ConstPtr &psd_data1);
    void Psd2DataCallback(const std_msgs::UInt32::ConstPtr &psd_data2);
    void Psd3DataCallback(const std_msgs::UInt32::ConstPtr &psd_data2);
    void imuMsgCallback(const sensor_msgs::Imu::ConstPtr &imu);
    image_transport::Subscriber image_sub;
    ros::Subscriber Psd_Data_sub, Psd_Data_sub2, Psd_Data_sub3, imu_sub;
    ros::Publisher vision_pub;
    enum{psd, psd2, psd3};
};

}  // namespace ROBIT_vision2

#endif /* ROBIT_vision2_QNODE_HPP_ */
