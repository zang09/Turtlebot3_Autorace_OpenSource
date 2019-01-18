/**
 * @file /include/tledak_gui/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ROBIT_master_QNODE_HPP_
#define ROBIT_master_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/
#define RAD2DEG (180.0 / M_PI)
// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <string>
#include "iostream"
#include <QThread>
#include <ros/ros.h>
#include <ros/network.h>
#include "ROBIT_driving.hpp"
#include <std_msgs/Bool.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Bool.h>
#include "../../devel/include/ROBIT_vision2/vision_msg2.h"
#endif
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ROBIT_master {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT

public:
    ROBIT_vision2::vision_msg2 vision_data;
    bool init();
    int l_motorspeed, r_motorspeed;
    bool tunnel_detect;
    bool sw_data;
    QNode(int argc, char** argv );
    virtual ~QNode(){}
    void run();

Q_SIGNALS:
    void Recv_Data();
    void rosShutdown();

private:
    ros::Subscriber Img_Data_sub;
    ros::Subscriber Imu_Data_sub;
    ros::Publisher  Mot_Data_pub;
    ros::Publisher  Tun_Data_pub;
    ros::Subscriber Tun_Data_sub;
    ros::Subscriber Sw_Data_sub;
    ros::Subscriber Cds_Data_sub;

    void SwDataCallback(const std_msgs::Bool::ConstPtr &sw_data);
    void ImgDataCallback(const ROBIT_vision2::vision_msg2::ConstPtr &vision_msg);
    void TunDataCallback(const std_msgs::UInt32::ConstPtr &tunnel_msg);
    void CdsDataCallback(const std_msgs::UInt32::ConstPtr &cds_msg);
    void imuMsgCallback(const sensor_msgs::Imu::ConstPtr &imu);
    int init_argc;
    char** init_argv;

    enum{cds = 0, imu, psd, robot_y };
};

}  // namespace ROBIT_gui

#endif /* ROBIT_gui_QNODE_HPP_ */
