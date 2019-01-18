/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <iostream>
#include "../include/ROBIT_master/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/
ROBIT_vision2::vision_msg2 ROBIT_driving::r_data;
namespace ROBIT_master {

using namespace Qt;
using namespace std;
extern bool isRecved;
extern bool button_clicked;
/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
    : QMainWindow(parent)
    ,qnode(argc,argv)
{
    button_clicked = false;
    ui.setupUi(this);
    qnode.init();
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
    QObject::connect(&qnode, SIGNAL(Recv_Data()), this, SLOT(updateData()));
}

void MainWindow::updateData()
{
    ui.label_imu->setText(QString::number(ROBIT_driving::direction_angle));
    ui.label_cds->setText(QString::number(ROBIT_driving::sensor_data[cds]));
    ui.label_linear_x->setText(QString::number(ROBIT_driving::motor_value.linear.x));
    ui.label_angular_z->setText(QString::number(ROBIT_driving::motor_value.angular.z));
    ui.label_tunnel_detect->setText(QString::number(ROBIT_driving::tunnel_escape));
    if(ROBIT_driving::r_data.l_line_info)
    {
        ui.label_left_detect->setText("Detect");
        ui.label_left_pixel->setText(QString::number(ROBIT_driving::r_data.l_diff_pixel));
    }
    else
    {
        ui.label_left_detect->setText("NO POINT");
        ui.label_left_pixel->setText("NO POINT");
    }

    if(ROBIT_driving::green_light == 1) {
        ui.label_traffic_green->setText("Detect");
    }
    else {
        ui.label_traffic_green->setText("Fuck");
    }

    if(ROBIT_driving::r_data.r_line_info)
    {
        ui.label_right_detect->setText("Detect");
        ui.label_right_pixel->setText(QString::number(ROBIT_driving::r_data.r_diff_pixel));
    }
    else
    {
        ui.label_right_detect->setText("NO POINT");
        ui.label_right_pixel->setText("NO POINT");
    }

    if(ROBIT_driving::r_data.gatebar_detect)
        ui.label_gatebar_detect->setText("Detect");
    else
        ui.label_gatebar_detect->setText("NOPE");

    if(ROBIT_driving::r_data.parking_detect)
        ui.label_parking_sign->setText("Detect");
    else
        ui.label_parking_sign->setText("NOPE");

    isRecved = false;
}

void MainWindow::on_pushButton_clicked()
{
    if(button_clicked)
    {
        button_clicked = false;
        ui.pushButton->setText("start");
    }
    else{
        button_clicked = true;
        ui.pushButton->setText("stop");
    }
}

}





