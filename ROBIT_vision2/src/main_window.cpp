/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
#include "../include/ROBIT_vision2//main_window.hpp"

int vision_line::yellow_value[6] = {0, };
int vision_line::Canny_value[2] = {0, };
int vision_line::white_value[6] = {0, };
int vision::parking_value[6] = {0, };
int vision::gatebar_value[6] = {0, };
int vision::traffic_value[6] = {0, };
bool vision_record::button_clicked = false;
bool vision::timer_ctrl = false;
bool vision::process_flag[3] = {0, };
int vision::timer_cnt = 0;
int vision::traffic_action = 0;


namespace ROBIT_vision2 {

using namespace Qt;
using namespace std;
extern bool isRecved;

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
    : QMainWindow(parent) , click_flag(1), record_time(0)
    , qnode(argc,argv)
{
    ui.setupUi(this);
    qnode.init();
    QTimer *my_timer =  new QTimer(this);
    QTimer *record_timer =  new QTimer(this);
    get_data();
    update_label();
    qRegisterMetaType<cv::Mat>("cv::Mat");
    QObject::connect(my_timer, SIGNAL(timeout()), this, SLOT(QTimer_callback()));
    QObject::connect(record_timer, SIGNAL(timeout()), this, SLOT(recordtimer()));
    QObject::connect(&qnode, SIGNAL(recvImg()), this, SLOT(updateImg()));
    QObject::connect(&qnode.robit_vision, SIGNAL(perspective_callback(cv::Mat)),this, SLOT(updatePerspectiveImg(cv::Mat)));
    QObject::connect(&qnode.robit_vision, SIGNAL(mission_callback(void)),this, SLOT(updateMissionData(void)));
    QObject::connect(&qnode.robit_vision, SIGNAL(white_line_callback(cv::Mat)),this, SLOT(updateWhiteEdgeImg(cv::Mat)));
    QObject::connect(&qnode.robit_vision, SIGNAL(yellow_line_callback(cv::Mat)),this, SLOT(updateYellowEdgeImg(cv::Mat)));
    QObject::connect(&qnode.robit_vision, SIGNAL(parking_sign_callback(cv::Mat)),this, SLOT(updateparkingImg(cv::Mat)));
    QObject::connect(&qnode.robit_vision, SIGNAL(gatebar_callback(cv::Mat)),this, SLOT(updategatebarImg(cv::Mat)));
    QObject::connect(&qnode.robit_vision, SIGNAL(traffic_callback(cv::Mat)),this, SLOT(updatetrafficImg(cv::Mat)));
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
    my_timer->start(50);
    record_timer->start(1000);
    if(qnode.img_qnode != NULL) qnode.img_qnode = NULL;
}

void MainWindow::updateMissionData(void)
{
    if(vision::process_flag[0])
        ui.checkBox_Cross->setChecked(1);
    if(vision::process_flag[1])
        ui.checkBox_Construct->setChecked(1);
    if(vision::process_flag[2])
        ui.checkBox_Parking->setChecked(1);
}

void MainWindow::updateImg(void)
{
    QImage raw_image((const unsigned char*)(qnode.img_qnode->data), qnode.img_qnode->cols, qnode.img_qnode->rows, QImage::Format_RGB888);
    ui.label_raw_image->setPixmap(QPixmap::fromImage(raw_image.rgbSwapped()));
    QImage raw_image2((const unsigned char*)(qnode.img_qnode->data), qnode.img_qnode->cols, qnode.img_qnode->rows, QImage::Format_RGB888);
    ui.label_raw_image2->setPixmap(QPixmap::fromImage(raw_image2.rgbSwapped()));
    delete qnode.img_qnode;
    if(qnode.img_qnode!=NULL)qnode.img_qnode=NULL;
    isRecved = false;
}

void MainWindow::updatePerspectiveImg(cv::Mat perspective_img)
{
    QImage perspective_image((const unsigned char*)(perspective_img.data), perspective_img.cols, perspective_img.rows,  QImage::Format_RGB888);
    ui.label_perspective->setPixmap(QPixmap::fromImage(perspective_image.rgbSwapped()));
}

void MainWindow::updateWhiteEdgeImg(cv::Mat white_line_img)
{
    QImage White_line_image((const unsigned char*)(white_line_img.data), white_line_img.cols, white_line_img.rows,  QImage::Format_Indexed8);
    ui.label_white_line->setPixmap(QPixmap::fromImage(White_line_image));
}

void MainWindow::updateYellowEdgeImg(cv::Mat yellow_line_img)
{
    QImage Yellow_line_image((const unsigned char*)(yellow_line_img.data), yellow_line_img.cols, yellow_line_img.rows,  QImage::Format_RGB888);
    ui.label_yellow_line->setPixmap(QPixmap::fromImage(Yellow_line_image.rgbSwapped()));
}

void MainWindow::updateparkingImg(cv::Mat parking_img)
{
    QImage Parking_image((const unsigned char*)(parking_img.data), parking_img.cols, parking_img.rows,  QImage::Format_Indexed8);
    ui.label_parking->setPixmap(QPixmap::fromImage(Parking_image));
}

void MainWindow::updategatebarImg(cv::Mat gatebar_img)
{
    QImage Gatebar_image((const unsigned char*)(gatebar_img.data), gatebar_img.cols, gatebar_img.rows,  QImage::Format_Indexed8);
    ui.label_gatebar->setPixmap(QPixmap::fromImage(Gatebar_image));
}

void MainWindow::updatetrafficImg(cv::Mat traffic_img)
{
    QImage Traffic_image((const unsigned char*)(traffic_img.data), traffic_img.cols, traffic_img.rows, traffic_img.step,  QImage::Format_Indexed8);
    ui.label_traffic->setPixmap(QPixmap::fromImage(Traffic_image));
}

void MainWindow::on_upper_H_valueChanged(int value)
{
    if(click_flag == 1)
        vision_line::yellow_value[0] = value;
    else if(click_flag == 2)
        vision_line::white_value[0] = value;
    else if(click_flag == 3)
        vision::gatebar_value[0] = value;
    else if(click_flag == 4)
        vision::traffic_value[0] = value;
    ui.label_upper_H->setText(QString::number(value));
}

void MainWindow::on_upper_S_valueChanged(int value)
{
    if(click_flag == 1)
        vision_line::yellow_value[1] = value;
    else if(click_flag == 2)
        vision_line::white_value[1] = value;
    else if(click_flag == 3)
        vision::gatebar_value[1] = value;
    else if(click_flag == 4)
        vision::traffic_value[1] = value;
    ui.label_upper_S->setText(QString::number(value));
}

void MainWindow::on_upper_V_valueChanged(int value)
{
    if(click_flag == 1)
        vision_line::yellow_value[2] = value;
    else if(click_flag == 2)
        vision_line::white_value[2] = value;
    else if(click_flag == 3)
        vision::gatebar_value[2] = value;
    else if(click_flag == 4)
        vision::traffic_value[2] = value;
    ui.label_upper_V->setText(QString::number(value));
}

void MainWindow::on_lower_H_valueChanged(int value)
{
    if(click_flag == 1)
        vision_line::yellow_value[3] = value;
    else if(click_flag == 2)
        vision_line::white_value[3] = value;
    else if(click_flag == 3)
        vision::gatebar_value[3] = value;
    else if(click_flag == 4)
        vision::traffic_value[3] = value;
    ui.label_lower_H->setText(QString::number(value));
}

void MainWindow::on_lower_S_valueChanged(int value)
{
    if(click_flag == 1)
        vision_line::yellow_value[4] = value;
    else if(click_flag == 2)
        vision_line::white_value[4] = value;
    else if(click_flag == 3)
        vision::gatebar_value[4] = value;
    else if(click_flag == 4)
        vision::traffic_value[4] = value;
    ui.label_lower_S->setText(QString::number(value));
}

void MainWindow::on_lower_V_valueChanged(int value)
{
    if(click_flag == 1)
        vision_line::yellow_value[5] = value;
    else if(click_flag == 2)
        vision_line::white_value[5] = value;
    else if(click_flag == 3)
        vision::gatebar_value[5] = value;
    else if(click_flag == 3)
        vision::traffic_value[5] = value;
    ui.label_lower_V->setText(QString::number(value));
}

void MainWindow::on_Canny_value_valueChanged(int value)
{  
    vision_line::Canny_value[0] = value;
    ui.label_Canny_value->setText(QString::number(vision_line::Canny_value[0]));
}

void MainWindow::on_Canny_ratio_valueChanged(int value)
{
    vision_line::Canny_value[1] = value;
    ui.label_Canny_ratio->setText(QString::number(vision_line::Canny_value[1]));
}

void MainWindow::on_Save_clicked()
{
    ofstream save_parameter("/home/robit/catkin_ws/src/ROBIT_vision2/txt_data/prameter_data.txt");

    if(save_parameter.is_open())
    {
        for(int i = 0 ; i < 6 ; i++)
           save_parameter << vision_line::yellow_value[i] <<endl;
        for(int i = 0 ; i < 2 ; i++)
           save_parameter << vision_line::Canny_value[i] <<endl;
        for(int i = 0 ; i < 6 ; i++)
           save_parameter << vision_line::white_value[i] <<endl;
        for(int i = 0 ; i < 6 ; i++)
           save_parameter << vision::gatebar_value[i] <<endl;
        for(int i = 0 ; i < 6 ; i++)
           save_parameter << vision::traffic_value[i] <<endl;
    }
    save_parameter.close();
}

void MainWindow::on_Record_clicked()
{
    if(!vision_record::button_clicked)
        vision_record::button_clicked = true;
    else
        vision_record::button_clicked = false;
}

void MainWindow::get_data()
{
    ifstream get_parameter("/home/robit/catkin_ws/src/ROBIT_vision2/txt_data/prameter_data.txt");

    if(get_parameter.is_open())
    {
        for(int i = 0 ; i < 6 ; i++)
           get_parameter >> vision_line::yellow_value[i];
        for(int i = 0 ; i < 2 ; i++)
           get_parameter >> vision_line::Canny_value[i];
        for(int i = 0 ; i < 6 ; i++)
           get_parameter >> vision_line::white_value[i];
        for(int i = 0 ; i < 6 ; i++)
           get_parameter >> vision::gatebar_value[i];
        for(int i = 0 ; i < 6 ; i++)
           get_parameter >> vision::traffic_value[i];
    }
}

void MainWindow::on_Yellow_check_clicked()
{
    ui.Parking_check->setChecked(false);
    ui.Gatebar_check->setChecked(false);
    ui.traffic->setChecked(false);
    click_flag = 1;
    update_label();
}

void MainWindow::on_Parking_check_clicked()
{
    ui.Yellow_check->setChecked(false);
    ui.Gatebar_check->setChecked(false);
    ui.traffic->setChecked(false);
    click_flag = 2;
    update_label();
}

void MainWindow::on_Gatebar_check_clicked()
{
    ui.Yellow_check->setChecked(false);
    ui.Parking_check->setChecked(false);
    ui.traffic->setChecked(false);
    click_flag = 3;
    update_label();
}

void MainWindow::on_traffic_clicked()
{
    ui.Gatebar_check->setChecked(false);
    ui.Yellow_check->setChecked(false);
    ui.Parking_check->setChecked(false);
    click_flag = 4;
    update_label();
}

void MainWindow::on_Reset_clicked()
{
    vision::traffic_action = 1;
}

void MainWindow::update_label()
{
    int paste_array[6] = {0, };
    if(click_flag == 1){
        for(int i = 0 ; i < 6 ; i++)
            paste_array[i] = vision_line::yellow_value[i];
    }
    else if(click_flag == 2){
        for(int i = 0 ; i < 6 ; i++)
            paste_array[i] = vision_line::white_value[i];
    }
    else if(click_flag == 3){
        for(int i = 0 ; i < 6 ; i++)
            paste_array[i] = vision::gatebar_value[i];
    }
    else if(click_flag == 4){
        for(int i = 0 ; i < 6 ; i++)
            paste_array[i] = vision::traffic_value[i];
    }
    ui.label_upper_H->setText(QString::number(paste_array[0]));
    ui.upper_H->setValue(paste_array[0]);
    ui.label_upper_S->setText(QString::number(paste_array[1]));
    ui.upper_S->setValue(paste_array[1]);
    ui.label_upper_V->setText(QString::number(paste_array[2]));
    ui.upper_V->setValue(paste_array[2]);
    ui.label_lower_H->setText(QString::number(paste_array[3]));
    ui.lower_H->setValue(paste_array[3]);
    ui.label_lower_S->setText(QString::number(paste_array[4]));
    ui.lower_S->setValue(paste_array[4]);
    ui.label_lower_V->setText(QString::number(paste_array[5]));
    ui.lower_V->setValue(paste_array[5]);
    ui.label_Canny_value->setText(QString::number(vision_line::Canny_value[0]));
    ui.Canny_value->setValue(vision_line::Canny_value[0]);
    ui.label_Canny_ratio->setText(QString::number(vision_line::Canny_value[1]));
    ui.Canny_ratio->setValue(vision_line::Canny_value[1]);
}

void MainWindow::QTimer_callback()
{
    if(vision::timer_ctrl)
        vision::timer_cnt++;
    else
        vision::timer_cnt = 0;
    ui.lcdNumber_2->display(QString::number(vision::timer_cnt));
    ui.lcdNumber_psd1->display(QString::number(vision::sensor_data[0]));
    ui.lcdNumber_psd2->display(QString::number(vision::sensor_data[1]));
    ui.lcdNumber_psd3->display(QString::number(vision::sensor_data[2]));
}

void MainWindow::recordtimer()
{
    if(vision_record::button_clicked)
    {
        record_time++;
    }
    ui.lcdNumber->display(QString::number(record_time));
}

MainWindow::~MainWindow() {}
}  // namespace ROBIT_vision2
