/**
 * @file /include/tledak_vision2/main_window.hpp
 *
 * @brief Qt based gui for tledak_vision2.
 *
 * @date November 2010
 **/
#ifndef ROBIT_vision2_MAIN_WINDOW_H
#define ROBIT_vision2_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QtGui/QMainWindow>
#include <iostream>
#include <fstream>
#include <QMainWindow>
#include <QMessageBox>
#include "ui_main_window.h"
#include "qnode.hpp"
#include <QTimer>

/*****************************************************************************
** Namespace
*****************************************************************************/

using namespace cv;
namespace ROBIT_vision2 {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    MainWindow(int argc, char** argv, QWidget *parent = 0);
    ~MainWindow();

    void get_data();

    void update_label();

    int click_flag, record_time;

public Q_SLOTS:
    void updateMissionData(void);

    void updateImg(void);

    void recordtimer(void);

    void updatePerspectiveImg(cv::Mat perspective_img);

    void updateWhiteEdgeImg(cv::Mat white_line_img);

    void updateYellowEdgeImg(cv::Mat yellow_line_img);

    void updateparkingImg(cv::Mat parking_img);

    void updategatebarImg(cv::Mat gatebar_img);

    void updatetrafficImg(cv::Mat traffic_img);

    void on_Canny_value_valueChanged(int value);

    void on_Canny_ratio_valueChanged(int value);

    void on_upper_H_valueChanged(int value);

    void on_upper_S_valueChanged(int value);

    void on_upper_V_valueChanged(int value);

    void on_lower_H_valueChanged(int value);

    void on_lower_S_valueChanged(int value);

    void on_lower_V_valueChanged(int value);

    void on_Save_clicked();

    void on_Yellow_check_clicked();

    void on_Parking_check_clicked();

    void on_Gatebar_check_clicked();

    void on_Record_clicked();

    void on_traffic_clicked();

    void on_Reset_clicked();

    void QTimer_callback();

private:
    Ui::MainWindowDesign ui;
    QNode qnode;
};

}  // namespace ROBIT_vision2

#endif // ROBIT_vision2_MAIN_WINDOW_H
