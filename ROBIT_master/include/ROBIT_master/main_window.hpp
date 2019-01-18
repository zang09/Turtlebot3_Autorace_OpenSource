/**
 * @file /include/tledak_gui/main_window.hpp
 *
 * @brief Qt based gui for tledak_gui.
 *
 * @date November 2010
 **/
#ifndef ROBIT_master_MAIN_WINDOW_H
#define ROBIT_master_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include <QMainWindow>
#include <QtGui>
#include <QMessageBox>
#include "ui_main_window.h"
#include "qnode.hpp"
#include <time.h>

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace ROBIT_master {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
    ~MainWindow(){}

public Q_SLOTS:
  void updateData(void);
  void on_pushButton_clicked();

private:
  Ui::MainWindow ui;
	QNode qnode;
    enum{cds = 0, imu, robot_y };
};

}  // namespace ROBIT_gui

#endif // ROBIT_gui_MAIN_WINDOW_H
