#ifndef ROBIT_DRIVING_H
#define ROBIT_DRIVING_H

#include "qnode.hpp"
#include "iostream"
#include "../../devel/include/ROBIT_vision2/vision_msg2.h"
#include "../../devel/include/ROBIT_master/motion_cplt.h"
#include <geometry_msgs/Twist.h>

#define Linear_MAX 0.22
#define Linear_SLOW 0.14
#define Linear_Gain 0.0115
#define Curve_Gain 0.01


#define Slow_Linear_Gain 0.006
#define Slow_Curve_Gain 0.005
#define Center 160
#define Curve_R 242
#define Curve_L 55
#define IMU_gain 30.0

class ROBIT_driving
{
public:
    ROBIT_driving():button_click(0), motion_complete(0){}
    void Go();
    void update_parameter(const ROBIT_vision2::vision_msg2::ConstPtr& vision_data, bool button_clicked);
    bool motion_complete;
    static ROBIT_vision2::vision_msg2 r_data;
    static geometry_msgs::Twist motor_value;
    static bool sw_data, confirm_gatebar, state;
    static int sensor_data[3], direction_angle, tunnel_escape, green_light;
private:
    void linetracing(double speed);
    void slow_linetracing();
    void parking_motion();
    void cross_motion();
    void set_speed(double linear, double angular);
    void analyze_situation();
    void construct_motion();

    double linear_x, angular_z;
    bool button_click;
    int situation, center;

    enum {cds = 0, imu, robot_y};
    enum {slow, left_c1, front_c1};
    enum {mark, line1, go, turn1, line2, detect, turn2l, turn2r, front, back, turn3l, turn3r, line3, front2, turn4};
    enum {sign, wait, turn_l1, turn_r1, drive, turn_l2, turn_r2};
    enum {none = 0, parking, gatebar, traffic, cross, construct};
};

#endif // ROBIT_DRIVING_H
