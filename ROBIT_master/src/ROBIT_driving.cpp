#include "../include/ROBIT_master/ROBIT_driving.hpp"

#define SAFETY_MODE 0
using namespace std;
using namespace ROBIT_master;
int ROBIT_driving::green_light = 0;

void ROBIT_driving::update_parameter(const ROBIT_vision2::vision_msg2::ConstPtr& vision_data, bool button_clicked)
{
    r_data.l_diff_pixel = vision_data->l_diff_pixel;
    r_data.r_diff_pixel = vision_data->r_diff_pixel;
    r_data.l_line_info = vision_data->l_line_info;
    r_data.r_line_info = vision_data->r_line_info;
    r_data.l_angle = vision_data->l_angle;
    r_data.r_angle = vision_data->r_angle;
    r_data.parking_info = vision_data->parking_info;
    r_data.parking_detect = vision_data->parking_detect;
    r_data.gatebar_detect = vision_data->gatebar_detect;
    r_data.traffic = vision_data->traffic;
    r_data.cross_detect = vision_data->cross_detect;
    r_data.cross_pixel = vision_data->cross_pixel;
    r_data.cross_info = vision_data->cross_info;
    r_data.construct_detect = vision_data->construct_detect;
    r_data.construct_info = vision_data->construct_info;
    button_click = button_clicked;
}

void ROBIT_driving::Go()
{
    analyze_situation();
    switch (situation)
    {
    case none:
        linetracing(Linear_MAX);
        break;
    case parking:
        parking_motion();
        break;
    case gatebar:
        ROBIT_driving::tunnel_escape = 0;
        set_speed(0.0, 0.0);

        break;
    case traffic:
        set_speed(0.0, 0.0);
        break;
    case cross:
        cross_motion();
        break;
    case construct:
        construct_motion();
    default:
        break;
    }
    if(!button_click)
        set_speed(0.0, 0.0);
    if(r_data.traffic == 0)
        set_speed(0.0, 0.0);
    else
        green_light = 1;
#ifdef SAFETY_MODE
    {
        // safety mode
    }
#else
    {
        // speed mode
    }
#endif
}

void ROBIT_driving::construct_motion()
{
    switch(r_data.construct_info)
    {
    case slow:
        linetracing(Linear_MAX);
        break;
    case left_c1:
        set_speed(0.0, (135 - direction_angle)/30.0);  //135
        break;
    case front_c1:
        if(direction_angle < 110)
            set_speed(0.22, -1.2);
        else
            set_speed(0.22, -0.65); //0.7
        break;
    }
}

void ROBIT_driving::cross_motion()
{
    switch(r_data.cross_info)
    {
    case turn_l1:
        if(direction_angle < 230)
            set_speed(0.22, 1.65);
        else
            set_speed(0.22, 1.3);
        break;
    case turn_r1:
        if(direction_angle > 140)
            set_speed(0.22, -1.65);
        else
            set_speed(0.22, -1.33);
        break;
    case drive:
        linetracing(Linear_MAX);
        break;
    case turn_l2:
        set_speed(0.22, 1.4);
        break;
    case turn_r2:
        set_speed(0.22, -1.45);
        break;
    }
}

void ROBIT_driving::parking_motion()
{
    switch(r_data.parking_info)
    {
    case mark:
        linetracing(Linear_MAX);
        break;
    case line1:
        linetracing(Linear_MAX);
        break;
    case go:
        linetracing(Linear_MAX);
        break;
    case turn1:
        set_speed(0.22, 1.5);
        break;
    case line2:
        linetracing(Linear_MAX);
        break;
    case turn2r:
        set_speed(-0.22, 1.85);
        break;
    case turn2l:
        set_speed(-0.22, -1.85);
        break;
    case front:
        set_speed(0.22, 0.0);
        break;
    case back:
        set_speed(-0.22, 0.0);
        break;
    case detect:
        linetracing(Linear_MAX);
        break;
    case turn3r:
        set_speed(0.22, 1.75);
        break;
    case turn3l:
        break;
    case line3:
        linetracing(Linear_MAX);
        break;
    case front2:
        set_speed(0.22, 0.0);
        break;
    case turn4:
        set_speed(0.22, 1.8);
        break;
    }
}

void ROBIT_driving::linetracing(double speed)
{
    int pixel_gap = 0;
    linear_x = speed;
    if(r_data.r_line_info && r_data.l_line_info)
    {
        pixel_gap = Center - (int)((r_data.l_diff_pixel + r_data.r_diff_pixel) / 2);
        angular_z= (double)pixel_gap * Linear_Gain;
    }
    else if(r_data.r_line_info && !r_data.l_line_info)
    {
        if(r_data.r_angle >= 83 && r_data.r_angle < 92)
            angular_z  = (double)(295 - r_data.r_diff_pixel) * Linear_Gain;
        else if(r_data.r_angle >= 92 || r_data.r_diff_pixel < Curve_R)
            angular_z = (double)(295 - r_data.r_diff_pixel) * Curve_Gain;
        else if(r_data.cross_detect && r_data.r_angle < 80 && r_data.r_angle > 60)
            angular_z = (double)(90 - r_data.r_angle) * (Linear_Gain - 0.01);
    }
    else if(!r_data.r_line_info && r_data.l_line_info)
    {
        if(r_data.l_angle >= 85 && r_data.l_angle < 92)
            angular_z = (double)(r_data.l_diff_pixel - 25) * Linear_Gain * -1; //void ROBIT_driving::slow_linetracing()
        else if(r_data.l_angle < 85 || r_data.l_diff_pixel > Curve_L)
            angular_z = (double)(r_data.l_diff_pixel - 25) * Curve_Gain * -1;
    }
    set_speed(linear_x, angular_z);
}

void ROBIT_driving::analyze_situation()
{
    if(r_data.parking_detect)
        situation = parking;
    else if(r_data.gatebar_detect)
    {
        situation = gatebar;
        confirm_gatebar = true;
    }
    else if(r_data.cross_detect)
        situation = cross;
    else if(r_data.construct_detect)
        situation = construct;
    else
        situation = none;
}

void ROBIT_driving::set_speed(double linear, double angular)
{
    motor_value.linear.x = linear;
    motor_value.angular.z = angular;
}
