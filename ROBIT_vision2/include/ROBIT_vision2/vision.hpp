#ifndef VISION_H
#define VISION_H

#endif // VISION_H
#include <QObject>
#include <opencv2/opencv.hpp>
#include "opencv2/features2d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/core/core.hpp"
#include <cv_bridge/cv_bridge.h>
#include "../include/ROBIT_vision2/vision_msg2.h"
#include "darknetdetector.hpp"

using namespace cv;
using namespace std;
using namespace cv::xfeatures2d;

#define RAW_X 320
#define RAW_Y 240
#define Y_POS_GAP 120

#define LINE_MAXIMUM_WIDTH 55
#define LINE_MINIMUM_WIDTH 10
#define LINE_MAXIMUM_WIDTH2 40

#define REC_AREA_RATE_MIN           0.6
#define REC_AREA_RATE_MAX           0.95
#define REC_AREA_MIN                2500
#define REC_AREA_MAX                2900
#define REC_CHECK                   18
#define PARKING_MATCHES_SIZE_MIN    15
#define PARKING_MATCHES_SIZE_MAX    100

#define Red cv::Scalar(0,0,255)
#define Blue cv::Scalar(255,0,0)
#define Green cv::Scalar(0,255,0)
#define Yellow cv::Scalar(0,255,255)
#define CPL cv::Scalar(255,153,153)
#define Purple cv::Scalar(153,0,102)
#define Skyblue cv::Scalar(255,153,0)

class vision : public QObject{
    Q_OBJECT

public:
     explicit vision(QObject *parent = 0) : QObject(parent), gatebar_count(0), parking_count(0), gatebar_detect(0), line_count(0)
    , parking_detect(0), cross_detect(0), construct_detect1(0), construct_detect2(0)
    {
        detector_init("/home/robit/darknet/yolov3-tiny_obj.cfg",
                      "/home/robit/darknet/yolov3-tiny_obj_10000.weights");
        for(int i = 0 ; i < 3 ; i++){
            traffic_condition[i] = false;
            sign_count[i] = 0;
        }
        for(int i = 0 ; i < 4 ; i++){
            sign_count[i] = 0;
            sign_condition[i] = false;
        }
    }

    ~vision(){ detector_uninit(); }

    void START(Mat &input_img);
    static void Change_to_Binary(Mat &input_img, Mat &output_img, int value[], bool flag);
    static int traffic_action, parking_value[6], gatebar_value[6], traffic_value[6], sensor_data[3], timer_cnt,
               parking_condition, cross_condition, construct_condition, direction_angle;
    static bool motion_cplt, direction, timer_ctrl, process_flag[3];
    ROBIT_vision2::vision_msg2 Vision_msg;
    Mat Raw_image;

private:
    int gatebar_count, parking_count, line_count, cross_pixel, sign_count[3];
    Mat Gatebar_img, Perspective_img, Perspective_img2, Traffic_img;
    bool traffic_condition[3], gatebar_detect, cross_detect, construct_detect1, construct_detect2, parking_detect, pre_line_detect, sign_condition[4], rotate;

    void Perspective_View(Mat &input_img, Mat &output_img);
    void Perspective_View2(Mat &input_img, Mat &output_img);
    void Traffic_Process(Mat &raw_image, Mat &draw_image);
    void Parking_Process();
    void Cross_Process(bool direction);
    void Sign_Process(Mat &raw_image, Mat &draw_image);
    void Find_Sign_Img(Mat &input_img, Mat &draw_image);
    void Find_Gatebar(Mat &input_img);
    void Update_Message();
    void Construct_Process();

    enum {mark, line1, go, turn1, line2, detect, turn2l, turn2r, front, back, turn3l, turn3r, line3, front2, turn4};
    enum {green, yellow, red};
    enum {sign, wait, turn_l1, turn_r1, drive, turn_l2, turn_r2};
    enum {left, right, construct, parking};
    enum {slow, left_c1, front_c1};

Q_SIGNALS:
    void perspective_callback(cv::Mat perspective_img);
    void white_line_callback(cv::Mat edge_img);
    void yellow_line_callback(cv::Mat edge_line_img);
    void parking_sign_callback(cv::Mat parkingsign_img);
    void gatebar_callback(cv::Mat gatebar_img);
    void traffic_callback(cv::Mat traffic_img);
    void mission_callback();
};

class vision_labeling
{
public:
    vision_labeling(Mat &binary_img, int threshold);
    int num_of_labels, max_num, pixel_num, max_pixel;
    void do_labeling();
    void draw_maximum_label_rect(Mat &raw_img);
    vector<cv::Rect> blobs;

private:
    int pixel_threshold;
    Mat img_binary, img_labels, stats, centroids;
};

class vision_line
{
public:
    vision_line(Mat &line_img, Mat &line_img2);
    Mat line_color, line_color2, line_hsv, line_binary, line_binary2, line_binary3, line_binary4, yellow_line,
    white_line, edge_img, edge_img2, reinforce_line, V_reinforce_line,
    Y_reinforce_line, W_reinforce_line;

    static int yellow_value[6], Canny_value[2], white_value[6];
    static bool r_line_detect, l_line_detect, v_line_detect, y_line_detect, w_line_detect;
    static int r_line_xpos, l_line_xpos;
    static double white_line_deg, yellow_line_deg, vertical_line_deg;

private:
    void Line_Color_Analyze();
    void RANSAC_LINE(Mat &perspective_img, Mat &perspective_img2);
    void Reinforce_Line(Mat &input_img, Mat &output_img);
    void Reinforce_Vertical_Line(Mat &input_img, Mat &output_img);
    void Reinforce_Color_Line(Mat &input_img);
    void Generate_Line_Value();
};

class vision_record
{
public:
    vision_record(): count(0){}

    static bool button_clicked;
    void do_Record(Mat &input_img);
    void prepare_Record();
private:
    VideoWriter outputVideo;
    int count;
    string output_video;
};

class vision_RANSAC {

    struct sLine{
        double mx;
        double my;
        double sx;
        double sy;
    };

private:
    unsigned char* m_ImgData;
    int m_width;
    int m_height;
    double m_lineThreshold;
    double m_degree;
    Point m_line_A,m_line_B;
    vector<Point> m_pos;
    void getSamples();
    void compute_model_parameter(const vector<Point>& vInlierPoint, sLine &model);
    double compute_distance(sLine &line, Point &x)const;

public:
    bool m_noSamples;
    vision_RANSAC(const Mat& img);
    void runRansac();
    double getDegree()const{return m_degree;}
    Point getPointLineA()const{return m_line_A;}
    Point getPointLineB()const{return m_line_B;}
};
