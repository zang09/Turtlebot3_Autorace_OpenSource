#include "../include/ROBIT_vision2/vision.hpp"

using namespace cv;
using namespace std;
using namespace cv::xfeatures2d;

int sign_state;
string sign_value;

int vision_line::r_line_xpos = 0;
int vision_line::l_line_xpos = 0;

bool vision_line::r_line_detect = false;
bool vision_line::l_line_detect = false;
bool vision_line::v_line_detect = false;
bool vision_line::y_line_detect = false;
bool vision_line::w_line_detect = false;

double vision_line::white_line_deg = 0.0;
double vision_line::yellow_line_deg = 0.0;
double vision_line::vertical_line_deg = 0.0;

int vision::parking_condition = 0;
int vision::cross_condition = 0;
int vision::construct_condition = 0;

bool vision::direction = false;

void detect_mat(Mat frame_detect, float* detections_output, int* num_output_class, float thresh, float hier_thresh, vector<cv::Rect>& objBoxes)
{
    double time = what_is_the_time_now();

    // do detect in an unsigned char* data buffer getted from Mat::data or IplImage.imageData
    unsigned char* data = (unsigned char*)frame_detect.data;
    int w = frame_detect.cols;
    int h = frame_detect.rows;
    int c = frame_detect.channels();

    float* detections = test_detector_uchar(data, w, h, c, thresh, hier_thresh, num_output_class);

    for(int i = 0; i < *num_output_class; i++) {
        sign_state = (int)detections[i*6+0];

        detections_output[i*6+0] = detections[i*6+0]; // ith detection's category
        detections_output[i*6+1] = detections[i*6+1]; // ith detection's confidence score
        detections_output[i*6+2] = detections[i*6+2]; // ith detection's top-left x-coordinate of bbox
        detections_output[i*6+3] = detections[i*6+3]; // ith detection's top-left y-coordinate of bbox
        detections_output[i*6+4] = detections[i*6+4]; // ith detection's width of bbox
        detections_output[i*6+5] = detections[i*6+5]; // ith detection's height of bbox
        objBoxes.push_back(cv::Rect(detections_output[i*6+2], detections_output[i*6+3], detections_output[i*6+4], detections_output[i*6+5]));
    }
}

void vision::START(Mat &input_img)
{
    Raw_image = input_img.clone();
    Perspective_View(input_img, Perspective_img);
    Perspective_View2(input_img, Perspective_img2);
    Find_Sign_Img(Raw_image,input_img);
    Find_Gatebar(input_img);
    Traffic_Process(Raw_image, input_img);
    Construct_Process();
    vision_line line(Perspective_img, Perspective_img2);
    Update_Message();

    Q_EMIT white_line_callback(line.yellow_line);
    Q_EMIT yellow_line_callback(Perspective_img);
    Q_EMIT perspective_callback(Perspective_img2);
    Q_EMIT mission_callback();
}

void vision::Update_Message()
{
    Vision_msg.gatebar_detect = gatebar_detect;
    Vision_msg.r_angle = (int)(vision_line::white_line_deg);
    Vision_msg.l_angle = (int)(vision_line::yellow_line_deg);
    Vision_msg.r_line_info = vision_line::r_line_detect;
    Vision_msg.l_line_info = vision_line::l_line_detect;
    Vision_msg.r_diff_pixel = vision_line::r_line_xpos;
    Vision_msg.l_diff_pixel = vision_line::l_line_xpos;
    Vision_msg.cross_detect = cross_detect;
    Vision_msg.cross_info = cross_condition;
    Vision_msg.traffic = traffic_action;
    Vision_msg.parking_detect = parking_detect;
    Vision_msg.parking_info = parking_condition;
    Vision_msg.construct_detect = construct_detect2;
    Vision_msg.construct_info = construct_condition;
    if(cross_pixel > -160 && cross_pixel < 160)
        Vision_msg.cross_pixel = cross_pixel;
}

void vision::Change_to_Binary(Mat &input_img, Mat &output_img, int value[], bool flag)
{
    Mat mask1 = getStructuringElement(MORPH_RECT, Size(3, 3), Point(1, 1));
    Mat mask2 = getStructuringElement(MORPH_RECT, Size(15, 15), Point(1, 1));
    output_img = input_img.clone();
    medianBlur(output_img , output_img , 9);
    GaussianBlur(output_img , output_img , Size(15,15) , 2.0);
    cvtColor(output_img , output_img , CV_RGB2HSV);
    inRange(output_img, Scalar(value[3],value[4],value[5]),Scalar(value[0],value[1],value[2]), output_img);
    if(!flag){
        erode(output_img, output_img, mask1, Point(-1, -1), 2);
        dilate(output_img, output_img, mask1, Point(-1, -1), 2);
    }
    else{
        erode(output_img, output_img, mask1, Point(-1, -1), 2);
        dilate(output_img, output_img, mask2, Point(-1, -1), 2);
    }
}

void vision::Perspective_View(Mat &input_img, Mat &output_img)
{
    Point2f inputQuad[4], outputQuad[4];
    Mat lambda = Mat::zeros(input_img.rows, input_img.cols, input_img.type());

    inputQuad[0] = Point2f(50,155); inputQuad[1] = Point2f(270,155); inputQuad[2] = Point2f(300, 185); inputQuad[3] = Point2f(20, 185);
    outputQuad[0] = Point2f(0,0); outputQuad[1] = Point2f(RAW_X,0); outputQuad[2] = Point2f(RAW_X,RAW_Y); outputQuad[3] = Point2f(0,RAW_Y);
    lambda = getPerspectiveTransform(inputQuad, outputQuad);
    warpPerspective(input_img.clone(), output_img, lambda, output_img.size());

    circle(input_img,Point(50,155),3,Scalar(255,204,51),-1);
    circle(input_img,Point(270,155),3,Scalar(255,204,51),-1);
    circle(input_img,Point(20,185),3,Scalar(255,204,51),-1);
    circle(input_img,Point(300,185),3,Scalar(255,204,51),-1);
}

void vision::Perspective_View2(Mat &input_img, Mat &output_img)
{
    Point2f inputQuad[4], outputQuad[4];
    Mat lambda = Mat::zeros(input_img.rows, input_img.cols, input_img.type());

    inputQuad[0] = Point2f(20,115); inputQuad[1] = Point2f(300,115); inputQuad[2] = Point2f(300, 155); inputQuad[3] = Point2f(20, 155);
    outputQuad[0] = Point2f(0,0); outputQuad[1] = Point2f(RAW_X,0); outputQuad[2] = Point2f(RAW_X,RAW_Y); outputQuad[3] = Point2f(0,RAW_Y);
    lambda = getPerspectiveTransform(inputQuad, outputQuad);
    warpPerspective(input_img.clone(), output_img, lambda, output_img.size());

    circle(input_img,Point(50,155),3,Scalar(255,204,51),-1);
    circle(input_img,Point(270,155),3,Scalar(255,204,51),-1);
    circle(input_img,Point(20,185),3,Scalar(255,204,51),-1);
    circle(input_img,Point(300,185),3,Scalar(255,204,51),-1);
}

void vision::Find_Sign_Img(Mat &input_img, Mat &draw_image)
{
    Scalar color;
    Mat darknet = input_img(Rect(0, 0, 320, 120)).clone();
    cvtColor(darknet, darknet, CV_BGR2RGB);
    float* detections = (float*)calloc(255 * 6, sizeof(float));
    int num_of_outputs = 0;
    double thresh = 0.8;
    double hier_thresh = 0.0;
    vector<Rect> results;

    if(!process_flag[2])
        detect_mat(darknet, detections, &num_of_outputs, thresh, hier_thresh, results);

    if(!results.empty())
    {
        switch (sign_state) {
        case 0:
            sign_count[0]++;
            sign_value = "left";
            color = Scalar(0, 255, 0);
            if((direction_angle >= 168 && direction_angle <= 180) && !process_flag[0] && sign_count[0] > 1){
                sign_condition[left] = true;
                process_flag[0] = true;
                cross_detect = true;
                cross_condition = turn_l1;
                direction = true;
            }break;
        case 1:
            color = Scalar(255, 255, 0);
            sign_count[1]++;
            sign_value = "right";
            color = Scalar(0, 255, 0);
            if((direction_angle >= 170 && direction_angle <= 180) && !process_flag[0] && sign_count[1] > 2){
                sign_condition[right] = true;
                process_flag[0] = true;
                cross_detect = true;
                cross_condition = turn_r1;
                direction = false;
            }break;
        case 2:
            sign_count[2]++;
            sign_value = "parking";
            color = Scalar(255, 0, 0);
            if((direction_angle >= 170 && direction_angle <= 190) && !construct_detect2 && sign_count[2] > 3){
                process_flag[2] = true;
                parking_detect = true;
            }break;
        default:
            break;
        }
    }
    else
        sign_value = "NONE";

    for(int i = 0; i < results.size(); i++)
        rectangle(draw_image, results[i], color, 4);

    Sign_Process(Raw_image, input_img);
    Parking_Process();
    free(detections);
}

void vision::Find_Gatebar(Mat &input_img)
{
    Gatebar_img = input_img(Rect(0,0,320,150)).clone();
    Change_to_Binary(Gatebar_img, Gatebar_img, gatebar_value, 0);
    Q_EMIT gatebar_callback(Gatebar_img);
    vision_labeling Gatebar_labeling(Gatebar_img, 1300);
    Gatebar_labeling.do_labeling();
    if(Gatebar_labeling.blobs.size() > 0)
    {
        int max_x, min_x, max_num = 0, min_num = 0 , y_gap;
        max_x = Gatebar_labeling.blobs[0].x;
        min_x = Gatebar_labeling.blobs[0].x;
        for(int i = 0; i < Gatebar_labeling.blobs.size() ; i++)
        {
            if(Gatebar_labeling.blobs[i].x > max_x)
                max_num = i;
            if(Gatebar_labeling.blobs[i].x < min_x)
                min_num = i;
        }
        for(int i = 0 ; i < Gatebar_labeling.blobs.size() ; i++)
            rectangle(input_img, Gatebar_labeling.blobs[i], Red, 2);
        y_gap = Gatebar_labeling.blobs[max_num].y - Gatebar_labeling.blobs[min_num].y;
        if(Gatebar_labeling.blobs.size() >= 2 && sensor_data[0] > 380)
        {
            y_gap = Gatebar_labeling.blobs[max_num].y - Gatebar_labeling.blobs[min_num].y;
            if(abs(y_gap) < 50){
                gatebar_count++;
                if (gatebar_count > 0)
                    gatebar_detect = true;
                else
                    gatebar_detect = false;
            }
        }
        else
            gatebar_detect = false;
    }
    else
    {
        gatebar_detect = false;
        gatebar_count = 0;
    }
}

void vision::Traffic_Process(Mat &raw_image, Mat &draw_image)
{
    Mat Traffic_Sign_img = raw_image(Rect(200,80,100,70)).clone();
    rectangle(draw_image, Rect(200,80,100,70),Yellow, 3, 8, 0);
    Change_to_Binary(Traffic_Sign_img, Traffic_Sign_img, traffic_value, 0);
    Q_EMIT traffic_callback(Traffic_Sign_img);

    vision_labeling Traffic_labeling(Traffic_Sign_img, 300);
    Traffic_labeling.do_labeling();
    Traffic_labeling.draw_maximum_label_rect(draw_image);
    if(Traffic_labeling.blobs.size() > 0)
    {
        int x =  Traffic_labeling.blobs[Traffic_labeling.max_num].x + (int)(Traffic_labeling.blobs[Traffic_labeling.max_num].width / 2) + 200;
        int y =  Traffic_labeling.blobs[Traffic_labeling.max_num].y + (int)(Traffic_labeling.blobs[Traffic_labeling.max_num].height / 2) + 80;
        int radius  =  (int)(Traffic_labeling.blobs[Traffic_labeling.max_num].width / 2);
        circle(draw_image,Point(x, y), radius, Scalar(51,153,51), 3, 8, 0);
        traffic_action = 1;
    }
}

void vision::Construct_Process()
{
    if(!construct_detect1 && sensor_data[1] > 250 && (direction_angle < 10 || direction_angle > 350))  //30, 50
    {
        construct_detect1 = true;
    }

    if(construct_detect1 && sensor_data[1] > 250 && (direction_angle > 30 && direction_angle < 50))  //30, 50
    {
        timer_ctrl = true;
        construct_detect2 = true;
    }

    if(construct_detect1 && construct_detect2 && (timer_cnt < 45 && sensor_data[1] > 200 && sensor_data[0] > 80) && (direction_angle > 85 && direction_angle < 110))
    {
        process_flag[1] = true;
        construct_detect1 = false;
        timer_ctrl = false;
        construct_condition = left_c1;
    }
    else if(construct_detect2 && timer_cnt >= 45)
    {
        construct_detect1 = false;
        construct_detect2 = false;
        timer_ctrl = false;
    }

    if(construct_condition >= left_c1)
    {
        switch(construct_condition)
        {
        case left_c1:
            if(direction_angle >= 130 && direction_angle <= 140)
                construct_condition = front_c1;
            break;
        case front_c1:
            if((direction_angle >= 30 && direction_angle <= 80 && vision_line::w_line_detect))
            {
                construct_detect2 = false;
                construct_condition = 0;
            }
            break;
        }
    }
}

void vision::Sign_Process(Mat &raw_image, Mat &draw_image)
{
    Mat Blue_Sign_img = raw_image(Rect(0,0,320,120)).clone();
    Change_to_Binary(Blue_Sign_img, Blue_Sign_img, parking_value, 0);
    Q_EMIT parking_sign_callback(Blue_Sign_img);
    if(cross_condition >= wait && cross_detect)
    {
        if(sign_condition[right])
            Cross_Process(1);
        else if(sign_condition[left])
            Cross_Process(0);
    }
}

void vision::Cross_Process(bool direction)
{
    if(direction){
        if(cross_condition > wait)  //right
        {
            if(direction_angle >= 120 && direction_angle <= 130)
                cross_condition = drive;
            else if(cross_condition == drive){
                timer_ctrl = true;
                if(timer_cnt > 30 && vision_line::y_line_detect){
                    cross_condition = turn_r2;
                    timer_ctrl = false;
                }
            }
            else if(cross_condition == turn_r2){
                if(direction_angle >= 170 && direction_angle <= 190){
                    sign_condition[right] = false;
                    cross_detect = false;
                    cross_condition = 0;
                    sign_count[0] = 0;
                    sign_count[1] = 0;
                }
            }
        }
    }
    else{
        if(cross_condition > wait)  //left
        {
            if(direction_angle >= 225 && direction_angle <= 235)
                cross_condition = drive;
            else if(cross_condition == drive)
            {
                timer_ctrl = true;
                if(timer_cnt > 30 && vision_line::w_line_detect && (direction_angle >= 50 && direction_angle <= 130))
                {
                    vision::direction = true;
                    cross_condition = turn_l2;
                    timer_ctrl = false;
                }
            }
            else if(cross_condition == turn_l2){
                if(direction_angle >= 150 && direction_angle <= 170){
                    sign_condition[left] = false;
                    cross_detect = false;
                    cross_condition = 0;
                    sign_count[0] = 0;
                    sign_count[1] = 0;
                }
            }
        }
    }
}

void vision::Parking_Process()
{
    if(parking_detect && parking_condition >= mark)
    {
        switch (parking_condition)
        {
        case mark:
            if(vision_line::l_line_detect)
                parking_condition = line1;
            break;
        case line1:
            if(!vision_line::l_line_detect)
                parking_condition = go;
            break;
        case go:
            if(vision_line::y_line_detect){
                parking_condition = turn1;
            }
            break;
        case turn1:
            if(direction_angle <= 265 && direction_angle >= 255)
                parking_condition = line2;
            break;
        case line2:
            timer_ctrl = true;
            if(timer_cnt > 68 && sensor_data[2] > 350)
            {
                parking_condition = detect; rotate = 1; timer_ctrl = false;
            }
            else if(timer_cnt > 68 && sensor_data[1] > 350)
            {
                parking_condition = detect; rotate = 0; timer_ctrl = false;
            }
            break;
        case detect:
            if(vision_line::v_line_detect && rotate == 1)
                parking_condition = turn2l;
            else if(vision_line::v_line_detect && rotate == 0)
                parking_condition = turn2r;

            break;
        case turn2l:
            if(direction_angle <= 190 && direction_angle >= 170) // target
                parking_condition = back;
            break;
        case turn2r:
            if(direction_angle <= 10 || direction_angle >= 355)  // target /*direction_angle <= 185 && direction_angle >=175*/
                parking_condition = back;
            break;
        case back:
            timer_ctrl = true;
            if(timer_cnt > 4){
                parking_condition = front;
                timer_ctrl = false;
            }break;
        case front:
            timer_ctrl = true;
            if(timer_cnt > 6){
                if(rotate)
                    parking_condition = turn3l;
                else
                    parking_condition = turn3r;
                timer_ctrl = false;
            }break;
        case turn3r:
            if(direction_angle >= 75 && direction_angle <= 90)
                parking_condition = line3;
            break;
        case turn3l:
            if(direction_angle >= 90 && direction_angle <= 105)
                parking_condition = line3;
            break;
        case line3:
            if(!vision_line::r_line_detect && !vision_line::l_line_detect)
                parking_condition = front2;
            break;
        case front2:
            timer_ctrl = true;
            if(timer_cnt > 15){
                parking_condition = turn4;
                timer_ctrl = false;
            }break;
        case turn4:
            if(direction_angle >= 165 && direction_angle <= 175){
                parking_condition = 0;
                parking_detect = false;
                timer_ctrl = false;
                sign_count[2] = 0;
            }break;
        }
    }
}

vision_line:: vision_line(Mat &line_img, Mat &line_img2)
{  
    line_color = line_img.clone();
    line_color2 = line_img2.clone();

    vision::Change_to_Binary(line_img, line_binary, yellow_value, 0);
    vision::Change_to_Binary(line_img2, line_binary2, yellow_value, 0);
    vision::Change_to_Binary(line_img, line_binary3, white_value, 0);\
    vision::Change_to_Binary(line_img2, line_binary4, white_value, 0);

    medianBlur(line_color, line_color , 9);
    GaussianBlur(line_color, line_color , Size(15,15) , 2.0);
    Canny(line_color, edge_img, Canny_value[0], Canny_value[1], 3);

    medianBlur(line_color2, line_color2 , 9);
    GaussianBlur(line_color2, line_color2 , Size(15,15) , 2.0);
    Canny(line_color2, edge_img2, Canny_value[0], Canny_value[1], 3);

    Reinforce_Line(edge_img, reinforce_line);
    if(vision::parking_condition >= 2)
        Reinforce_Vertical_Line(edge_img, V_reinforce_line);
    if(vision::cross_condition >= 4 || vision::construct_condition >= 1 || vision::parking_condition >= 1)
    Reinforce_Color_Line(edge_img2);

    Line_Color_Analyze();
    RANSAC_LINE(line_img, line_img2);
}

void vision_line::Reinforce_Line(Mat &input_img, Mat &output_img)
{
    output_img = Mat::zeros(RAW_Y , RAW_X , CV_8U);
    int pos_average = 0;

    for(int y = 0; y < input_img.rows - 1; y++)
    {
        uchar* pointer_input_img = input_img.ptr<uchar>(y);
        uchar* pointer_output_img = output_img.ptr<uchar>(y);

        for(int x = 0; x < input_img.cols - 1; x++)
        {
            if(pointer_input_img[x] == 255)
            {
                vector<int> stack;
                stack.push_back(0);
                for(int dx = 0 ; dx < LINE_MAXIMUM_WIDTH && (x + dx) < (RAW_X - 1) ; dx++)
                {
                    if(pointer_input_img[x + dx] == 255)
                        stack.push_back(x + dx);
                }
                if(stack.size() > 0)
                {
                    if(stack[stack.size() - 1] - x > LINE_MINIMUM_WIDTH)
                    {
                        for(int i = 1 ; i < stack.size() ; i++)
                            pos_average += stack[i];
                        pos_average = (int)(pos_average / (stack.size() - 1));
                        pointer_output_img[pos_average] = 255;
                        if(pos_average + 1 < 320)
                            pointer_output_img[pos_average + 1] = 255;
                        if(pos_average - 1 > 0)
                            pointer_output_img[pos_average - 1] = 255;
                        pos_average = 0;
                    }
                }
            }
        }
    }
}

void  vision_line::Reinforce_Color_Line(Mat &input_img)
{
    Y_reinforce_line = Mat::zeros(RAW_Y, RAW_X, CV_8U);
    W_reinforce_line = Mat::zeros(RAW_Y, RAW_X, CV_8U);
    int pos_average = 0;

    for(int y = 90; y < 180; y++)
    {
        for(int x = 40; x < 280 ; x++)
        {
            uchar *pointer_input_img = input_img.ptr<uchar>(y);
            uchar *pointer_line_binary_img = line_binary2.ptr<uchar>(y); //yellow
            if(pointer_input_img[x] == 255)
            {
                vector<int> stack;
                stack.push_back(0);
                for(int dy = 0 ; dy < LINE_MAXIMUM_WIDTH  && (y + dy) < (180 - 1) ; dy++)
                {
                    uchar* pointer_input_check = input_img.ptr<uchar>(y + dy);
                    if(pointer_input_check[x] == 255)
                        stack.push_back(y + dy);
                }
                if(stack.size() > 0)
                {
                    if(stack[stack.size() - 1] - y > LINE_MINIMUM_WIDTH)
                    {
                        for(int i = 1 ; i < stack.size() ; i++)
                            pos_average += stack[i];
                        pos_average = (int)(pos_average / (stack.size() - 1));
                        if(pointer_line_binary_img[x] == 255)
                        {
                            for(int i = -1 ; i < 2 ; i++)
                            {
                                uchar* pointer_output_img = Y_reinforce_line.ptr<uchar>(y + i);
                                for(int i = -1 ; i < 2 ; i++)
                                {
                                    if(x + i < 320 && x + i > 0)
                                        pointer_output_img[x + i] = 255;
                                }
                            }
                        }

                        pos_average = 0;
                    }
                }
            }
        }
    }

    pos_average = 0;

    for(int y = 60; y < 150; y++)
    {
        uchar* pointer_input_img = input_img.ptr<uchar>(y);
        uchar *pointer_line_binary_img2 = line_binary4.ptr<uchar>(y); //white
        for(int x = 40; x < 280 ; x++)
        {
            if(pointer_input_img[x] == 255)
            {
                vector<int> stack;
                stack.push_back(0);
                for(int dx = 0 ; dx < LINE_MAXIMUM_WIDTH2 && (x + dx) < (280 - 1) ; dx++)
                {
                    if(pointer_input_img[x + dx] == 255)
                        stack.push_back(x + dx);
                }
                if(stack.size() > 0)
                {
                    if(stack[stack.size() - 1] - x > LINE_MINIMUM_WIDTH)
                    {
                        for(int i = 1 ; i < stack.size()  ; i++)
                            pos_average += stack[i];
                        pos_average = (int)(pos_average / (stack.size() - 1));
                        if(pointer_line_binary_img2[pos_average] == 255)
                        {
                            for(int i = -1 ; i < 2 ; i++)
                            {
                                uchar* pointer_output_img = W_reinforce_line.ptr<uchar>(y + i);
                                for(int j = -1 ; j < 2 ; j++)
                                {
                                    if(x + j < 320 && x + j > 0)
                                        pointer_output_img[x + j] = 255;
                                }
                            }
                        }
                        pos_average = 0;
                    }
                }
            }
        }
    }
}

void vision_line::Reinforce_Vertical_Line(Mat &input_img, Mat &output_img)
{
    output_img = Mat::zeros(RAW_Y, RAW_X, CV_8U);
    int pos_average = 0;
    int y_gap;
    if(vision::parking_condition > 2)
    {
        y_gap = 0;
    }
    else
    {
        y_gap = 100;
    }
    for(int y = y_gap; y < input_img.rows - 1; y++)
    {
        for(int x = 100; x < 220 ; x++)
        {
            uchar* pointer_input_img = input_img.ptr<uchar>(y);
            if(pointer_input_img[x] == 255)
            {
                vector<int> stack;
                stack.push_back(0);
                for(int dy = 0 ; dy < LINE_MAXIMUM_WIDTH && (y + dy) < (RAW_Y - 1) ; dy++)
                {
                    uchar* pointer_input_check = input_img.ptr<uchar>(y + dy);
                    if(pointer_input_check[x] == 255)
                        stack.push_back(y + dy);
                }
                if(stack.size() > 0)
                {
                    if(stack[stack.size() - 1] - y > LINE_MINIMUM_WIDTH)
                    {
                        for(int i = 1 ; i < stack.size() ; i++)
                            pos_average += stack[i];
                        pos_average = (int)(pos_average / (stack.size() - 1));
                        if(pos_average + 1 < 320 && pos_average - 1 > 0)
                        {
                            if(vision::parking_condition >= 3)
                            {
                                uchar *pointer_line_binary_img = line_binary.ptr<uchar>(pos_average);
                                if(pointer_line_binary_img[x] == 255)
                                {
                                    for(int i = -1 ; i < 2 ; i++)
                                    {
                                        uchar* pointer_output_img = output_img.ptr<uchar>(pos_average + i);
                                        for(int i = -1 ; i < 2; i++)
                                            pointer_output_img[x + i] = 255;
                                    }
                                }
                            }
                            else
                            {
                                for(int i = -1 ; i < 2 ; i++)
                                {
                                    uchar* pointer_output_img = output_img.ptr<uchar>(pos_average + i);
                                    for(int i = -1 ; i < 2; i++)
                                        pointer_output_img[x + i] = 255;
                                }
                            }
                        }
                        pos_average = 0;
                    }
                }
            }
        }
    }
}

void vision_line::Line_Color_Analyze()
{
    yellow_line = Mat::zeros(RAW_Y / 2 , RAW_X , CV_8U);
    white_line = Mat::zeros(RAW_Y / 2 , RAW_X , CV_8U);
    if(vision::parking_condition < 4)
    {
        for(int y = Y_POS_GAP ; y < reinforce_line.rows ; y++)
        {
            uchar *pointer_line_img  = reinforce_line.ptr<uchar>(y);
            uchar *pointer_line_binary_img = line_binary.ptr<uchar>(y);
            uchar *pointer_line_binary_img2 = line_binary3.ptr<uchar>(y);
            uchar *pointer_yellow_img  = yellow_line.ptr<uchar>(y - Y_POS_GAP);
            uchar *pointer_white_img  = white_line.ptr<uchar>(y - Y_POS_GAP);
            for(int x = 0 ; x < reinforce_line.cols ; x++)
            {
                if(pointer_line_img[x] == 255)
                {
                    if(pointer_line_binary_img[x] == 255)
                    {
                        for(int i = -1 ; i < 2 ; i++)
                            pointer_yellow_img[x + i] = 255;
                    }
                    else if(pointer_line_binary_img2[x] == 255)
                    {
                        for(int i = -1 ; i < 2 ; i++)
                            pointer_white_img[x] = 255;
                    }
                }
            }
        }
    }
    else
    {
        for(int y = Y_POS_GAP ; y < reinforce_line.rows ; y++)
        {
            uchar *pointer_line_img  = reinforce_line.ptr<uchar>(y);
            uchar *pointer_yellow_img  = yellow_line.ptr<uchar>(y - Y_POS_GAP);
            uchar *pointer_white_img  = white_line.ptr<uchar>(y - Y_POS_GAP);
            for(int x = 0 ; x < reinforce_line.cols; x++)
            {
                if(pointer_line_img[x] == 255)
                {
                    if(x < 160)
                        pointer_yellow_img[x] = 255;
                    else
                        pointer_white_img[x] = 255;
                }
            }
        }
    }

    if(vision::parking_condition < 4)
    {
        for(int y = 0; y < white_line.rows ; y++)
        {
            uchar *pointer_white_img  = white_line.ptr<uchar>(y);
            for(int x = 0 ; x < 100 ; x++)
                pointer_white_img[x] = 0;
        }
        for(int y = 0; y < white_line.rows ; y++)
        {
            uchar *pointer_yellow_img  = yellow_line.ptr<uchar>(y);
            for(int x = 220 ; x < 320 ; x++)
                pointer_yellow_img[x] = 0;
        }
    }
}

void vision_line::RANSAC_LINE(Mat &perspective_img, Mat &perspective_img2)
{
    vision_RANSAC Yellow_RANSAC(yellow_line);
    vision_RANSAC White_RANSAC(white_line);
    vision_RANSAC Vertical_RANSAC(V_reinforce_line);
    vision_RANSAC Y_Vertical_RANSAC(Y_reinforce_line);
    vision_RANSAC W_Vertical_RANSAC(W_reinforce_line);

    Yellow_RANSAC.runRansac();
    White_RANSAC.runRansac();
    Vertical_RANSAC.runRansac();
    Y_Vertical_RANSAC.runRansac();
    W_Vertical_RANSAC.runRansac();

    if(!Yellow_RANSAC.m_noSamples)
    {
        Point yellow_point_a = Yellow_RANSAC.getPointLineA();
        yellow_point_a = Point(yellow_point_a.x,yellow_point_a.y + Y_POS_GAP);
        Point yellow_point_b = Yellow_RANSAC.getPointLineB();
        yellow_point_b = Point(yellow_point_b.x,yellow_point_b.y + Y_POS_GAP);
        l_line_detect = true;
        l_line_xpos = (int)((yellow_point_a.x + yellow_point_b.x) / 2);
        yellow_line_deg = Yellow_RANSAC.getDegree();
        line(perspective_img, yellow_point_a, yellow_point_b, Blue, 2, CV_AA);
    }
    else
        l_line_detect = false;

    if(!White_RANSAC.m_noSamples)
    {
        Point white_point_a = White_RANSAC.getPointLineA();
        white_point_a = Point(white_point_a.x,white_point_a.y + Y_POS_GAP);
        Point white_point_b = White_RANSAC.getPointLineB();
        white_point_b = Point(white_point_b.x,white_point_b.y + Y_POS_GAP);
        if(vision::cross_condition >= 2 && vision::direction)
        {
            r_line_detect = false;
        }
        else
        {
            r_line_detect = true;
            r_line_xpos = (int)((white_point_a.x + white_point_b.x) / 2);
            white_line_deg =  White_RANSAC.getDegree();
            line(perspective_img, white_point_a, white_point_b, Red, 2, CV_AA);
        }
    }
    else
        r_line_detect = false;

    if(!Vertical_RANSAC.m_noSamples)
    {
        Point vertical_point_a = Vertical_RANSAC.getPointLineA();
        Point vertical_point_b = Vertical_RANSAC.getPointLineB();
        v_line_detect = true;
        vertical_line_deg = Vertical_RANSAC.getDegree();
        line(perspective_img, vertical_point_a, vertical_point_b, Green, 2, CV_AA);
    }
    else
        v_line_detect = false;

    if(!Y_Vertical_RANSAC.m_noSamples)
    {
        Point y_vertical_point_a = Y_Vertical_RANSAC.getPointLineA();
        Point y_vertical_point_b = Y_Vertical_RANSAC.getPointLineB();
        y_line_detect = true;
        line(perspective_img2, y_vertical_point_a, y_vertical_point_b, Purple, 2, CV_AA);
    }
    else
        y_line_detect = false;

    if(!W_Vertical_RANSAC.m_noSamples)
    {
        Point w_vertical_point_a = W_Vertical_RANSAC.getPointLineA();
        Point w_vertical_point_b = W_Vertical_RANSAC.getPointLineB();
        w_line_detect = true;
        line(perspective_img2, w_vertical_point_a, w_vertical_point_b, Skyblue, 2, CV_AA);
    }
    else
        w_line_detect = false;
}

vision_labeling::vision_labeling(Mat &binary_img, int threshold)
{
    img_binary = binary_img.clone();
    pixel_threshold = threshold;
}

void vision_labeling::do_labeling()
{
    pixel_num = 0;
    num_of_labels = connectedComponentsWithStats(img_binary, img_labels,stats,centroids,8,CV_32S);
    for(int i = 1 ; i < num_of_labels ; i++)
    {
        int area =  stats.at<int>(i,CC_STAT_AREA);
        int left = stats.at<int>(i,CC_STAT_LEFT);
        int top = stats.at<int>(i,CC_STAT_TOP);
        int width = stats.at<int>(i,CC_STAT_WIDTH);
        int height = stats.at<int>(i,CC_STAT_HEIGHT);

        for(int y = top ; y < top + height ; y++)
        {
            uchar *pointer_binary_img  = img_binary.ptr<uchar>(y);
            for(int x = left ; x < left + width ; x++)
            {
                if(pointer_binary_img[x] == 255)
                    pixel_num++;
            }
        }
        if(pixel_num > pixel_threshold)
            blobs.push_back(cv::Rect(left,top,width,height));
    }
}

void vision_labeling::draw_maximum_label_rect(Mat &raw_img)
{
    max_num = 0; max_pixel = 0;
    if(blobs.size() > 0)
    {
        int *num_of_pixel = new int[blobs.size()];
        for(int i = 0 ; i < blobs.size() ; i++)
            num_of_pixel[i] = 0;
        for(int i = 0 ; i < blobs.size() ; i++)
        {
            for(int y = blobs[i].y ; y < blobs[i].y + blobs[i].height ; y++)
            {
                uchar *pointer_binary_img  = img_binary.ptr<uchar>(y);
                for(int x = blobs[i].x ; x < blobs[i].x + blobs[i].width ; x++)
                {
                    if(pointer_binary_img[x] == 255)
                        num_of_pixel[i]++;
                }
            }
            if(num_of_pixel[i] > max_pixel)
            {
                max_pixel = num_of_pixel[i];
                max_num = i;
            }
        }
        delete num_of_pixel;
    }
}

void vision_record::prepare_Record()
{
    if(button_clicked && count == 0)
    {
        count++;
        output_video = string("left.avi");
        outputVideo.open(output_video,  cv::VideoWriter::fourcc('M','J','P','G'), 13.0, cv::Size(320,240), true);
    }
}

void vision_record::do_Record(Mat &input_img)
{
    if(button_clicked){
        outputVideo << input_img;
    }
}

vision_RANSAC::vision_RANSAC(const Mat& img)
{
    m_ImgData = (unsigned char *)img.data;
    m_width  = img.cols;
    m_height = img.rows;
    m_lineThreshold = 10;
    m_pos.clear();
    m_noSamples = true;
    m_degree = 0.0;
}

void vision_RANSAC::getSamples()
{
    const unsigned int threshold_nSamples = (m_width + m_height) / 4;
    Point Pt_temp;
    for(int i = 0 ,idxY = 0 ; i < m_height ; i++, idxY += m_width)
    {
        for(int j = 0; j < m_width; j++)
        {
            if(m_ImgData[idxY + j] != 0)
            {
                Pt_temp.x = j;
                Pt_temp.y = i;
                m_pos.push_back(Pt_temp);
            }
        }
    }
    if(m_pos.size() >= threshold_nSamples) m_noSamples = false;
}

void vision_RANSAC::compute_model_parameter(const vector<Point>& vInlierPoint, sLine &model)
{
    double sx  = 0, sy  = 0;
    double sxx = 0, syy = 0;
    double sxy = 0;
    for(size_t i = 0; i < vInlierPoint.size() ; ++i)
    {
        const double x = (double)vInlierPoint.at(i).x;
        const double y = (double)vInlierPoint.at(i).y;

        sx  += x;
        sy  += y;
        sxx += x*x;
        sxy += x*y;
        syy += y*y;
    }
    const double sw= vInlierPoint.size();
    const double vxx = (sxx - sx * sx / sw) / sw;
    const double vxy = (sxy - sx * sy / sw) / sw;
    const double vyy = (syy - sy * sy / sw) / sw;
    const double theta = atan2(2.0 * vxy, vxx - vyy) / 2.0;

    model.mx = cos(theta);
    model.my = sin(theta);

    model.sx = sx / sw;
    model.sy = sy / sw;
}

double vision_RANSAC::compute_distance(sLine &line, Point &x)const
{
    return fabs((double)((double)(x.x - line.sx)*line.my - ((double)x.y - line.sy)*line.mx))/sqrt(line.mx*line.mx + line.my*line.my);
}

void vision_RANSAC::runRansac()
{
    getSamples();
    if(m_noSamples == true) return;
    srand(time(NULL));

    unsigned int nSamples = 0;

    nSamples=(m_pos.size() * 4) / 10;

    const double distance_threshold = 1.0;
    double max_cost = 0.0;

    sLine estimated_model;

    for(int i = 0 ; i < nSamples ; i++)
    {
        const int  n = rand() % m_pos.size();
        const int  m = rand() % m_pos.size();

        if(n == m)continue;
        if(norm(m_pos[n] - m_pos[m]) < m_lineThreshold)continue;

        vector<Point> SamplePt;
        SamplePt.push_back(m_pos[n]);
        SamplePt.push_back(m_pos[m]);

        compute_model_parameter(SamplePt,estimated_model);

        double cost = 0.;

        const double sqrtmmyy = sqrt(estimated_model.mx * estimated_model.mx + estimated_model.my * estimated_model.my);
        const double mysx = estimated_model.sx * estimated_model.my;
        const double mxsy = estimated_model.sy * estimated_model.mx;
        const double sub_ms = mxsy - mysx;

        for(size_t i = 0; i < m_pos.size(); i++)
        {
            const double distance = fabs((((double)m_pos[i].x*estimated_model.my) - ((double)m_pos[i].y * estimated_model.mx)) + sub_ms) / sqrtmmyy;
            if (distance < distance_threshold)
                cost += 1.0;
        }
        const double CostThresHold = norm(SamplePt[0] - SamplePt[1]) / 3.0;

        if(cost < CostThresHold)continue;

        if(cost > max_cost)
        {
            m_line_A = m_pos[n];
            m_line_B = m_pos[m];
            max_cost = cost;
        }
    }

    if(m_line_A.x < m_line_B.x)
    {
        Point tempP = m_line_A;
        m_line_A = m_line_B;
        m_line_B = tempP;
    }

    m_degree = -(atan2(m_line_A.y - m_line_B.y, m_line_A.x - m_line_B.x) * 180.0) / CV_PI;
    if(m_degree < 0.0) m_degree += 180.0;
}

