#include <iostream>
#include "opencv2/opencv.hpp"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include <stdio.h>
#include <vector>

using namespace std;
using namespace cv;

#define WIDTH     640
#define HEIGHT    480

#define ROI_CENTER_Y  60
#define NO_LINE 20
#define LANE_WIDTH 640


int prev_x_left = 0;
int prev_x_right = WIDTH;
int cols_crop = 0;

Mat Region_of_Interest_crop(Mat image)
{
    Mat img_roi_crop;	
    int row_crop = 300;
    Rect rect(cols_crop,row_crop,image.cols-cols_crop,image.rows-row_crop);	 
    img_roi_crop = image(rect);

    return img_roi_crop;
}

Mat Canny_Edge_Detection(Mat img)
{
   Mat mat_blur_img, mat_canny_img;
   blur(img, mat_blur_img, Size(3,3));	
   Canny(mat_blur_img,mat_canny_img, 70,170,3);
	
   return mat_canny_img;	
}

int detect_line(Mat image, Mat roi_img, vector<Vec4i> linesP)
{
    float  c[NO_LINE] = {0.0, };
    float  d[NO_LINE] = {0.0, };

    int L_c =0;
    int R_c =0;

    vector<float> slopes;
    vector<Vec4i> filtered_lines;

    for(int i=0; i<linesP.size();i++){
        if(i>=NO_LINE) break;
        Vec4i L= linesP[i];
        if(L[2]-L[0] == 0) c[i] = 999;
        else{
            c[i] = float(L[3]-L[1])/float(L[2]-L[0]);
            d[i] = L[0]-c[i] *L[1];
        }

        if(abs(c[i]) > 0.4){
            slopes.push_back(c[i]);
            filtered_lines.push_back(linesP[i]);
        }
    }

    vector<Vec4i> left_lines;
    vector<Vec4i> right_lines;

    for(int i=0; i<filtered_lines.size();i++){
        if(i>=NO_LINE) break;
        Vec4i L= filtered_lines[i];
        
        if(slopes[i] < 0 && L[2] < WIDTH / 2){
            left_lines.push_back(filtered_lines[i]);
        }
        else if(slopes[i] > 0 && L[0] > WIDTH / 2){
            right_lines.push_back(filtered_lines[i]);
        }
    }

    float m_left = 0.0;
    float b_left = 0.0;
    float x_sum = 0.0;
    float y_sum = 0.0;
    float m_sum = 0.0; 

    for(int i=0; i<left_lines.size();i++){
        if(i>=NO_LINE) break;
        Vec4i L= left_lines[i];
        line(roi_img, Point(L[0],L[1]), Point(L[2],L[3]), Scalar(0, 0, 255), 2);
    }

    for(int i=0; i<right_lines.size();i++){
        if(i>=NO_LINE) break;
        Vec4i L= right_lines[i];
        line(roi_img, Point(L[0],L[1]), Point(L[2],L[3]), Scalar(0, 255, 0), 2);
    }
    
    if(left_lines.size() != 0){        
        for(int i=0; i<left_lines.size();i++){
            if(i>=NO_LINE) break;
            Vec4i L= left_lines[i];
            
            x_sum += L[0] + L[2];
            y_sum += L[1] + L[3];
            
            if(L[0] != L[2])
                m_sum += float(L[3] - L[1]) / float(L[2] - L[0]);
            else
                m_sum += 0;   
        }
        m_left = m_sum / left_lines.size();
        b_left = (y_sum / (left_lines.size()*2)) - m_left * (x_sum / (left_lines.size()*2));

        if (m_left != 0.0){
            line(roi_img, Point(int((0.0 - b_left) / m_left), 0), Point(int((roi_img.rows - b_left) / m_left), roi_img.rows), Scalar(255,0,0), 5);
        }
    }

    x_sum = 0.0;
    y_sum = 0.0;
    m_sum = 0.0; 
    float m_right = 0.0; 
    float b_right = 0.0;

    if(right_lines.size() != 0){
        for(int i=0; i<right_lines.size();i++){
            if(i>=NO_LINE) break;
            Vec4i L= right_lines[i];
            
            x_sum += L[0] + L[2];
            y_sum += L[1] + L[3];
            
            if(L[0] != L[2])
                m_sum += float(L[3] - L[1]) / float(L[2] - L[0]);
            else
                m_sum += 0;
        }
        m_right = m_sum / right_lines.size();
        b_right = (y_sum / (right_lines.size()*2)) - m_right * (x_sum / (right_lines.size()*2));

        if (m_right != 0.0){
            line(roi_img, Point(int((0.0 - b_right) / m_right),0), Point(int((roi_img.rows - b_right) / m_right), roi_img.rows), Scalar(255,255,0), 5);
        }
    }

    int x_l = 0;
    int x_r = 0; 

    if ((m_left == 0.0) && (m_right == 0.0)) {
        x_l = prev_x_left;
        x_r = prev_x_right;
    }
    else {
        if (m_left == 0.0) {
            x_r = int((ROI_CENTER_Y-b_right)/m_right);
            x_l = x_r - LANE_WIDTH;
        }
        else if (m_right == 0.0) {
            x_l = int((ROI_CENTER_Y-b_left)/m_left);
            x_r = x_l + LANE_WIDTH;
        }
        else {
            x_l = int((ROI_CENTER_Y-b_left)/m_left);
            x_r = int((ROI_CENTER_Y-b_right)/m_right);
        }
    }
    prev_x_left = x_l;
    prev_x_right = x_r;

    line(roi_img, Point(0, ROI_CENTER_Y), Point(roi_img.cols,ROI_CENTER_Y), Scalar(255,255,255), 5);
    int roi_mid = (x_l + x_r)/2;
    rectangle(roi_img, Point(roi_mid-5, ROI_CENTER_Y-5), Point(roi_mid+5, ROI_CENTER_Y+5), (0,0,255), 3);
    rectangle(roi_img, Point(WIDTH/2-5, ROI_CENTER_Y-5), Point(WIDTH/2+5, ROI_CENTER_Y+5), (255,0,0), 3);

    imshow("image", image);
    imshow("roi_img", roi_img);
    waitKey(1);

    return roi_mid;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "opencv_line_detect_cpp");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::Int16>("opencv_line_midpoint", 10);
    Mat image;
    VideoCapture cap(6, CAP_V4L);

    if(!cap.isOpened()){
        cerr << "ERROR : Unable to open camera\n";
        return -1;
    }


    while (ros::ok())
    {
        Mat image;
        cap >> image;
        cout<<image.rows<<" "<<image.cols;
        Mat roi_img = Region_of_Interest_crop(image);

        Mat image_gray;
        cvtColor(roi_img, image_gray, COLOR_RGB2GRAY);

        Mat canny_edge = Canny_Edge_Detection(image_gray);

        vector<Vec4i> linesP;
        HoughLinesP(canny_edge, linesP, 1, CV_PI/180,50,50,20);  
        
        int mid_point = detect_line(image, roi_img, linesP);
        int angle = (mid_point - WIDTH / 2) / 6;
        

        std_msgs::Int16 msg;
        msg.data = angle+45;
        cout << "angle : " << angle+45 << endl;
        pub.publish(msg);
    }
}
