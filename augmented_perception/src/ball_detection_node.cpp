
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/video/background_segm.hpp>

#include <stdio.h>
#include <opencv2/opencv.hpp>

#include <cmath>
#include <iostream>

using namespace sensor_msgs;
using namespace cv;
using namespace std;

image_transport::Publisher pub;

//unsigned char *bg;
unsigned char *input;

cv::Mat bg;
cv_bridge::CvImagePtr cv_ptr;

int hue;
bool firstframe = true;
bool paused = false;

int getH(int r, int g, int b){

    int max,min,delta;

    if(r >= g && r >= b)
    {
        max = r;
    }

    if(g >= r && g >= b)
    {
        max = g;
    }

    if(b >= r && b >= g) {

        max = b;
    }

    if(r <= g && r <= b)
    {
        min = r;
    }

    if(g <= r && g <= b)
    {
        min = g;
    }

    if(b <= r && b <= g) {
        min = b;
    }

    delta = max - min;

    if(delta == 0){
        return 0;
    }


    int result;

    if(max == r){
        result = (int)((60/1.41)*(fmod(((g-b)/(float)delta),6)))%256;
    }

    if(max == g){
        result = (int)((60/1.41)*(((b-r)/(float)delta+2)))%256;
    }

    if(max == b){
        result = (int)((60/1.41)*(((r-g)/(float)delta+4)))%256;
    }

    if(result < 0 ){
        return 256-result;
    }
    else return result;
}

int getS(int r, int g, int b){

    int max,min,delta;

    if(r >= g && r >= b)
    {
        max = r;
    }

    if(g >= r && g >= b)
    {
        max = g;
    }

    if(b >= r && b >= g) {
        max = b;
    }

    if(r <= g && r <= b)
    {
        min = r;
    }

    if(g <= r && g <= b)
    {
        min = g;
    }

    if(b <= r && b <= g) {
        min = b;
    }

    delta = max - min;

    if(max == 0){
        return 0;
    }
    else{
        return (int)((delta*1.0/max)*255);
    }
}

int getV(int r, int g, int b){

    int max,min,delta;

    if(r >= g && r >= b)
    {
        return r;
    }

    if(g >= r && g >= b)
    {
        return g;
    }

    else{ //if(b >= r && b >= g) {
        return b;
    }
}

void mouseHandler(int event, int x, int y, int flags, void* param)
{

    if ( flags == (EVENT_FLAG_CTRLKEY + EVENT_FLAG_LBUTTON) )
    {
        Mat* rgb = (Mat*) param;
        cout << "AAA" << endl;
        // this is bugged
        /*if (event == CV_EVENT_LBUTTONDOWN)
        {
            printf("%d %d: %d, %d, %d\n",
                   x, y,
                   (int)(*rgb).at<Vec3b>(Point(x, y))[0],
                   (int)(*rgb).at<Vec3b>(Point(x, y))[1],
                   (int)(*rgb).at<Vec3b>(Point(x, y))[2]);
        }*/
    }

}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{

    int key = -1;
    Mat image;

    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        key = cv::waitKey(30);

        //Show image
        image = cv_ptr->image;

        if(!paused){
            cvSetMouseCallback("camera",mouseHandler,&image);
            cv::imshow("camera", image);
        }

    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }

    input = (unsigned char*)(cv_ptr->image.data);

    if(firstframe || key == 32 /*spacebar*/){ //define bg
        firstframe = false;
        bg = image.clone();
        blur(bg,bg,cv::Size(20,20));
        cv::imshow("bg", bg);
        cv::waitKey(30);
    }

    if(key == 112) paused=!paused; // P


    //subtract BG
    Mat sub = image - bg;

    //up half ignore
    for(int y = 0;y < cv_ptr->image.rows/2;y++) {
        for (int x = 0; x < cv_ptr->image.cols; x++) {
            sub.at<Vec3b>(Point(x, y))[0] = 0;
            sub.at<Vec3b>(Point(x, y))[1] = 0;
            sub.at<Vec3b>(Point(x, y))[2] = 0;
        }
    }


    for(int y = cv_ptr->image.rows/2;y < cv_ptr->image.rows;y++) {
        for (int x = 0; x < cv_ptr->image.cols; x++) {
            unsigned char b = sub.at<Vec3b>(Point(x, y))[0];
            unsigned char g = sub.at<Vec3b>(Point(x, y))[1];
            unsigned char r = sub.at<Vec3b>(Point(x, y))[2];

            int h = getH(r,g,b);
            int s = getS(r,g,b);
            int v = getV(r,g,b);

            if(v < 10){
                sub.at<Vec3b>(Point(x, y))[0] = 0;
                sub.at<Vec3b>(Point(x, y))[1] = 0;
                sub.at<Vec3b>(Point(x, y))[2] = 0;
            }
            else{
                if(h > 10 && h < 251){
                    sub.at<Vec3b>(Point(x, y))[0] = 0;
                    sub.at<Vec3b>(Point(x, y))[1] = 0;
                    sub.at<Vec3b>(Point(x, y))[2] = 0;
                }
                else{
                    sub.at<Vec3b>(Point(x, y))[0] = 255;
                    sub.at<Vec3b>(Point(x, y))[1] = 255;
                    sub.at<Vec3b>(Point(x, y))[2] = 255;
                }
            }

        }
    }


    // Publish the data.
    cv_bridge::CvImage out_msg;
    out_msg.header   = msg->header; // Same timestamp and tf frame as input image
    out_msg.encoding = sensor_msgs::image_encodings::BGR8; // Or whatever
    out_msg.image    = sub; // Your cv::Mat

    pub.publish(out_msg.toImageMsg());

}

int main(int argc, char **argv)
{
    //Init ROS
    ros::init(argc, argv, "ball_detection_node");
    ros::NodeHandle nh;

    cv::namedWindow("camera",CV_WINDOW_NORMAL);
    cv::startWindowThread();

    cv::namedWindow("bg",CV_WINDOW_NORMAL);
    cv::startWindowThread();

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("camera/image_color", 1, imageCallback);

    // Create a ROS publisher for the output point cloud
    pub = it.advertise("output/ball", 1);

    ros::spin();
    cv::destroyAllWindows();
}