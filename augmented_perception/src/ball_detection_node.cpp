
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>

#include <opencv2/video/background_segm.hpp>

#include <stdio.h>
#include <opencv2/opencv.hpp>

#include <cmath>
#include <iostream>

using namespace sensor_msgs;
using namespace cv;
using namespace std;

image_transport::Publisher pub;

// unsigned char *bg;
unsigned char* input;

cv::Mat bg;
cv::Mat click;
cv_bridge::CvImagePtr cv_ptr;

uchar hue = 2;
int vibrance = 110;
bool firstframe = true;
bool paused = false;

// rosparams
bool using_rviz = false;

int getH(int r, int g, int b)
{
  int max, min, delta;

  if (r >= g && r >= b)
  {
    max = r;
  }

  if (g >= r && g >= b)
  {
    max = g;
  }

  if (b >= r && b >= g)
  {
    max = b;
  }

  if (r <= g && r <= b)
  {
    min = r;
  }

  if (g <= r && g <= b)
  {
    min = g;
  }

  if (b <= r && b <= g)
  {
    min = b;
  }

  delta = max - min;

  if (delta == 0)
  {
    return 0;
  }

  int result;

  if (max == r)
  {
    result = (int)((60 / 1.41) * (fmod(((g - b) / (float)delta), 6))) % 256;
  }

  if (max == g)
  {
    result = (int)((60 / 1.41) * (((b - r) / (float)delta + 2))) % 256;
  }

  if (max == b)
  {
    result = (int)((60 / 1.41) * (((r - g) / (float)delta + 4))) % 256;
  }

  if (result < 0)
  {
    return 256 - result;
  }
  else
    return result;
}

int getS(int r, int g, int b)
{
  int max, min, delta;

  if (r >= g && r >= b)
  {
    max = r;
  }

  if (g >= r && g >= b)
  {
    max = g;
  }

  if (b >= r && b >= g)
  {
    max = b;
  }

  if (r <= g && r <= b)
  {
    min = r;
  }

  if (g <= r && g <= b)
  {
    min = g;
  }

  if (b <= r && b <= g)
  {
    min = b;
  }

  delta = max - min;

  if (max == 0)
  {
    return 0;
  }
  else
  {
    return (int)((delta * 1.0 / max) * 255);
  }
}

int getV(int r, int g, int b)
{
  int max, min, delta;

  if (r >= g && r >= b)
  {
    return r;
  }

  if (g >= r && g >= b)
  {
    return g;
  }

  else
  {  // if(b >= r && b >= g) {
    return b;
  }
}

void mouseHandler(int event, int x, int y, int flags, void* param)
{
  if (flags == (EVENT_FLAG_LBUTTON))
  {
    // this is bugged
    if (event == CV_EVENT_LBUTTONDOWN)
    {
      Vec3b pixel = click.at<Vec3b>(y, x);

      uchar b = pixel[0];
      uchar g = pixel[1];
      uchar r = pixel[2];
      hue = getH((int)r, (int)g, (int)b) - 2;
      ROS_INFO("Selected Hue: %d", (int)hue);
    }
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

    // Show image
    image = cv_ptr->image;

    click = image.clone();

    if (!paused)
    {
      cvSetMouseCallback("camera", mouseHandler, &click);
      cv::imshow("camera", image);
    }
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }

  input = (unsigned char*)(cv_ptr->image.data);

  if (firstframe || key == 32 /*spacebar*/)
  {  // define bg
    firstframe = false;
    bg = image.clone();
    cv::imshow("bg", bg);
    blur(bg, bg, cv::Size(20, 20));
    cv::waitKey(30);
  }

  if (key == 112)
    paused = !paused;  // P

  // subtract BG
  Mat sub = image - bg;

  // up half ignore
  for (int y = 0; y < sub.rows / 2; y++)
  {
    for (int x = 0; x < sub.cols; x++)
    {
      sub.at<Vec3b>(Point(x, y))[0] = 0;
      sub.at<Vec3b>(Point(x, y))[1] = 0;
      sub.at<Vec3b>(Point(x, y))[2] = 0;
    }
  }

  bool pointsToDelete[sub.rows / 2 * sub.cols] = { false };
  int meanX = 0;
  int meanY = 0;
  int boundX = 0;
  int boundY = 0;
  int cntBallPoint = 0;

  uchar Hthreshold = 8;
  uchar hmax = hue + Hthreshold;
  uchar hmin = hue - Hthreshold;

  bool more_is_less = false;
  if (hmax < hmin)
  {
    more_is_less = true;
  }

  for (int y = sub.rows / 2; y < sub.rows; y++)
  {
    for (int x = 0; x < sub.cols; x++)
    {
      unsigned char b = sub.at<Vec3b>(Point(x, y))[0];
      unsigned char g = sub.at<Vec3b>(Point(x, y))[1];
      unsigned char r = sub.at<Vec3b>(Point(x, y))[2];

      int h = getH(r, g, b);
      int v = getV(r, g, b);

      if (v < 10)
      {
        sub.at<Vec3b>(Point(x, y))[0] = 0;
        sub.at<Vec3b>(Point(x, y))[1] = 0;
        sub.at<Vec3b>(Point(x, y))[2] = 0;
      }
      else
      {
        if (((h > (int)hmax && h < (int)hmin) && more_is_less) || ((h > (int)hmax || h < (int)hmin) && !more_is_less))
        {
          sub.at<Vec3b>(Point(x, y))[0] = 0;
          sub.at<Vec3b>(Point(x, y))[1] = 0;
          sub.at<Vec3b>(Point(x, y))[2] = 0;
        }
        else
        {
          sub.at<Vec3b>(Point(x, y))[0] = 255;
          sub.at<Vec3b>(Point(x, y))[1] = 255;
          sub.at<Vec3b>(Point(x, y))[2] = 255;

          // detect if this points is to be black afterwards
          int cnt = 0;

          if (sub.at<Vec3b>(Point(x - 1, y))[0] == 255)
            cnt++;
          if (sub.at<Vec3b>(Point(x - 1, y - 1))[0] == 255)
            cnt++;
          if (sub.at<Vec3b>(Point(x, y - 1))[0] == 255)
            cnt++;
          if (sub.at<Vec3b>(Point(x + 1, y - 1))[0] == 255)
            cnt++;

          if (sub.at<Vec3b>(Point(x - 2, y))[0] == 255)
            cnt++;
          if (sub.at<Vec3b>(Point(x - 2, y - 1))[0] == 255)
            cnt++;
          if (sub.at<Vec3b>(Point(x + 2, y - 1))[0] == 255)
            cnt++;
          if (sub.at<Vec3b>(Point(x - 1, y - 2))[0] == 255)
            cnt++;
          if (sub.at<Vec3b>(Point(x, y - 2))[0] == 255)
            cnt++;
          if (sub.at<Vec3b>(Point(x + 1, y - 2))[0] == 255)
            cnt++;

          if (sub.at<Vec3b>(Point(x - 3, y))[0] == 255)
            cnt++;
          if (sub.at<Vec3b>(Point(x, y - 3))[0] == 255)
            cnt++;

          if (cnt < 12)
          {
            pointsToDelete[(y - sub.rows / 2) * sub.cols + x] = true;
          }
          else
          {  // probably ball
            boundX *= cntBallPoint;
            boundY *= cntBallPoint;

            cntBallPoint++;

            meanX += x;
            meanY += y;

            boundX += x;
            boundY += y;

            boundX /= cntBallPoint;
            boundY /= cntBallPoint;
          }
        }
      }
    }
  }

  // delete the points
  for (int y = sub.rows / 2; y < sub.rows; y++)
  {
    for (int x = 0; x < sub.cols; x++)
    {
      if (y == cv_ptr->image.rows / 2)
      {
        sub.at<Vec3b>(Point(x, y))[0] = 0;
        sub.at<Vec3b>(Point(x, y))[1] = 0;
        sub.at<Vec3b>(Point(x, y))[2] = 0;
      }
      else if (pointsToDelete[(y - sub.rows / 2) * sub.cols + x] == 1)
      {
        sub.at<Vec3b>(Point(x, y))[0] = 0;
        sub.at<Vec3b>(Point(x, y))[1] = 0;
        sub.at<Vec3b>(Point(x, y))[2] = 0;
      }
    }
  }

  // draw bounding circle
  // or bounding box
  if (cntBallPoint > 100000)
  {  // if ball detected
    meanX /= cntBallPoint;
    meanY /= cntBallPoint;
    int radius = sqrt(cntBallPoint) * 0.59;
    // cv::circle(sub,Point(meanX,meanY),radius,cv::Scalar(0,0,255),3);
    int pointx = (meanX - boundX) * 2 + boundX;
    int pointy = (meanY - boundY) * 2 + boundY;
    cv::rectangle(sub, Point(boundX, boundY), Point(pointx, pointy), Scalar(0, 255, 0), 3);
  }

  // Publish the data.
  cv_bridge::CvImage out_msg;
  out_msg.header = msg->header;                           // Same timestamp and tf frame as input image
  out_msg.encoding = sensor_msgs::image_encodings::BGR8;  // Or whatever
  out_msg.image = sub;                                    // Your cv::Mat

  if (!using_rviz)
  {
    cv::imshow("binary", sub);
    cv::waitKey(30);
  }

  pub.publish(out_msg.toImageMsg());
}

int main(int argc, char** argv)
{
  // Init ROS
  ros::init(argc, argv, "ball_detection_node");
  ros::NodeHandle nh;

  if (!nh.getParam("/using_rviz", using_rviz))
  {
    using_rviz = false;
  }

  cv::namedWindow("bg", CV_WINDOW_NORMAL);
  cv::startWindowThread();

  cv::namedWindow("camera", CV_WINDOW_NORMAL);
  cv::startWindowThread();

  if (!using_rviz)
  {
    cv::namedWindow("binary", CV_WINDOW_NORMAL);
    cv::startWindowThread();
  }

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("camera/image_color", 1, imageCallback);

  // Create a ROS publisher for the output point cloud
  pub = it.advertise("output/ball", 1);

  ros::spin();
  cv::destroyAllWindows();
}