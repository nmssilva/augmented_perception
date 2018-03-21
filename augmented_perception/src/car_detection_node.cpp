#include "laser_geometry/laser_geometry.h"
#include "message_filters/subscriber.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud.h"
#include "tf/message_filter.h"
#include "tf/transform_listener.h"

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <opencv2/opencv.hpp>

#include <iostream>

using namespace std;
using namespace cv;

ros::Publisher pub_scan_E;
image_transport::Publisher pub_image;
cv_bridge::CvImagePtr cv_ptr;

Mat gray, prevGray, image, frame;

TermCriteria termcrit(TermCriteria::COUNT | TermCriteria::EPS, 20, 0.03);
vector<Point2f> points[2];
Point2f point;

bool paused = false;
bool needToInit = false;
bool addRemovePt = false;

const int MAX_COUNT = 500;

Size subPixWinSize(10, 10), winSize(31, 31);

void scan_E_cb(const sensor_msgs::LaserScan::ConstPtr& input)
{
  laser_geometry::LaserProjection projector;
  tf::TransformListener listener;

  int n_pos = (input->angle_max - input->angle_min) / input->angle_increment;

  // Create a container for the data.
  sensor_msgs::LaserScan output;
  output = *input;

  // Do something with cloud.

  int lb_front_pos = 275;
  int lb_rear_pos = 400;
  int rb_front_pos = 100;
  int rb_rear_pos = 60;

  float b_threshold = 2;
  float lb_front = output.ranges[lb_front_pos] - b_threshold;
  float lb_rear = output.ranges[lb_rear_pos] - b_threshold;
  float rb_front = output.ranges[rb_front_pos] - b_threshold;
  float rb_rear = output.ranges[rb_rear_pos] - b_threshold;

  float lb_increment = (lb_front - lb_rear) / (lb_rear_pos - lb_front_pos);
  float rb_increment = (rb_front - rb_rear) / (rb_rear_pos - rb_front_pos);

  for (int i = 0; i <= n_pos; i++)
  {
    if (i >= lb_front_pos && i <= lb_rear_pos)
    {
      float max_range = lb_rear + (lb_increment * (i - lb_front_pos));
      output.ranges[i] = max_range;
    }
    else
    {
      output.ranges[i] = 0;
    }
  }

  // Publish the data.
  pub_scan_E.publish(output);
}

static void onMouse(int event, int x, int y, int /*flags*/, void* /*param*/)
{
  if (event == EVENT_LBUTTONDOWN)
  {
    point = Point2f((float)x, (float)y);
    addRemovePt = true;
  }
}

void image_cb(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    char c = (char)waitKey(10);

    switch (c)
    {
      case 'r':
        needToInit = true;
        break;
      case 'c':
        points[0].clear();
        points[1].clear();
        break;
    }

    // Show image
    image = cv_ptr->image;

    if (!paused)
    {
      cvSetMouseCallback("camera", onMouse, 0);
    }
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }

  // up half ignore
  for (int y = 0; y < image.rows / 3; y++)
  {
    for (int x = 0; x < image.cols; x++)
    {
      image.at<Vec3b>(Point(x, y))[0] = 0;
      image.at<Vec3b>(Point(x, y))[1] = 0;
      image.at<Vec3b>(Point(x, y))[2] = 0;
    }
  }

  cvtColor(image, gray, COLOR_BGR2GRAY);

  if (needToInit)
  {
    // automatic initialization
    goodFeaturesToTrack(gray, points[1], MAX_COUNT, 0.01, 10, Mat(), 3, 3, 0, 0.04);
    cornerSubPix(gray, points[1], subPixWinSize, Size(-1, -1), termcrit);
    addRemovePt = false;
  }
  else if (!points[0].empty())
  {
    vector<uchar> status;
    vector<float> err;
    if (prevGray.empty())
      gray.copyTo(prevGray);

    calcOpticalFlowPyrLK(prevGray, gray, points[0], points[1], status, err, winSize, 3, termcrit, 0, 0.001);

    size_t i, k;
    for (i = k = 0; i < points[1].size(); i++)
    {
      if (addRemovePt)
      {
        if (norm(point - points[1][i]) <= 5)
        {
          addRemovePt = false;
          continue;
        }
      }

      if (!status[i])
        continue;

      points[1][k++] = points[1][i];
      circle(image, points[1][i], 20, Scalar(0, 255, 0), -1, 8);
    }
    points[1].resize(k);
  }
  if (addRemovePt && points[1].size() < (size_t)MAX_COUNT)
  {
    vector<Point2f> tmp;
    tmp.push_back(point);
    cornerSubPix(gray, tmp, winSize, Size(-1, -1), termcrit);
    points[1].push_back(tmp[0]);
    addRemovePt = false;
  }

  std::swap(points[1], points[0]);
  cv::swap(prevGray, gray);
  needToInit = false;

  // Publish data
  cv::imshow("camera", image);

  cv_bridge::CvImage out_msg;
  out_msg.header = msg->header;                           // Same timestamp and tf frame as input image
  out_msg.encoding = sensor_msgs::image_encodings::BGR8;  // Or whatever
  out_msg.image = image;                                  // Your cv::Mat

  pub_image.publish(out_msg.toImageMsg());
}

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "car_detection_node");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);

  // Create Camera Windows
  cv::namedWindow("camera", CV_WINDOW_NORMAL);
  cv::startWindowThread();

  // Create a ROS subscriber for the inputs
  image_transport::Subscriber sub_image = it.subscribe("/camera/image_color", 1, image_cb);
  ros::Subscriber sub_scan_E = nh.subscribe("/lms151_E_scan", 1, scan_E_cb);

  // Create a ROS publisher for the output point cloud
  pub_image = it.advertise("output/image", 1);
  pub_scan_E = nh.advertise<sensor_msgs::LaserScan>("/output/scan_E", 1);

  // Spin
  ros::spin();
  cv::destroyAllWindows();
}