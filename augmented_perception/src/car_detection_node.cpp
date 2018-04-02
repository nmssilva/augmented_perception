#include <ros/package.h>
#include "laser_geometry/laser_geometry.h"
#include "message_filters/subscriber.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud.h"
#include "tf/message_filter.h"
#include "tf/transform_listener.h"

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <opencv2/opencv.hpp>

#include <sys/stat.h>
#include <ctime>
#include <iostream>
#include <string>

using namespace std;
using namespace cv;

// Publishers
ros::Publisher pub_scan_0;
image_transport::Publisher pub_image;

// Images
cv_bridge::CvImagePtr cv_ptr;
Mat image_input, imToShow, sub;

// Optical-flow related variables
Mat gray, prevGray, frame;
TermCriteria termcrit(TermCriteria::COUNT | TermCriteria::EPS, 20, 0.03);
vector<Point2f> points[2];
Point2f point;
bool needToInit = false;
bool addRemovePt = false;
const int MAX_COUNT = 500;
Size subPixWinSize(10, 10), winSize(31, 31);

// Template-Matching related variables
Mat patch, first_patch, result;
Point2f pointdown, pointup, pointbox;

/* 0: Squared Difference
 * 1: Normalized Squared Difference
 * 2: Cross Correlation
 * 3: Normalized Cross Correlation
 * 4: Cross Correlation Coefficient
 * 5: Normalized Cross Correlation Coefficient
 */
int match_method = CV_TM_SQDIFF;  // 0: CV_TM_SQDIFF 1: CV_TM_SQDIFF_NORMED 2: CV_TM_CCORR
                                  // 3: CV_TM_CCORR_NORMED 4: CV_TM_CCOEFF 5: CV_TM_CCOEFF_NORMED
int max_Trackbar = 5;
int nframes = 0;
bool capture = false;
bool drawRect = false;
bool got_last_patches = false;
std::queue<Mat> frame_array;
std::queue<Mat> previous_frames;
std::queue<Mat> first_previous_frames;
std::queue<Mat> previous_patches;

Mat MatchingMethod(int, void*, Mat patch_frame, Mat previous_frame)
{
  Mat img_display;
  previous_frame.copyTo(img_display);
  int result_cols = previous_frame.cols - patch_frame.cols + 1;
  int result_rows = previous_frame.rows - patch_frame.rows + 1;
  result.create(result_rows, result_cols, CV_32FC1);

  matchTemplate(previous_frame, patch_frame, result, match_method);

  normalize(result, result, 0, 1, NORM_MINMAX, -1, Mat());

  double minVal;
  double maxVal;
  Point minLoc;
  Point maxLoc;
  Point matchLoc;

  minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, Mat());

  if (match_method == TM_SQDIFF || match_method == TM_SQDIFF_NORMED)
  {
    matchLoc = minLoc;
  }
  else
  {
    matchLoc = maxLoc;
  }

  cv::Rect myROI(matchLoc.x, matchLoc.y, patch_frame.cols, patch_frame.rows);

  patch_frame = img_display(myROI);

  return patch_frame;
}

void MatchingMethod(int, void*)
{
  Mat img_display;
  sub.copyTo(img_display);
  int result_cols = sub.cols - patch.cols + 1;
  int result_rows = sub.rows - patch.rows + 1;
  result.create(result_rows, result_cols, CV_32FC1);

  matchTemplate(sub, patch, result, match_method);

  normalize(result, result, 0, 1, NORM_MINMAX, -1, Mat());

  double minVal;
  double maxVal;
  Point minLoc;
  Point maxLoc;
  Point matchLoc;

  minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, Mat());

  if (match_method == TM_SQDIFF || match_method == TM_SQDIFF_NORMED)
  {
    matchLoc = minLoc;
  }
  else
  {
    matchLoc = maxLoc;
  }

  cv::Rect myROI(matchLoc.x, matchLoc.y, patch.cols, patch.rows);
  patch = img_display(myROI);

  // limit 100
  if (frame_array.size() >= 100)
  {
    frame_array.pop();
  }

  frame_array.push(patch.clone());

  nframes++;
  rectangle(imToShow, matchLoc, Point(matchLoc.x + patch.cols, matchLoc.y + patch.rows), Scalar(0, 0, 255), 2, 8, 0);
  rectangle(result, matchLoc, Point(matchLoc.x + patch.cols, matchLoc.y + patch.rows), Scalar::all(0), 2, 8, 0);
  imshow("camera", imToShow);
  return;
}

static void onMouse_TM(int event, int x, int y, int /*flags*/, void* /*param*/)
{
  if (event == EVENT_LBUTTONDOWN)
  {
    pointdown = Point2f((float)x, (float)y);
    drawRect = true;
  }
  if (event == EVENT_LBUTTONUP)
  {
    pointup = Point2f((float)x, (float)y);

    if (x < 0)
      pointup.x = 0;

    if (y < 0)
      pointup.y = 0;

    if (x > image_input.cols)
      pointup.x = image_input.cols;

    if (y > image_input.rows)
      pointup.y = image_input.rows;

    capture = true;
    got_last_patches = false;
    nframes = 0;

    while (frame_array.size() > 0)
    {
      frame_array.pop();
    }
  }

  pointbox = Point2f((float)x, (float)y);
}

void image_cb_TemplateMatching(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    char c = (char)waitKey(10);

    if (c == 's')
    {
      int gotframes = nframes;
      if (gotframes > 100)
        gotframes = 100;

      nframes = 0;

      if (gotframes == 0 || patch.empty())
      {
        ROS_INFO("There are no frames to save.");
      }
      else
      {
        ROS_INFO("Loading templates to save...");

        // get 5 last patches
        if (!got_last_patches)
        {
          Mat previous_patch = first_patch;
          for (int i = 0; i < 5; i++)
          {
            previous_patch = MatchingMethod(0, 0, previous_patch, first_previous_frames.front());
            first_previous_frames.pop();
            previous_patches.push(previous_patch);
          }
          got_last_patches = true;
        }

        ROS_INFO("Saving %s frames. Please enter object label: (type 'exit' to discard frames)",
                 boost::lexical_cast<std::string>(gotframes + 5).c_str());

        string path;
        path = ros::package::getPath("augmented_perception") + "/labelling/";
        mkdir(path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

        cin >> path;

        if (path.find("exit"))
        {
          path = ros::package::getPath("augmented_perception") + "/labelling/" + path;
          mkdir(path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

          path += "/" + boost::lexical_cast<std::string>(std::time(NULL));
          mkdir(path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

          for (int i = 0; i < gotframes; i++)
          {
            string impath;
            impath = path + "/" + boost::lexical_cast<std::string>(i + 1) + ".bmp";
            imwrite(impath, frame_array.front());
            frame_array.pop();
          }

          int i = 0;
          while (!previous_patches.empty())
          {
            i++;
            string impath;
            impath = path + "/previous_" + boost::lexical_cast<std::string>(i) + ".bmp";
            imwrite(impath, previous_patches.front());
            previous_patches.pop();
            previous_frames.pop();
          }

          ROS_INFO("Saved %s frames to %s", boost::lexical_cast<std::string>(gotframes + 5).c_str(), path.c_str());
        }
        else
        {
          cout << "Did not save\n";
        }
      }
    }
    if (c == 'c')
    {
      patch = Mat();
      ROS_INFO("Image cleared");
    }

    // Show image_input
    image_input = cv_ptr->image;
    imToShow = image_input.clone();
    sub = image_input.clone();

    // previous 5 frames
    if (previous_frames.size() >= 5)
    {
      previous_frames.pop();
    }

    previous_frames.push(image_input);

    if (drawRect)
    {
      // Draw area-to-crop rectangle
      float max_x, min_x, max_y, min_y;
      if (pointbox.x > pointdown.x)
      {
        min_x = pointdown.x;
        max_x = pointbox.x;
      }
      else
      {
        max_x = pointdown.x;
        min_x = pointbox.x;
      }
      if (pointbox.y > pointdown.y)
      {
        min_y = pointdown.y;
        max_y = pointbox.y;
      }
      else
      {
        max_y = pointdown.y;
        min_y = pointbox.y;
      }

      cv::rectangle(imToShow, Point((int)min_x, (int)min_y), Point((int)max_x, (int)max_y), Scalar(0, 255, 0), 3);
    }

    cv::imshow("camera", imToShow);
    cvSetMouseCallback("camera", onMouse_TM, 0);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }

  float max_x, min_x, max_y, min_y;
  if (pointup.x > pointdown.x)
  {
    min_x = pointdown.x;
    max_x = pointup.x;
  }
  else
  {
    max_x = pointdown.x;
    min_x = pointup.x;
  }
  if (pointup.y > pointdown.y)
  {
    min_y = pointdown.y;
    max_y = pointup.y;
  }
  else
  {
    max_y = pointdown.y;
    min_y = pointup.y;
  }

  if (max_x - min_x > 0 && max_y - min_y > 0 && capture)
  {
    cv::Rect myROI(min_x, min_y, max_x - min_x, max_y - min_y);
    patch = image_input(myROI);
    first_patch = patch.clone();
    while (!previous_frames.empty())
    {
      first_previous_frames.push(previous_frames.front());
      previous_frames.pop();
    }
    imshow("first_patch", first_patch);
    capture = false;
    drawRect = false;
  }

  // up half ignore
  for (int y = 0; y < sub.rows / 3; y++)
  {
    for (int x = 0; x < sub.cols; x++)
    {
      sub.at<Vec3b>(Point(x, y))[0] = 0;
      sub.at<Vec3b>(Point(x, y))[1] = 0;
      sub.at<Vec3b>(Point(x, y))[2] = 0;
    }
  }

  if (!patch.empty())
  {
    imshow("crop", patch);
    MatchingMethod(0, 0);
  }
}

void scan_0_cb(const sensor_msgs::LaserScan::ConstPtr& input)
{
  laser_geometry::LaserProjection projector;
  tf::TransformListener listener;

  int n_pos = (input->angle_max - input->angle_min) / input->angle_increment;

  // Create a container for the data.
  sensor_msgs::LaserScan output;
  output = *input;

  // Do something with cloud.

  // Publish the data.
  pub_scan_0.publish(output);
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

  cv::namedWindow("crop", CV_WINDOW_NORMAL);
  cv::startWindowThread();

  // Create a ROS subscriber for the inputs
  image_transport::Subscriber sub_image = it.subscribe("/camera/image_color", 1, image_cb_TemplateMatching);
  ros::Subscriber sub_scan_E = nh.subscribe("/ld_rms/scan0", 1, scan_0_cb);

  // Create a ROS publisher for the output point cloud
  pub_image = it.advertise("output/image_input", 1);
  pub_scan_0 = nh.advertise<sensor_msgs::LaserScan>("/output/scan_0", 1);

  // Spin
  ros::spin();
  cv::destroyAllWindows();
}