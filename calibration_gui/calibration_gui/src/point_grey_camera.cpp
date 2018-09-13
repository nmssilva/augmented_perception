#include <cstring>
#include <iostream>
#include <string>
#include "calibration_gui/point_grey_camera.h"

// Marker's publisher
ros::Publisher ballCentroidCam_pub;
ros::Publisher ballCentroidCamPnP_pub;
image_transport::Publisher ballCentroidImage_pub;

Mat CameraMatrix1, disCoeffs1;

int lowH;
int highH;
int lowS;
int highS;
int lowV;
int highV;
int valMinDist;
int valC;
int valA;
int maxR;
int minR;

bool firstframe;
cv::Mat bg;
cv::Mat click;
uchar hue = 2;

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

void PublishBallCenter(Mat &img, Mat &imgBinary, int centerX, int centerY, int boundX, int boundY)
{
  vector<vector<Point> > contours;
  Mat imgCanny;
  Canny(imgBinary, imgCanny, 100, 100 * 2,
        3);  // The canny threshold does not have significant effect on ball detection, it set at 100

  findContours(imgCanny, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

  // imshow("Test", imgCanny);

  vector<Point> approx;
  Mat dst = img.clone();
  vector<double> areas;

  pcl::PointXYZ centroid;  // Point structure for centroid
                           /* The following lines make sure that position (0,0,0) is published when
                            the ball is not detected (avoids publishing previous positions when the
                            ball is no longer detected) */
  centroid.x = 0;
  centroid.y = 0;
  centroid.z = 0;

  pcl::PointXYZ centroidRadius;  // Point structure for centroid
                                 /* The following lines make sure that position (0,0,0) is published when
                                  the ball is not detected (avoids publishing previous positions when the
                                  ball is no longer detected) */
  centroidRadius.x = 0;
  centroidRadius.y = 0;
  centroidRadius.z = 0;

  int radius = ((centerX - boundX) + (centerY - boundY)) / 2;

  centroid.x = centerX;
  centroid.y = centerY;
  centroid.z = radius;

  // Computing distnace from camera to center of detected circle
  double Dist, f_avg;

  f_avg = (CameraMatrix1.at<double>(0, 0) + CameraMatrix1.at<double>(1, 1)) / 2;

  Dist = (f_avg * (BALL_DIAMETER / (radius * 2)));

  // Method based on circle radius =======================================
  cv::Mat image_vector = (cv::Mat_<double>(3, 1) << centroid.x, centroid.y, 1);
  cv::Mat camera_vector = CameraMatrix1;

  camera_vector = CameraMatrix1.inv() * (Dist * image_vector);
  // cout << camera_vector << endl; // DEBUGGING
  // cout << CameraMatrix1.inv() << endl; // DEBUGGING
  centroidRadius.x = camera_vector.at<double>(0);
  centroidRadius.y = camera_vector.at<double>(1);
  centroidRadius.z = camera_vector.at<double>(2);

  CentroidPub(centroid, centroidRadius);
}

void mouseHandler(int event, int x, int y, int flags, void *param)
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

/**
   @brief Image processing and ball detection
   @param[in] img image captured by the Point Grey camera
   @return void
 */
void ImageProcessing(Mat &img)
{
  int key = -1;
  Mat image = (Mat)img;
  click = image.clone();
  cvSetMouseCallback("Camera", mouseHandler, &click);
  cv::imshow("Camera", image);
  key = cv::waitKey(30);

  if (firstframe || key == 32 /*spacebar*/)
  {  // define bg
    firstframe = false;
    bg = image.clone();
    cv::imshow("Background", bg);
    blur(bg, bg, cv::Size(20, 20));
    cv::waitKey(30);
  }

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
      if (y == sub.rows / 2)
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

    // int radius = sqrt(cntBallPoint) * 0.59;
    // cv::circle(sub, Point(meanX, meanY), radius, cv::Scalar(0, 0, 255), 3);
    int pointx = (meanX - boundX) * 2 + boundX;
    int pointy = (meanY - boundY) * 2 + boundY;

    // Publish the data
    PublishBallCenter(image, sub, meanX, meanY, boundX, boundY);

    cv::rectangle(sub, Point(boundX, boundY), Point(pointx, pointy), Scalar(0, 255, 0), 3);
  }

  // Show image
  cv::imshow("Binary", sub);
  cv::waitKey(30);
}

/**
   @brief Publishes the detected ball center
   @param[in] centroid detected ball center in pixels
   @param[in] centroidRadius detected ball center in the camera frame
   @return void
 */
void CentroidPub(const pcl::PointXYZ centroid, const pcl::PointXYZ centroidRadius)
{
  // Method based on solvePnP ================================================
  geometry_msgs::PointStamped CentroidCam;

  CentroidCam.point.x = centroid.x / 500 - 1.5;
  CentroidCam.point.y = centroid.y / 200 - 2.5;
  CentroidCam.point.z = centroid.z;

  CentroidCam.header.stamp = ros::Time::now();
  ballCentroidCamPnP_pub.publish(CentroidCam);

  // ROS_INFO("(%f,%f,%f)", centroid.x, centroid.y, centroid.z);

  // Method based on circle radius ===========================================
  CentroidCam.point.x = centroidRadius.x;
  CentroidCam.point.y = centroidRadius.y;
  CentroidCam.point.z = centroidRadius.z;

  CentroidCam.header.stamp = ros::Time::now();
  ballCentroidCam_pub.publish(CentroidCam);
  // std::cout << CentroidCam << std::endl;
}

/**
   @brief Creates a circle that represents the detected ball and its center
   @param[out] im undistorted captured image
   @param[in] label text to write (currently not used)
   @param[in] contour detected ball contour
   @return void
 */
void setLabel(cv::Mat &im, const std::string label, std::vector<cv::Point> &contour)
{
  int fontface = cv::FONT_HERSHEY_SIMPLEX;
  double scale = 0.8;
  int thickness = 1;
  int baseline = 0;

  cv::Size text = cv::getTextSize(label, fontface, scale, thickness, &baseline);
  cv::Rect r = cv::boundingRect(contour);
  int radius = (r.width / 2 + r.height / 2) / 2;

  cv::Point pt(r.x + ((r.width - text.width) / 2), r.y + ((r.height + text.height) / 2));
  cv::Point center(r.x + ((r.width) / 2), r.y + ((r.height) / 2));
  // cv::rectangle(im, pt + cv::Point(0, baseline), pt + cv::Point(text.width, -text.height), CV_RGB(255,255,255),
  // CV_FILLED);
  // cv::putText(im, label, pt, fontface, scale, CV_RGB(0,0,0), thickness, 8);
  // circle( im, center, 4, Scalar(255,0,0), -1, 8, 0 );
  cv::line(im, cv::Point(center.x - 7, center.y), cv::Point(center.x + 7, center.y), cv::Scalar(255, 255, 255),
           2);  // crosshair horizontal
  cv::line(im, cv::Point(center.x, center.y - 7), cv::Point(center.x, center.y + 7), cv::Scalar(255, 255, 255),
           2);  // crosshair vertical
  // circle outline
  circle(im, center, radius, Scalar(255, 255, 255), 2, 8, 0);

  // imshow("Test", im);
  // cv::waitKey(30);

  sensor_msgs::ImagePtr image_msg;
  image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", im).toImageMsg();
  ballCentroidImage_pub.publish(image_msg);
}

/**
   @brief Main function of the ball detection node for Point Grey camera
   @param argc
   @param argv
   @return int
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "Point_Grey");

  ros::NodeHandle n("~");
  string node_ns = ros::this_node::getNamespace();
  node_ns.erase(0, 2);
  n.getParam("ballDiameter", BALL_DIAMETER);

  cout << "Node namespace:" << node_ns << endl;
  cout << "Ball diameter:" << BALL_DIAMETER << endl;
  // read calibration paraneters
  string a = "/intrinsic_calibrations/ros_calib.yaml";
  string path = ros::package::getPath("calibration_gui");
  path += a;
  FileStorage fs(path, FileStorage::READ);
  if (!fs.isOpened())
  {
    cout << "failed to open document" << endl;
    return -1;
  }

  fs["CM1"] >> CameraMatrix1;
  fs["D1"] >> disCoeffs1;
  std::cout << CameraMatrix1 << std::endl;
  std::cout << disCoeffs1 << std::endl;

  image_transport::ImageTransport it(n);

  string raw_data_topic = "/" + node_ns;

  string ballDetection_topic = raw_data_topic + "/BD_" + node_ns;

  ballCentroidImage_pub = it.advertise(ballDetection_topic + "/BallDetection", 1);
  ballCentroidCam_pub = n.advertise<geometry_msgs::PointStamped>(ballDetection_topic + "/SphereCentroid", 1);
  ballCentroidCamPnP_pub = n.advertise<geometry_msgs::PointStamped>(ballDetection_topic + "/SphereCentroidPnP", 1);

  cv::namedWindow("Background", CV_WINDOW_NORMAL);
  cv::startWindowThread();

  cv::namedWindow("Camera", CV_WINDOW_NORMAL);
  cv::startWindowThread();

  cv::namedWindow("Binary", CV_WINDOW_NORMAL);
  cv::startWindowThread();

  firstframe = true;

  CameraRaw cameraRaw(node_ns);

  ros::Rate loop_rate(15);

  std::cout << "test" << std::endl;
  while (ros::ok())
  {
    if (!cameraRaw.camImage.empty())
    {
      ImageProcessing(cameraRaw.camImage);
    }
    ros::spinOnce();
  }

  ROS_INFO("OUT");
  // destroy the windows
  cv::destroyAllWindows();
  return 0;
}
