#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.h>

#include <ros/package.h>
#include <ros/ros.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <stdio.h>
#include <fstream>

using namespace std;
using namespace cv;

image_transport::Publisher pub;

cv_bridge::CvImagePtr cv_ptr;
Mat image;

struct BBox
{
  int x;
  int y;
  int width;
  int height;
  int id;
  string label;
};

string filename;
std::map<unsigned int, std::vector<BBox> > file_map;
bool foundframe = false;

void initializeFileMap()
{
  string path = ros::package::getPath("augmented_perception") + "/datasets/" + filename;
  std::ifstream infile(path.c_str());

  string line;
  unsigned int frame_id;

  while (std::getline(infile, line))
  {
    if (line.find(" ") != std::string::npos)  // BBox
    {
      BBox box;
      int x, y, width, height, id;
      string label;

      sscanf(line.substr(0, line.find(" ")).c_str(), "%d", &x);
      line = line.substr(line.find(" ") + 1, line.length());
      sscanf(line.substr(0, line.find(" ")).c_str(), "%d", &y);
      line = line.substr(line.find(" ") + 1, line.length());
      sscanf(line.substr(0, line.find(" ")).c_str(), "%d", &width);
      line = line.substr(line.find(" ") + 1, line.length());
      sscanf(line.substr(0, line.find(" ")).c_str(), "%d", &height);
      line = line.substr(line.find(" ") + 1, line.length());
      label = line.substr(0, line.find(" "));
      line = line.substr(line.find(" ") + 1, line.length());
      sscanf(line.substr(0, line.find(" ")).c_str(), "%d", &id);

      box.x = x;
      box.y = y;
      box.width = width;
      box.height = height;
      box.id = id;
      box.label = label;

      file_map[frame_id].push_back(box);
    }
    else
    {  // Frame
      sscanf(line.c_str(), "%d", &frame_id);
    }
  }
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }

  // Show image_input
  image = cv_ptr->image;

  // Your cv::Mat

  std::vector<BBox> boxes;

  unsigned int actual_frame_id = cv_ptr->header.seq;

  for (int i = 0; i < 5; i++)
  {
    if (file_map[actual_frame_id - i].size() > 0)
    {
      boxes = file_map[actual_frame_id];
      foundframe = true;
      break;
    }
    foundframe = false;
  }

  if (foundframe)
  {
    for (int i = 0; i < boxes.size(); i++)
    {
      BBox box = boxes[i];
      // ROS_INFO("%d %d %d %d", box.x, box.y, box.width, box.height);
      cv::rectangle(image, Point(box.x, box.y), Point(box.x + box.width, box.y + box.height), Scalar(0, 255, 0), 3);
    }
  }

  imshow("camera", image);

  char c = (char)waitKey(10);

  // Publish the data.
  cv_bridge::CvImage out_msg;
  out_msg.header = msg->header;                           // Same timestamp and tf frame as input image
  out_msg.encoding = sensor_msgs::image_encodings::BGR8;  // Or whatever
  out_msg.image = image;

  pub.publish(out_msg.toImageMsg());
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dataset_playback_node");
  ros::NodeHandle nh;

  cv::namedWindow("camera", CV_WINDOW_NORMAL);
  cv::startWindowThread();

  if (argc < 2)
  {
    cout << "usage: rosrun augmented_perception dataset_playback_node <filename>\n";
    exit(0);
  }
  else
  {
    filename = argv[1];
    initializeFileMap();
  }

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/camera/image_color", 1, imageCallback);

  // Create a ROS publisher for the output point cloud
  pub = it.advertise("output/image", 1);

  ros::spin();
  cv::destroyAllWindows();
}