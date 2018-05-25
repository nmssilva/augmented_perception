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

#include <cmath>
#include <cstdlib>
#include <iostream>

// OpenCV Includes
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
// General Includes
#include <iostream>

using namespace std;
using namespace cv;

Mat image_input, projection;

std::vector<cv::Point3f> objectPoints;

cv::Mat rvec = cv::Mat(3, 1, CV_32FC1);
cv::Mat tvec = cv::Mat(3, 1, CV_32FC1);

cv::Mat distCoeffs = cv::Mat(5, 1, CV_32FC1);
cv::Mat cameraMatrix = cv::Mat(3, 3, CV_32FC1);

/*
 * Creates a vector of points that represent a cube with given side dimension
 */
std::vector<cv::Point3f> Generate3DPoints(float size, float x, float y) {
	std::vector<cv::Point3f> points;
	points.push_back(cv::Point3f(y+size, size/2, 0));//x-size));
	points.push_back(cv::Point3f(y+size, size/2, 0));//x+size));
	points.push_back(cv::Point3f(y-size, size/2, 0));//x-size));
	points.push_back(cv::Point3f(y-size, size/2, 0));//x+size));
	points.push_back(cv::Point3f(y+size, -size/2, 0));//x-size));
	points.push_back(cv::Point3f(y+size, -size/2, 0));//x+size));
	points.push_back(cv::Point3f(y-size, -size/2, 0));//x-size));
	points.push_back(cv::Point3f(y-size, -size/2, 0));//x+size));
	return points;
}

int main(int argc, char **argv) {

	image_input = cv::imread(ros::package::getPath("augmented_perception") + "/frames/frame0069.jpg",
							 CV_LOAD_IMAGE_COLOR);

	cv::namedWindow("image", CV_WINDOW_NORMAL);
	cv::resizeWindow("image", 800, 666);

	image_input.copyTo(projection);

	for (int i = 0; i < 3; i++) {
		rvec.at<int>(i) = 0;
		tvec.at<int>(i) = 10;
	}

	distCoeffs.at<int>(0) = -0.2015966527847064;
	distCoeffs.at<int>(1) = 0.1516937421259596;
	distCoeffs.at<int>(2) = -0.0009340794635090795;
	distCoeffs.at<int>(3) = -0.0006787308984611241;
	distCoeffs.at<int>(4) = 0;

	cameraMatrix.at<int>(0) = 1454.423376687359; //fx
	cameraMatrix.at<int>(4) = 1458.005828758985; //fy
	cameraMatrix.at<int>(2) = 822.9545738617143; //cx
	cameraMatrix.at<int>(5) = 590.5652711935882; //cy
	cameraMatrix.at<int>(8) = 1;

	// create cube points
	std::vector<cv::Point3f> o_points = Generate3DPoints(1.5, 8., 0.);
	// position cube
	std::vector<cv::Point2f> projectedPoints;
	cv::projectPoints(o_points, rvec, tvec, cameraMatrix, distCoeffs, projectedPoints);
	// create cube lines
	cout << projectedPoints.at(0) << endl;
	cout << projectedPoints.at(1) << endl;
	cout << projectedPoints.at(2) << endl;
	cout << projectedPoints.at(3) << endl;
	cout << projectedPoints.at(4) << endl;
	cout << projectedPoints.at(5) << endl;
	cout << projectedPoints.at(6) << endl;
	cout << projectedPoints.at(7) << endl;

	cout << o_points.at(0) << endl;
	cout << o_points.at(1) << endl;
	cout << o_points.at(2) << endl;
	cout << o_points.at(3) << endl;
	cout << o_points.at(4) << endl;
	cout << o_points.at(5) << endl;
	cout << o_points.at(6) << endl;
	cout << o_points.at(7) << endl;

	for(int i = 0; i < projectedPoints.size(); i++){
		projectedPoints.at(i).y += ((cameraMatrix.at<int>(2)+cameraMatrix.at<int>(5))/2);
		projectedPoints.at(i).x = cameraMatrix.at<int>(2)-projectedPoints.at(i).x;
	}

	cv::line(image_input, projectedPoints.at(0), projectedPoints.at(1), cv::Scalar(255, 0, 0), 2, 8);
	cv::line(image_input, projectedPoints.at(1), projectedPoints.at(2), cv::Scalar(255, 0, 0), 2, 8);
	cv::line(image_input, projectedPoints.at(2), projectedPoints.at(3), cv::Scalar(255, 0, 0), 2, 8);
	cv::line(image_input, projectedPoints.at(3), projectedPoints.at(0), cv::Scalar(255, 0, 0), 2, 8);
	cv::line(image_input, projectedPoints.at(1), projectedPoints.at(5), cv::Scalar(0, 255, 0), 2, 8);
	cv::line(image_input, projectedPoints.at(2), projectedPoints.at(6), cv::Scalar(0, 255, 0), 2, 8);
	cv::line(image_input, projectedPoints.at(5), projectedPoints.at(6), cv::Scalar(0, 0, 255), 2, 8);
	cv::line(image_input, projectedPoints.at(6), projectedPoints.at(7), cv::Scalar(0, 0, 255), 2, 8);
	cv::line(image_input, projectedPoints.at(5), projectedPoints.at(4), cv::Scalar(0, 0, 255), 2, 8);
	cv::line(image_input, projectedPoints.at(4), projectedPoints.at(0), cv::Scalar(0, 255, 0), 2, 8);
	cv::line(image_input, projectedPoints.at(7), projectedPoints.at(3), cv::Scalar(0, 255, 0), 2, 8);
	cv::line(image_input, projectedPoints.at(4), projectedPoints.at(7), cv::Scalar(0, 0, 255), 2, 8);

	while (true) {
		imshow("image", image_input);
		waitKey(1);
	}
}