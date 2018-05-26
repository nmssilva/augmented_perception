/***********************************************************************************
Name:           chessboard.cpp
Revision:
Date:           05-10-2013
Author:         Paulo Dias
Comments:       ChessBoard Tracking


images
Revision:
Libraries:
Notes:          Code generated with Visual Studio 2013 Intel OpenCV 2.4.8 libraries 
***********************************************************************************/
#include <iostream>
#include <vector>
#include <ros/package.h>

// OpenCV Includes
#include <stdio.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;

// Function FindAndDisplayChessboard
// find corners in a cheesboard with board_w x board_h dimensions
// Display the corners in image and return number of detected corners
int FindAndDisplayChessboard(cv::Mat image, int board_w, int board_h, std::vector<cv::Point2f> *corners) {
	int board_size = board_w * board_h;
	CvSize board_sz = cvSize(board_w, board_h);

	cv::Mat grey_image;

	cv::cvtColor(image, grey_image, CV_BGR2GRAY);

	// find chessboard corners
	bool found = cv::findChessboardCorners(grey_image, board_sz, *corners, 0);

	// Draw results
	if (true) {
		cv::drawChessboardCorners(image, board_sz, cv::Mat(*corners), found);
		cv::imshow("Calibration", image);
		printf("\n Number of corners: %lu", corners->size());
		cv::waitKey(0);
	}
	return corners->size();
}

std::vector < cv::Point3f > Generate3DPoints ( int size )
{
	std::vector < cv::Point3f > points;
	points.push_back ( cv::Point3f ( 0, 0, 0 ) );
	points.push_back ( cv::Point3f ( size, 0, 0 ) );
	points.push_back ( cv::Point3f ( size, size, 0 ) );
	points.push_back ( cv::Point3f ( 0, size, 0 ) );
	points.push_back ( cv::Point3f ( 0, 0, size ) );
	points.push_back ( cv::Point3f ( size, 0, size ) );
	points.push_back ( cv::Point3f ( size, size, size ) );
	points.push_back ( cv::Point3f ( 0, size, size ) );
	return points;
}

int main(int argc, char **argv) {
	// ChessBoard Properties
	int n_boards = 13; //Number of images
	int board_w = 9;
	int board_h = 6;

	int board_sz = board_w * board_h;

	char filename[200];

	// Chessboard coordinates and image pixels
	std::vector<std::vector<cv::Point3f> > object_points;
	std::vector<std::vector<cv::Point2f> > image_points;

	// Corners detected in each image
	std::vector<cv::Point2f> corners;

	int corner_count;

	cv::Mat image;
	int i;

	int sucesses = 0;

	// chessboard coordinates
	std::vector<cv::Point3f> obj;
	for (int j = 0; j < board_sz; j++)
		obj.push_back(cv::Point3f(float(j / board_w), float(j % board_w), 0.0));

	for (i = 0; i < n_boards; i++) {
		// read image
		sprintf(filename, (ros::package::getPath("augmented_perception") + "/images/left%02d.jpg").c_str(), i + 1);
		printf("\nReading %s", filename);
		image = cv::imread(filename, CV_LOAD_IMAGE_COLOR);

		if (!image.data) {
			printf("\nCould not load image file: %s\n", filename);
			getchar();
			return 0;
		}

		// find and display corners
		corner_count = FindAndDisplayChessboard(image, board_w, board_h, &corners);

		if (corner_count == board_w * board_h) {
			image_points.push_back(corners);
			object_points.push_back(obj);
			sucesses++;
		}
	}

	cv::Mat intrinsic_matrix = cv::Mat(3, 3, CV_32FC1);
	cv::Mat distortion_coeffs;
	std::vector<cv::Mat> rotation_vectors;
	std::vector<cv::Mat> translation_vectors;

	calibrateCamera ( object_points, image_points, image.size (), intrinsic_matrix, distortion_coeffs, rotation_vectors, translation_vectors );

	std::cout << std::endl << "Intrinsics = "<< std::endl << " " << intrinsic_matrix << std::endl;

	std::cout << std::endl << "Distortion = "<< std::endl << " " << distortion_coeffs << std::endl;


	cout << distortion_coeffs.ptr(0,0) << endl;

	std::cout << std::endl << "Translations = "<< std::endl ;
	for (i=0;i<n_boards;i++)
		std::cout << translation_vectors.at(i) << std::endl;

	std::cout << std::endl << "Rotations= "<< std::endl ;
	for (i=0;i<n_boards;i++)
		std::cout << rotation_vectors.at(i) << std::endl;

	for ( i = 0; i < n_boards; i++ )
	{
		// read image
		sprintf(filename, (ros::package::getPath("augmented_perception") + "/images/left%02d.jpg").c_str(), i + 1);
		std::cout << "\n" << "Reading : " << filename << "\n";
		image = cv::imread ( filename, CV_LOAD_IMAGE_COLOR );
		// create cube points
		std::vector < cv::Point3f > o_points = Generate3DPoints ( 3 );
		// position cube
		std::vector < cv::Point2f > projectedPoints;
		cv::projectPoints ( o_points, rotation_vectors.at ( i ), translation_vectors.at ( i ), intrinsic_matrix, distortion_coeffs, projectedPoints );

		cout << o_points.at(0) << endl;
		cout << o_points.at(1) << endl;
		cout << o_points.at(2) << endl;
		cout << o_points.at(3) << endl;
		cout << o_points.at(4) << endl;
		cout << o_points.at(5) << endl;
		cout << o_points.at(6) << endl;
		cout << o_points.at(7) << endl;

		// create cube lines
		cv::line ( image, projectedPoints.at ( 0 ), projectedPoints.at ( 1 ), cv::Scalar ( 0, 255, 0 ), 2, 8 );
		cv::line ( image, projectedPoints.at ( 1 ), projectedPoints.at ( 2 ), cv::Scalar ( 0, 255, 0 ), 2, 8 );
		cv::line ( image, projectedPoints.at ( 2 ), projectedPoints.at ( 3 ), cv::Scalar ( 0, 255, 0 ), 2, 8 );
		cv::line ( image, projectedPoints.at ( 3 ), projectedPoints.at ( 0 ), cv::Scalar ( 0, 255, 0 ), 2, 8 );
		cv::line ( image, projectedPoints.at ( 1 ), projectedPoints.at ( 5 ), cv::Scalar ( 0, 255, 0 ), 2, 8 );
		cv::line ( image, projectedPoints.at ( 2 ), projectedPoints.at ( 6 ), cv::Scalar ( 0, 255, 0 ), 2, 8 );
		cv::line ( image, projectedPoints.at ( 5 ), projectedPoints.at ( 6 ), cv::Scalar ( 0, 255, 0 ), 2, 8 );
		cv::line ( image, projectedPoints.at ( 6 ), projectedPoints.at ( 7 ), cv::Scalar ( 0, 255, 0 ), 2, 8 );
		cv::line ( image, projectedPoints.at ( 5 ), projectedPoints.at ( 4 ), cv::Scalar ( 0, 255, 0 ), 2, 8 );
		cv::line ( image, projectedPoints.at ( 4 ), projectedPoints.at ( 0 ), cv::Scalar ( 0, 255, 0 ), 2, 8 );
		cv::line ( image, projectedPoints.at ( 7 ), projectedPoints.at ( 3 ), cv::Scalar ( 0, 255, 0 ), 2, 8 );
		cv::line ( image, projectedPoints.at ( 4 ), projectedPoints.at ( 7 ), cv::Scalar ( 0, 255, 0 ), 2, 8 );
		// show image
		cv::imshow ( "Projection", image );
		cv::waitKey ( 0 );
	}

	return 0;
}
