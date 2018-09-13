/**************************************************************************************************
   Software License Agreement (BSD License)

   Copyright (c) 2014-2015, LAR toolkit developers - University of Aveiro - http://lars.mec.ua.pt
   All rights reserved.

   Redistribution and use in source and binary forms, with or without modification, are permitted
   provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this list of
   conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of
   conditions and the following disclaimer in the documentation and/or other materials provided
   with the distribution.
 * Neither the name of the University of Aveiro nor the names of its contributors may be used to
   endorse or promote products derived from this software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
   IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
   FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
   DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
   IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
   OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ***************************************************************************************************/
/**
   \file  gui_calibration_node.cpp
   \brief Calibration node class. Performs the actual extrinsic calibration of the chosen sensors
   \author David Silva
   \date   July, 2016
 */

#define _NODE_CPP_

// ROS includes
#include <ros/ros.h>
#include <ros/network.h>

// Project includes
#include "calibration_gui/gui_calibration_node.h"
#include "calibration_gui/gui_mainwindow.h"
#include "ui_mainwindow.h"
#include "calibration_gui/calibration.h"
#include "calibration_gui/visualization_rviz_calibration.h"

// Generic includes
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include <QDebug>

/**
   @brief QNode class constructor
   @param argc master URL
   @param argv host URL
   @param name node name
 */
QNode::QNode(int argc, char** argv, const std::string &name ) :
	init_argc(argc),
	init_argv(argv),
	node_name(name)
{
	connect(this, SIGNAL(showMsg(const QString&,QMessageBox::StandardButton*)),
	        this, SLOT(msgShower(const QString&,QMessageBox::StandardButton*)));

}

/**
   @brief QNode class destructor.
   See http://docs.ros.org/electric/api/qt_tutorials/html/qnode_8cpp_source.html
 */
QNode::~QNode() {
	shutdown();
}


/**
   @brief Method called by the qt application to stop the ros node before the qt app closes.
   See http://docs.ros.org/electric/api/qt_tutorials/html/qnode_8cpp_source.html
   @param void
   @return void
 */
void QNode::shutdown() {
	if(ros::isStarted()) {
		ros::shutdown(); // explicitly needed since we use ros::start();
		ros::waitForShutdown();
	}
	wait();
}

/**
   @brief Method to initialize the ROS node
   See http://docs.ros.org/electric/api/qt_tutorials/html/qnode_8cpp_source.html
   @param void
   @return true on success, false otherwise
 */
bool QNode::on_init() {
	ros::init(init_argc,init_argv,node_name);
	if ( !ros::master::check() ) {
		return false;
	}
	ros::start(); // our node handles go out of scope, so we want to control shutdown explicitly.
	//start();
	return true;
}

/**
   @brief Method to initialize the ROS node with configurable initialisation master and host URLs
   See http://docs.ros.org/electric/api/qt_tutorials/html/qnode_8cpp_source.html
   @param[in] master_url master URL
   @param[in] host_url host URL
   @return true on success, false otherwise
 */
bool QNode::on_init(const std::string &master_url, const std::string &host_url) {
	std::map<std::string,std::string> remappings;
	remappings["__master"] = master_url;
	remappings["__hostname"] = host_url;
	ros::init(remappings,node_name);
	if ( !ros::master::check() ) {
		return false;
	}
	ros::start(); // our node handles go out of scope, so we want to control shutdown explicitly.
	//start();
	return true;
}

/**
   @brief Function called after the thread is started. It executes the calibration process; acquiring data from the chosen sensors and then computing their extrinsic transformations relative to a reference sensor.
   @param void
   @return void
 */
void QNode::run() {
	ros::Publisher markers_pub;
	//ros::Publisher car_pub;

	ros::NodeHandle n;

	markers_pub = n.advertise<visualization_msgs::MarkerArray>( node_name + "/CalibrationPoints", 10000);
	//car_pub = n.advertise<visualization_msgs::Marker>(node_name + "/3DModel", 1);

	CircleCentroids centroids(calibrationNodes, isCamera);

	vector<pcl::PointXYZ> sensorsBallCenters;
	vector<pcl::PointXYZ> camCentroidPnP;
	vector<cv::Mat> camImage;

	// Vector for containing future pointclouds for each sensor
	vector<pcl::PointCloud<pcl::PointXYZ> > sensorClouds;
	// Vector for containing image pointclouds from cameras - solvePnP method
	vector<pcl::PointCloud<pcl::PointXYZ> > cameraCloudsPnP;
	for (int i=0; i < calibrationNodes.size(); i++)
	{
		pcl::PointCloud<pcl::PointXYZ> sensorCloud;
		sensorClouds.push_back(sensorCloud);
		if (isCamera[i])
		{
			pcl::PointCloud<pcl::PointXYZ> cameraCloudPnP;
			cameraCloudsPnP.push_back(cameraCloudPnP);
		}
	}

	// Vector for containing future sensor poses
	vector<geometry_msgs::Pose> sensorPoses;
	// Vector for containing future sensor poses
	vector<geometry_msgs::Pose> cameraPosesPnP;
	for (int i=0; i < calibrationNodes.size(); i++)
	{
		geometry_msgs::Pose sensorPose;
		sensorPoses.push_back(sensorPose);
		if (isCamera[i])
		{
			geometry_msgs::Pose cameraPosePnP;
			cameraPosesPnP.push_back(cameraPosePnP);
		}
	}
	sensorPoses.front().orientation.w = 1.0; // so it can be multiplied by transformations later, only the reference sensor needs this

	// Pointclouds used for Rviz visualization
	vector<geometry_msgs::Pose> visualizationPoses;
	vector<pcl::PointCloud<pcl::PointXYZ> > visualizationClouds;


	int count=0;

	createDirectory( );

	qDebug() << "Calibration is going to start";

	float diff_displacement;
	float reference_dist;
	float dist;
	vector<float> eu_dist;

	ros::Rate loop_rate(50);

	while(count < num_of_points && ros::ok() && doCalibration)
	{
		sensorsBallCenters = centroids.getSensorsBallCenters();
		camCentroidPnP = centroids.getCamCentroidPnP();
		camImage = centroids.getCamImage();

		// qDebug() << sensorsBallCenters.size();

		int finder = 0;
		bool found = false;
		while ( finder < sensorsBallCenters.size() )
		{
			if (sensorsBallCenters[finder].x == -999)
			{
				found = true;
				break;
			}
			finder++;
		}
		finder=0;
		while ( finder < camCentroidPnP.size() && !found )
		{
			if (camCentroidPnP[finder].x == -999)
			{
				found = true;
				break;
			}
			finder++;
		}

		if(!found)
		{
			diff_displacement = 0;
			reference_dist = 0;
			found = false;

			if(count>0) // code beow is skipped on the first cycle (count = 0)
			{
				eu_dist.clear();
				for (int i = 0; i < sensorsBallCenters.size(); i++)
				{
					// Calculation of the distance between the corrent center of the ball detected and the previous
					for (int j = 0; j < count ; j++)
					{
						dist = pointEuclideanDistance (sensorClouds[i].points[j], sensorsBallCenters[i]);
						if (dist < min_distance)
							break;
					}
					eu_dist.push_back( pointEuclideanDistance (sensorClouds[i].points[count-1], sensorsBallCenters[i]) );
					// Sums the squared difference between the reference sensor (first sensor on sensors_ball_centers) and all the other sensors
					diff_displacement = abs(eu_dist.front() - eu_dist.back());
					cout << endl << "points = " << i << " " << sensorsBallCenters[i]  << endl;
					cout << "diff_displacement = " << diff_displacement << endl;
          //if (diff_displacement > max_displacement || dist < min_distance)
          if (diff_displacement > max_displacement)
					{
						std::cout << "diff_displacement too high" << std::endl;
						found = true;
						break;
					}
					for (int i=0; i < eu_dist.size(); ++i)
					{
						std::cout << eu_dist[i] << ' ';
					}
					std::cout << std::endl;
				}
			}
			std::cout << "new points conditions: " << count << " " << found << " " << dist << std::endl;
			if( (count == 0) || (!found && dist >= min_distance) ) // the limit is set by the max_displacement variable in meters. If it's higher these points will be discarded
			{
				int cameraCounter = 0;
				for ( int i = 0; i < sensorsBallCenters.size(); i++ )
				{
					cout << "crash" << sensorsBallCenters[i] << endl;
					sensorClouds[i].push_back(sensorsBallCenters[i]); // sensorCLouds now contains ball center points for every sensor
					qDebug() << "nocrash";
					if (isCamera[i])
					{
						cameraCloudsPnP[cameraCounter].push_back(camCentroidPnP[cameraCounter]);
						string imgPath = file_path + "img_" + calibrationNodes[i] +"_" + boost::lexical_cast<std::string>(count) + ".jpg";
						cout << imgPath << "\n" << camImage.size() << endl;
						imwrite( imgPath, camImage[cameraCounter] );

						cameraCounter++;
						qDebug() << cameraCounter;
					}
				}


				// Saving all point clouds to a PCD file
				cameraCounter = 0;
				for ( int i = 0; i < sensorClouds.size(); i++ )
				{
					pcl::io::savePCDFileASCII(file_path + calibrationNodes[i] + ".pcd", sensorClouds[i]);
					if (isCamera[i])
					{
						pcl::io::savePCDFileASCII(file_path + calibrationNodes[i] + "_PnP.pcd", cameraCloudsPnP[cameraCounter]);
						cameraCounter++;
					}
				}

				qDebug() << "pcd save";

				// PointClouds and Poses visualization for Rviz (vector concatenation can probably be improved)
				visualizationPoses.clear();
				visualizationClouds.clear();
				visualizationPoses.reserve( sensorPoses.size() + cameraPosesPnP.size() );
				visualizationClouds.reserve( sensorClouds.size() + cameraCloudsPnP.size() );

				visualizationPoses.insert( visualizationPoses.end(), sensorPoses.begin(), sensorPoses.end() );
				visualizationPoses.insert( visualizationPoses.end(), cameraPosesPnP.begin(), cameraPosesPnP.end() );
				visualizationClouds.insert( visualizationClouds.end(), sensorClouds.begin(), sensorClouds.end() );
				visualizationClouds.insert( visualizationClouds.end(), cameraCloudsPnP.begin(), cameraCloudsPnP.end() );

				visualization_msgs::MarkerArray targets_markers;
				targets_markers.markers = createTargetMarkers(visualizationClouds, visualizationPoses, displayNames);
				markers_pub.publish(targets_markers);

				count++;
				if (count < num_of_points && !acquisitionIsAuto) // If acquisitionIsAuto is false then the user is prompted before acquiring points.
				{
					QMessageBox::StandardButton reply;

					QMutexLocker locker(&mutex);
					QString msg = "Point " + QString::number(count) + " of " + QString::number(num_of_points)
					              + " done.\nPress OK to acquire the next point.";
					emit showMsg(msg, &reply);
					waitCondition.wait(&mutex);
				}
			}
		}
		ros::spinOnce();
		loop_rate.sleep();
	}

	if (doCalibration)
	{

		int cameraCounter = 0;
		for (int i = 1; i < sensorsBallCenters.size(); i++) // starts with i=1 because for i=0 the target and uncalibrated sensor are the same
		{
			// Estimating rigid transform between target sensor and other sensors
			estimateTransformation(sensorPoses[i], sensorClouds.front(), sensorClouds[i],
			                       calibrationNodes.front(), calibrationNodes[i]);
			if (isCamera[i])
			{
				estimateTransformationCamera(cameraPosesPnP[cameraCounter], sensorClouds.front(), cameraCloudsPnP[cameraCounter],
				                             calibrationNodes.front(), calibrationNodes[i], camImage[cameraCounter], true, true);
				cameraCounter++;
			}
		}

		// Implement 3D models with translation and rotation sliders
		/*vector<double> RPY;
		   // ATLASCAR model rotations
		   RPY.push_back(M_PI/2); // X-rotation
		   RPY.push_back(0.0); // Y-rotation
		   RPY.push_back(M_PI); // Z-rotation

		   vector<double> translation;
		   translation.push_back(-4.387/2+ 0.05); // X translation. 4.387 is the car's length
		   translation.push_back(-1.702/2 + 0.05 ); // Y translation. 1.702 is the car's width
		   translation.push_back(-0.46); // Z translation. 0.46 is the height of the reference LMS sensor

		   visualization_msgs::Marker atlascar = addCar(RPY, translation); // builds the marker to publish
		   car_pub.publish( atlascar );

		   RPY.clear();
		   // Clouds and lasers rotations so point cloud is aligned with Rviz grid
		   RPY.push_back(0.0);
		   RPY.push_back(0.0);
		   RPY.push_back(55 * M_PI/180);*/

		// PointClouds and Poses visualization for Rviz (vector concatenation can probably be improved)
		visualizationPoses.clear();
		visualizationClouds.clear();
		visualizationPoses.reserve( sensorPoses.size() + cameraPosesPnP.size() );
		visualizationPoses.reserve( sensorClouds.size() + cameraCloudsPnP.size() );

		visualizationPoses.insert( visualizationPoses.end(), sensorPoses.begin(), sensorPoses.end() );
		visualizationPoses.insert( visualizationPoses.end(), cameraPosesPnP.begin(), cameraPosesPnP.end() );
		visualizationClouds.insert( visualizationClouds.end(), sensorClouds.begin(), sensorClouds.end() );
		visualizationClouds.insert( visualizationClouds.end(), cameraCloudsPnP.begin(), cameraCloudsPnP.end() );

		visualization_msgs::MarkerArray targets_markers;
		targets_markers.markers = createTargetMarkers(visualizationClouds, visualizationPoses, displayNames);
		markers_pub.publish(targets_markers);

		cout<<"Calibration complete!"<<endl;
	}

	emit calibrationComplete(); // used to signal the gui that the calibration is complete
}

/**
   @brief Sets relevant information about the sensors to be calibrated.
   @param[in] nodes vector that contains the names of the sensors launched
   @param[in] camera vector that contains information about the type of sensor. true if it's a camera, false otherwise
   @param[in] names sensor names to dispaly in the Rviz 3D visualization widget
   @return void
 */
void QNode::setNodes(const vector<string> nodes, const vector<bool> camera, const std::vector<std::string> names)
{
	qDebug() << "setLaunchedNodes";
	calibrationNodes = nodes;
	isCamera = camera;
    displayNames = names;
    std::cout << "Display Names: ";
    for (int i= 0; i < displayNames.size(); i++)
        std::cout << displayNames[i] << " ";
    std::cout << std::endl;
}

/**
   @brief Builds a message box to display information to the user
   @param[in] msg text shown in the message box
   @param[in] answer the answer given by the user to the message box
   @return void
 */
void QNode::msgShower(const QString& msg, QMessageBox::StandardButton* answer)
{
	QMutexLocker locker(&mutex);
	*answer = QMessageBox::information(0, "Next Point", msg);
	waitCondition.wakeOne();
}
