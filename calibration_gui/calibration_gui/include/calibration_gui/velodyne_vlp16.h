/**************************************************************************************************
 Software License Agreement (BSD License)

 Copyright (c) 2014-2015, LAR toolkit developers - University of Aveiro - http://lars.mec.ua.pt
 All rights reserved.

 Redistribution and use in source and binary forms, with or without modification, are permitted
 provided that the following conditions are met:

  *Redistributions of source code must retain the above copyright notice, this list of
   conditions and the following disclaimer.
  *Redistributions in binary form must reproduce the above copyright notice, this list of
   conditions and the following disclaimer in the documentation and/or other materials provided
   with the distribution.
  *Neither the name of the University of Aveiro nor the names of its contributors may be used to
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
\file  velodyne_vlp16.h
\brief Global include file
\author Diogo Correia
\date   June, 2017
*/

#ifndef _VELODYNE_VLP16_H_
#define _VELODYNE_VLP16_H_

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <rosbag/bag.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_broadcaster.h>
#include <sstream>
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Point.h"
#include <vector>
#include <iostream>
#include <stdlib.h>
#include <colormap/colormap.h>
#include <utility>
#include <map>
#include <fstream>
#include <string>
#include <ros/package.h>
#include <sensor_msgs/LaserScan.h>
#include "lidar_segmentation/lidar_segmentation.h"

using namespace std;

//Shared pointer to the Point class
typedef boost::shared_ptr<Point> PointPtr;
typedef boost::shared_ptr< ::sensor_msgs::LaserScan> LaserScanPtr;

void getClusters(vector<PointPtr> laserPoints, vector<ClusterPtr> * clusters_nn);
void velodyne_findBall(vector< vector<PointPtr> > laserscans);
double find_circle(vector<ClusterPtr> clusters, vector<ClusterPtr>& circleP, int layer);
void calculateSphereCentroid(vector<geometry_msgs::Point> center, geometry_msgs::PointStamped &sphereCentroid, vector<double> radius);
void rotatePoints(double& x,double& y, double& z, double angle);
void sphereDetection(pcl::PointCloud<pcl::PointXYZ> Kinect_cloud);
void getMax(vector<double> vec_in, double max, int max_indx);
vector<geometry_msgs::Point> removeOut(vector<geometry_msgs::Point> center, vector<double> radius, vector<double> radius_clean);
double pointsDist(geometry_msgs::Point center1, geometry_msgs::Point center3);
#endif
