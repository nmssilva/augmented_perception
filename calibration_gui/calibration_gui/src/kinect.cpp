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
 *#include <ros/package.h>

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
   \file  kinect.cpp
   \brief Algorithm for the ball center detection with Kinect based on the algorithm developed for the Swissranger
   \author David Silva
   \date   May, 2016
 */

 #include "calibration_gui/kinect.h"
 #include "calibration_gui/visualization_rviz_kinect.h"

// TF
 #include <tf/transform_broadcaster.h>
 #include <geometry_msgs/TransformStamped.h>

ros::Publisher markers_pub;
ros::Publisher sphereCenter_pub;


/**
   @brief write in a file the center of the sphere
   @param[in] sphereCoeffsRefined center sphere coordinates
   @return void
 */
void writeFile(Eigen::VectorXf sphereCoeffsRefined)
{
	FILE* pFile = fopen("sr_3m.txt", "a");
	fprintf(pFile, "%F %F %F\n ", sphereCoeffsRefined(0),sphereCoeffsRefined(1),sphereCoeffsRefined(2));
	fclose(pFile);
}

/**
   @brief Detection of the ball on the Kinect data
   @param[in] Kinect_cloud point cloud from the Kinect
   @return void
 */
void sphereDetection(pcl::PointCloud<pcl::PointXYZ> Kinect_cloud)
{

	ros::Time start = ros::Time::now();

	geometry_msgs::PointStamped sphereCenter;
	sphereCenter.point.x = -999;
	sphereCenter.point.y = -999;
	sphereCenter.point.z = -999;

  // Ball detection methods tried =============================================

	/* METHOD #1 ================================================================
	 * Detects the ball up to 2 meters, very fast
	 * Original method
	 *
	   pcl::PointCloud<pcl::PointXYZ>::Ptr Kinect_cloudPtr (new pcl::PointCloud<pcl::PointXYZ>(Kinect_cloud));

	   pcl::PointCloud<pcl::PointXYZ>::Ptr Kinect_cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

	   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZ>);

	   //std::cerr << "PointCloud before filtering: " << Kinect_cloudPtr->width * Kinect_cloudPtr->height << " data points." << std::endl;

	   pcl::VoxelGrid<pcl::PointXYZ> sor;
	   sor.setInputCloud (Kinect_cloudPtr);
	   sor.setFilterFieldName ("z");
	   sor.setFilterLimits (0, 5);
	   sor.setLeafSize (0.005f, 0.005f, 0.005f);
	   sor.filter (*Kinect_cloud_filtered);

	   //std::cerr << "PointCloud after filtering: " << Kinect_cloud_filtered->width * Kinect_cloud_filtered->height << " data points." << std::endl;

	   pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
	   pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
	   // Create the segmentation object
	   pcl::SACSegmentation<pcl::PointXYZ> seg;
	   // Optional
	   seg.setOptimizeCoefficients (true);
	   // Mandatory
	   seg.setModelType (pcl::SACMODEL_SPHERE);
	   seg.setMethodType (pcl::SAC_RANSAC);
	   //seg.setMaxIterations (1000);
	   seg.setDistanceThreshold (0.05);
	   seg.setRadiusLimits (0, BALL_DIAMETER);

	   //std::cout << "min: " << BALL_DIAMETER/2 - 0.05*BALL_DIAMETER/2 << "max: " << BALL_DIAMETER/2 + 0.05*BALL_DIAMETER/2 << std::endl;
	   seg.setInputCloud (Kinect_cloud_filtered);
	   seg.segment (*inliers, *coefficients);

	   ros::Time end = ros::Time::now();

	   cout << "Ball detection time filtered: " <<  (end - start).toNSec() * 1e-6 << " msec"  << endl;


	   if(inliers->indices.size ()>0)
	   {

	        if (coefficients->values[3]<BALL_DIAMETER/2 + 0.05*BALL_DIAMETER/2 && coefficients->values[3]>BALL_DIAMETER/2 - 0.05*BALL_DIAMETER/2)
	        {
	                sphereCenter.point.x = coefficients->values[0];
	                sphereCenter.point.y = coefficients->values[1];
	                sphereCenter.point.z = coefficients->values[2];


	                cout << "Accepted: " << *coefficients << endl;
	        }

	   }*/

	/* METHOD #2 ================================================================
	 * Detects the ball up to 2 meters, very slow
	 * Uses normal information to detect spheres
	 *
	   pcl::PointCloud<pcl::PointXYZ>::Ptr Kinect_cloudPtr (new pcl::PointCloud<pcl::PointXYZ>(Kinect_cloud));

	   pcl::PointCloud<pcl::PointXYZ>::Ptr Kinect_cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

	   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZ>);

	   //std::cerr << "PointCloud before filtering: " << Kinect_cloudPtr->width * Kinect_cloudPtr->height << " data points." << std::endl;

	   pcl::VoxelGrid<pcl::PointXYZ> sor;
	   sor.setInputCloud (Kinect_cloudPtr);
	   sor.setFilterFieldName ("z");
	   sor.setFilterLimits (0, 5);
	   sor.setLeafSize (0.005f, 0.005f, 0.005f);
	   sor.filter (*Kinect_cloud_filtered);

	   //std::cerr << "PointCloud after filtering: " << Kinect_cloud_filtered->width * Kinect_cloud_filtered->height << " data points." << std::endl;
	   // All the objects needed
	   pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	   pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
	   pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
	   pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
	   pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());

	   // Datasets
	   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
	   pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

	   // Estimate point normals
	   ne.setSearchMethod (tree);
	   ne.setInputCloud (Kinect_cloud_filtered);
	   ne.setKSearch (50);
	   ne.compute (*cloud_normals);

	   // Create the segmentation object for the planar model and set all the parameters
	   seg.setOptimizeCoefficients (true);
	   seg.setModelType (pcl::SACMODEL_NORMAL_SPHERE);
	   seg.setNormalDistanceWeight (0.054191);
	   seg.setMethodType (pcl::SAC_RANSAC);
	   seg.setMaxIterations (10000);
	   seg.setDistanceThreshold (0.0070936);
	   seg.setRadiusLimits (BALL_DIAMETER/2-0.05, BALL_DIAMETER/2+0.05);
	   seg.setInputCloud (Kinect_cloud_filtered);
	   seg.setInputNormals (cloud_normals);
	   // Obtain the plane inliers and coefficients
	   seg.segment (*inliers, *coefficients);

	   ros::Time end = ros::Time::now();

	   cout << "Ball detection time filtered: " <<  (end - start).toNSec() * 1e-6 << " msec"  << endl;


	   if(inliers->indices.size ()>0)
	   {

	        if (coefficients->values[3]<BALL_DIAMETER/2 + 0.05*BALL_DIAMETER/2 && coefficients->values[3]>BALL_DIAMETER/2 - 0.05*BALL_DIAMETER/2)
	        {
	                sphereCenter.point.x = coefficients->values[0];
	                sphereCenter.point.y = coefficients->values[1];
	                sphereCenter.point.z = coefficients->values[2];


	                cout << "Accepted: " << *coefficients << endl;
	        }

	   }*/

	/* METHOD #3 ================================================================
	 * Detects the ball up to 3 meters, fast. Optimized Method #1
	 */
	pcl::PointCloud<pcl::PointXYZ>::Ptr Kinect_cloudPtr (new pcl::PointCloud<pcl::PointXYZ>(Kinect_cloud));

	pcl::PointCloud<pcl::PointXYZ>::Ptr Kinect_cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZ>);

	//std::cerr << "PointCloud before filtering: " << Kinect_cloudPtr->width * Kinect_cloudPtr->height << " data points." << std::endl;

	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud (Kinect_cloudPtr);
	sor.setFilterFieldName ("z");
	sor.setFilterLimits (0, 5);
	sor.setLeafSize (0.005f, 0.005f, 0.005f);
	sor.filter (*Kinect_cloud_filtered);

	//std::cerr << "PointCloud after filtering: " << Kinect_cloud_filtered->width * Kinect_cloud_filtered->height << " data points." << std::endl;

	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());

	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// Optional
	seg.setOptimizeCoefficients (true);
	// Mandatory
	seg.setModelType (pcl::SACMODEL_SPHERE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setMaxIterations (10000);
	seg.setDistanceThreshold (0.0070936);
	seg.setProbability(0.99);
	seg.setRadiusLimits (BALL_DIAMETER/2-0.05, BALL_DIAMETER/2+0.05);
	seg.setInputCloud (Kinect_cloud_filtered);
	seg.segment (*inliers, *coefficients);

	ros::Time end = ros::Time::now();

	cout << "Ball detection time filtered: " <<  (end - start).toNSec() * 1e-6 << " msec"  << endl;

	cout << *coefficients << endl;

	if(inliers->indices.size ()>0)
	{

		if (coefficients->values[3]<BALL_DIAMETER/2 + 0.05*BALL_DIAMETER/2 && coefficients->values[3]>BALL_DIAMETER/2 - 0.05*BALL_DIAMETER/2)
		{
			sphereCenter.point.x = coefficients->values[0];
			sphereCenter.point.y = coefficients->values[1];
			sphereCenter.point.z = coefficients->values[2];


			cout << "Accepted: " << *coefficients << endl;
		}
	}
  // Ball detection ends here ==================================================

	sphereCenter.header.stamp = ros::Time::now();
	sphereCenter_pub.publish(sphereCenter);

	pcl::PointXYZ center;
	center.x = sphereCenter.point.x;
	center.y = sphereCenter.point.y;
	center.z = sphereCenter.point.z;

	visualization_msgs::MarkerArray targets_markers;
	targets_markers.markers = createTargetMarkers(center);
	markers_pub.publish(targets_markers);

}

/**
   @brief Main function of the ball detection node for kinect
   @param argc
   @param argv
   @return int
 */
int main(int argc, char **argv)
{
	ros::init(argc, argv, "kinect");

	ros::NodeHandle n("~");
	string node_ns = ros::this_node::getNamespace();
	node_ns.erase(0, 2);
	n.getParam("ballDiameter", BALL_DIAMETER);
	cout << "Node namespace:" << node_ns << endl;
	cout << "Ball diameter:" << BALL_DIAMETER << endl;

	markers_pub = n.advertise<visualization_msgs::MarkerArray>( "BallDetection", 1000);
	sphereCenter_pub = n.advertise<geometry_msgs::PointStamped>("SphereCentroid",1000);

	kinect cloud(node_ns);

	tf::Transform transform;
	tf::TransformBroadcaster tf_broadcast;
	transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
	transform.setRotation( tf::createQuaternionFromRPY(0,0,0) ); // ZYX

	ros::Rate loop_rate(30);

	while( ros::ok() )
	{
		tf_broadcast.sendTransform(tf::StampedTransform(transform, ros::Time::now(),"my_frame","camera_rgb_optical_frame"));

		if(cloud.cloud.points.size())
		{
			sphereDetection(cloud.cloud);
		}
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
