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
 \file  visualization_rviz_velodyne.cpp
 \brief Illustration of the ball detection on the laser data
 \details It is represented the segmentation of the laser data, and the ball when detected
 \author Diogo Correia
 \date   June, 2017
*/

#define _VLP_VISUALIZATION_RVIZ_CPP_

#include <lidar_segmentation/lidar_segmentation.h>
#include <lidar_segmentation/clustering.h>
#include "calibration_gui/common_functions.h"
#include "calibration_gui/visualization_rviz_velodyne.h"
#include <visualization_msgs/Marker.h>
#include <algorithm>
#include <iterator>
#include <iostream>
#include <vector>


/**
@brief Markers publication for the visualization of the laser scans, circle detected and ball detected
@param[in] sickLidarClusters_nn segmentatio of the several laser scans
@param[in] circlePoints points for the representation of the detected circles
@param[in] sphere center coordinates of the ball
@param[in] radius radius of the circles detected
@return vector<visualization_msgs::Marker>
*/
vector<visualization_msgs::Marker> createTargetMarkers(vector<LidarClustersPtr>& sickLidarClusters_nn, vector<LidarClustersPtr>& circlePoints, Point sphere, vector<double> radius )
{
    static Markers marker_list;

    //Reduce the elements status, ADD to REMOVE and REMOVE to delete
    marker_list.decrement();

    // Create a colormap
    class_colormap colormap("hsv",10, 1, false);

    vector<visualization_msgs::Marker> marker_ids_scan;
    vector<visualization_msgs::Marker> marker_ids_circle;

    visualization_msgs::Marker marker_ids_sphere;
    marker_ids_sphere.header.frame_id = "/velodyne";
    marker_ids_sphere.header.stamp = ros::Time::now();
    marker_ids_sphere.ns = "ids_sphere";
    marker_ids_sphere.action = visualization_msgs::Marker::ADD;
    marker_ids_sphere.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker_ids_sphere.scale.x = 0.1;
    marker_ids_sphere.scale.y = 0.1;
    marker_ids_sphere.scale.z = 0.1;
    marker_ids_sphere.color.a = 1.0;
    marker_ids_sphere.color.r = 1.0;
    marker_ids_sphere.color.g = 1.0;
    marker_ids_sphere.color.b = 1.0;

    vector<visualization_msgs::Marker> marker_clusters_scan;
    vector<visualization_msgs::Marker> marker_clusters_circle;

    visualization_msgs::Marker marker_sphere;
    marker_sphere.header.frame_id = "/velodyne";
    marker_sphere.header.stamp = marker_sphere.header.stamp;
    marker_sphere.ns = "sphere";
    marker_sphere.action = visualization_msgs::Marker::ADD;
    marker_sphere.type = visualization_msgs::Marker::SPHERE_LIST;
    marker_sphere.scale.x = 0.8;
    marker_sphere.scale.y = 0.8;
    marker_sphere.scale.z = 0.8;

    for(int i = 0; i<sickLidarClusters_nn.size(); i++){
      visualization_msgs::Marker marker_id_scan;
      marker_ids_scan.push_back(marker_id_scan);

      visualization_msgs::Marker marker_id_circle;
      marker_ids_circle.push_back(marker_id_circle);

      visualization_msgs::Marker marker_cluster_scan;
      marker_clusters_scan.push_back(marker_cluster_scan);

      visualization_msgs::Marker marker_cluster_circle;
      marker_clusters_circle.push_back(marker_cluster_circle);

      marker_ids_scan[i].header.frame_id = "/velodyne";
      marker_ids_scan[i].header.stamp = ros::Time::now();

      marker_ids_circle[i].header.frame_id = "/velodyne";
      marker_ids_circle[i].header.stamp = ros::Time::now();

      marker_clusters_scan[i].header.frame_id = "/velodyne";
      marker_clusters_scan[i].header.stamp = marker_clusters_scan[i].header.stamp;

      marker_clusters_circle[i].header.frame_id = "/velodyne";
      marker_clusters_circle[i].header.stamp = marker_clusters_circle[i].header.stamp;

      marker_ids_scan[i].ns = "ids_scan"+i;
      marker_ids_scan[i].action = visualization_msgs::Marker::ADD;

      marker_ids_circle[i].ns = "ids_circle"+i;
      marker_ids_circle[i].action = visualization_msgs::Marker::ADD;

      marker_clusters_scan[i].ns = "clusters_scan"+i;
      marker_clusters_scan[i].action = visualization_msgs::Marker::ADD;

      marker_clusters_circle[i].ns = "clusters_circle"+i;
      marker_clusters_circle[i].action = visualization_msgs::Marker::ADD;

      marker_ids_scan[i].type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      marker_ids_circle[i].type = visualization_msgs::Marker::TEXT_VIEW_FACING;

      marker_clusters_scan[i].type = visualization_msgs::Marker::SPHERE_LIST;
      marker_clusters_circle[i].type = visualization_msgs::Marker::LINE_STRIP;

      marker_ids_scan[i].scale.x = 0.1;
      marker_ids_scan[i].scale.y = 0.1;
      marker_ids_scan[i].scale.z = 0.1;
      marker_ids_circle[i].scale.x = 0.1;
      marker_ids_circle[i].scale.y = 0.1;
      marker_ids_circle[i].scale.z = 0.1;
      marker_clusters_scan[i].scale.x = 0.08;
      marker_clusters_scan[i].scale.y = 0.08;
      marker_clusters_scan[i].scale.z = 0.08;
      marker_clusters_circle[i].scale.x = 0.03;

      marker_ids_scan[i].color.a = 1.0;
      marker_ids_scan[i].color.r = 1.0;
      marker_ids_scan[i].color.g = 1.0;
      marker_ids_scan[i].color.b = 1.0;
      marker_ids_circle[i].color.a = 1.0;
      marker_ids_circle[i].color.r = 1.0;
      marker_ids_circle[i].color.g = 1.0;
      marker_ids_circle[i].color.b = 1.0;
    }

    vector<double>::iterator max = max_element(radius.begin(),radius.end());
    int id1 = distance(radius.begin(),max);
    radius[id1]=0;
    max = max_element(radius.begin(),radius.end());
    int id2 = distance(radius.begin(),max);
    radius[id2]=0;
    max = max_element(radius.begin(),radius.end());
    int id3 = distance(radius.begin(),max);
    radius[id3]=0;
    max = max_element(radius.begin(),radius.end());
    int id4 = distance(radius.begin(),max);

//    //green
//    if(id1==0)
//    {
//        marker_clusters_circle0.color.g=1.0f;
//        marker_clusters_circle0.color.a=1.0;
//    }
//    else if(id1==1)
//    {
//        marker_clusters_circle1.color.g=1.0f;
//        marker_clusters_circle1.color.a=1.0;
//    }
//    else if(id1==2)
//    {
//        marker_clusters_circle2.color.g=1.0f;
//        marker_clusters_circle2.color.a=1.0;
//    }
//    else if(id1==3)
//    {
//        marker_clusters_circle3.color.g=1.0f;
//        marker_clusters_circle3.color.a=1.0;
//    }

//    //blue
//    if(id2==0)
//    {
//        marker_clusters_circle0.color.b=1.0;
//        marker_clusters_circle0.color.a=1.0;
//    }
//    else if(id2==1)
//    {
//        marker_clusters_circle1.color.b=1.0;
//        marker_clusters_circle1.color.a=1.0;
//    }
//    else if(id2==2)
//    {
//        marker_clusters_circle2.color.b=1.0;
//        marker_clusters_circle2.color.a=1.0;
//    }
//    else if(id2==3)
//    {
//        marker_clusters_circle3.color.b=1.0;
//        marker_clusters_circle3.color.a=1.0;
//    }

//    //yellow
//    if(id3==0)
//    {
//        marker_clusters_circle0.color.r=1.0;
//        marker_clusters_circle0.color.g=1.0;
//        marker_clusters_circle0.color.a=1.0;
//    }
//    else if(id3==1)
//    {
//        marker_clusters_circle1.color.r=1.0;
//        marker_clusters_circle1.color.g=1.0;
//        marker_clusters_circle1.color.a=1.0;
//    }
//    else if(id3==2)
//    {
//        marker_clusters_circle2.color.r=1.0;
//        marker_clusters_circle2.color.g=1.0;
//        marker_clusters_circle2.color.a=1.0;
//    }
//    else if(id3==3)
//    {
//        marker_clusters_circle3.color.r=1.0;
//        marker_clusters_circle3.color.g=1.0;
//        marker_clusters_circle3.color.a=1.0;
//    }

//    //red
//    if(id4==0)
//    {
//        marker_clusters_circle0.color.r=1.0;
//        marker_clusters_circle0.color.a=1.0;
//    }
//    else if(id4==1)
//    {
//        marker_clusters_circle1.color.r=1.0;
//        marker_clusters_circle1.color.a=1.0;
//    }
//    else if(id4==2)
//    {
//        marker_clusters_circle2.color.r=1.0;
//        marker_clusters_circle2.color.a=1.0;
//    }
//    else if(id4==3)
//    {
//        marker_clusters_circle3.color.r=1.0;
//        marker_clusters_circle3.color.a=1.0;
//    }

    for(int j = 0; j<sickLidarClusters_nn.size(); j++){
      //Cluster from scan
      vector<ClusterPtr> clusters_scan0 = sickLidarClusters_nn[j]->Clusters;

      for ( uint i = 0; i<  clusters_scan0.size() ; i++)
      {
        ClusterPtr cluster_nn = clusters_scan0[i];

        std_msgs::ColorRGBA color = colormap.color(i);

        //Place in the marker every point belonging to the cluster "i"
        for(uint h=0;h<cluster_nn->support_points.size();h++)
        {
          geometry_msgs::Point pt;
          pt.x=cluster_nn->support_points[h]->x;
          pt.y=cluster_nn->support_points[h]->y;
          pt.z=cluster_nn->support_points[h]->z;
          //cout<<"z "<<pt.z<<endl;

          marker_clusters_scan[j].points.push_back(pt);
          marker_clusters_scan[j].colors.push_back(color);
        }

        marker_ids_scan[j].pose.position.x = cluster_nn->centroid->x;
        marker_ids_scan[j].pose.position.y = cluster_nn->centroid->y;
        marker_ids_scan[j].pose.position.z = 5;

        //text
        boost::format fm("%d");
        fm % cluster_nn->id;

        marker_ids_scan[j].text = fm.str();
        marker_ids_scan[j].id = cluster_nn->id;

        marker_list.update(marker_ids_scan[j]);
        marker_list.update(marker_clusters_scan[j]);

      } //end for
    }


    for(int j = 0; j<sickLidarClusters_nn.size(); j++){
      //circle fit scan0

      vector<ClusterPtr> circles_scan0 = circlePoints[j]->Clusters;
      for ( uint i = 0; i< circles_scan0.size() ; i++)
      {
        ClusterPtr cluster_circle = circles_scan0[i];

        std_msgs::ColorRGBA color = colormap.color(i);

        //Place in the marker every point belonging to the cluster "i"
        for(uint h=0;h<cluster_circle->support_points.size();h++)
        {
          geometry_msgs::Point pt;
          pt.x=cluster_circle->support_points[h]->x;
          pt.y=cluster_circle->support_points[h]->y;
          pt.z= -0.2;

          marker_clusters_circle[j].points.push_back(pt);
          //marker_clusters_circle[j].colors.push_back(color);
        }

        marker_ids_circle[j].pose.position.x = cluster_circle->centroid->x;
        marker_ids_circle[j].pose.position.y = cluster_circle->centroid->y;
        marker_ids_circle[j].pose.position.z = 1.5;

        //text
        boost::format fm("%d");
        fm % 1.0;

        marker_ids_circle[j].text = fm.str();
        marker_ids_circle[j].id = 1.0;

        marker_list.update(marker_ids_circle[j]);
        marker_list.update(marker_clusters_circle[j]);

      } //end for
    }

    //sphere

    if(sphere.x!=0)
    {
        std_msgs::ColorRGBA color = colormap.color(0);

        geometry_msgs::Point pt;
        pt.x=sphere.x;
        pt.y= sphere.y;
        pt.z= sphere.z;
        marker_sphere.points.push_back(pt);
        marker_sphere.colors.push_back(color);

        marker_ids_sphere.pose.position.x = 0;
        marker_ids_sphere.pose.position.y = 0;
        marker_ids_sphere.pose.position.z = 0;

        //text
        boost::format fm("%d");
        fm % 1.0;

        marker_ids_sphere.text = fm.str();
        marker_ids_sphere.id = 1.0;

        marker_list.update(marker_ids_sphere);
        marker_list.update(marker_sphere);
    }

    //Remove markers that should not be transmitted
    marker_list.clean();

    //Clean the marker_vector and put new markers in it;
    return marker_list.getOutgoingMarkers();

} //end function

/**
@brief Markers publication for the visualization of the ball detected
@param[in] sphereCenter center coordinates of the ball
@return vector<visualization_msgs::Marker>
*/
vector<visualization_msgs::Marker> createTargetMarkers(pcl::PointXYZ sphereCenter )
{

    static Markers marker_list;

    //Reduce the elements status, ADD to REMOVE and REMOVE to delete
    marker_list.decrement();

    visualization_msgs::Marker marker_sphere;

    marker_sphere.header.frame_id = "/velodyne";
    marker_sphere.header.stamp = ros::Time::now();

    marker_sphere.ns = "sphere";
    marker_sphere.action = visualization_msgs::Marker::ADD;

    marker_sphere.type = visualization_msgs::Marker::SPHERE_LIST;

    marker_sphere.scale.x = 0.9;
    marker_sphere.scale.y = 0.9;
    marker_sphere.scale.z = 0.9;


    //sphere

    if(sphereCenter.x!=0)
    {
        geometry_msgs::Point pt;
        pt.x=sphereCenter.x;
        pt.y=sphereCenter.y;
        pt.z=sphereCenter.z;
        marker_sphere.points.push_back(pt);
        marker_sphere.color.a=1;
        marker_sphere.color.g=1;

        marker_list.update(marker_sphere);
    }

    //Remove markers that should not be transmitted
    marker_list.clean();

    //Clean the marker_vector and put new markers in it;
    return marker_list.getOutgoingMarkers();

} //end function
