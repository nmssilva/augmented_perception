/*---PointCould Includes---*/
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_sphere.h>

/*---Velodyne Package Includes---*/
#include "velodyne_pointcloud/rawdata.h"

/*---LAR TK4 Includes---*/
#include "lidar_segmentation/clustering.h"
#include "lidar_segmentation/lidar_segmentation.h"
//#include <colormap/colormap.h>

/*---Calobration Package Includes---*/
#include "calibration_gui/velodyne_vlp16.h"
#include "calibration_gui/common_functions.h"
#include "calibration_gui/visualization_rviz_velodyne.h"


using namespace ros;
using namespace velodyne_rawdata;

geometry_msgs::PointStamped sphereCentroid;
Publisher sphereCentroid_pub;
Publisher velodyne_pub;


void getClusters(vector<PointPtr> laserPoints, vector<ClusterPtr> * clusters_nn){

  double threshold_nn = 0.1;
  nnClustering( laserPoints, threshold_nn , *clusters_nn);
}

/**
   @brief Handler for the incoming data
   @param[in] lidarPoints incoming Laser Points
   @param[in] iterations iteration of the Laser Scan
   @return void
 */
void velodyne_findBall(vector< vector<PointPtr> > laserscans)
{
  vector<LidarClustersPtr> clusters;
  vector<LidarClustersPtr> circlePoints;
  vector<double> radius;
  vector<geometry_msgs::Point> center;
  Point sphere;

  for(int i = 0; i<laserscans.size() ;i++){
    vector<ClusterPtr> clusters_nn;

    getClusters(laserscans[i], &clusters_nn);

    LidarClustersPtr cluster (new LidarClusters);
    cluster->Clusters = clusters_nn;
    clusters.push_back(cluster);
//    ROS_INFO("%d",clusters_nn.size());

    LidarClustersPtr circlePs (new LidarClusters);
    vector<ClusterPtr> circleP;
    double r;
    r=find_circle(clusters_nn,circleP,i);
    int num;
    if(r!=0)
      num++;

    radius.push_back(r);
    circlePs->Clusters = circleP;
    circlePoints.push_back(circlePs);


    /*--------Publish Circle Centroid--------*/
    center.push_back(sphereCentroid.point);

    int circlesNumb = 0;
    if(i==laserscans.size()-1)
    {
      for(int j=0; j<laserscans.size(); j++)
      {
        if(radius[j]>0.001)
          circlesNumb++;
      }

      if(circlesNumb>1)
      {
        calculateSphereCentroid(center, sphereCentroid, radius);
        sphere.x=sphereCentroid.point.x;
        sphere.y=sphereCentroid.point.y;
        sphere.z=sphereCentroid.point.z;
      }
      else
      {
        sphere.x=-100;
        sphere.y=0;
        sphere.z=0;
      }
      sphereCentroid.header.stamp = ros::Time::now();
      sphereCentroid_pub.publish(sphereCentroid);
    }    
  }

  /*---------Vizualize the Segmentation Results---------*/
  visualization_msgs::MarkerArray targets_markers;
  targets_markers.markers = createTargetMarkers(clusters,circlePoints, sphere,radius);

  velodyne_pub.publish(targets_markers);

} //end function

void getMax(vector<double> vec_in, double * max, int * max_indx){
  double maxim = 0;
  int max_idx;

  for(int i = 0; i<vec_in.size();i++){
    if(vec_in[i]>maxim){
      maxim = vec_in[i];
      max_idx = i;
    }
  }

  *max = maxim;
  *max_indx = max_idx;
}

vector<geometry_msgs::Point> removeOut(vector<geometry_msgs::Point> center, vector<double> radius, vector<double> *radius_clean, vector<int> *idx){
  vector<geometry_msgs::Point> center_clean;
//  vector<double> radius_clean;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>);

  cloud->width  = center.size();
  cloud->height = 1;
  cloud->points.resize (cloud->width * cloud->height);

  for(int i = 0; i<center.size();i++){
    cloud->points[i].x = center[i].x;
    cloud->points[i].y = center[i].y;
    cloud->points[i].z = center[i].z;
    cloud->points[i].intensity = radius[i];
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::copyPointCloud(*cloud, *cloud2);

  for(int i = 0; i<center.size();i++){
    cloud2->points[i].z = 0;
    cloud2->points[i].intensity = 0;
  }

  vector< int > indices;
  vector< int > indices2;

  pcl::RadiusOutlierRemoval<pcl::PointXYZI> outrem;
  // build the filter
  outrem.setInputCloud(cloud);
  outrem.setRadiusSearch(0.5);
  outrem.setMinNeighborsInRadius (2);
  // apply filter
  outrem.filter (*cloud_filtered);
  outrem.filter(indices);

  double ballDiameter = BALL_DIAMETER;
//  for(int i = 0; i<cloud_filtered->points.size();i++){
//    if(cloud_filtered->points[i].intensity>0 && cloud_filtered->points[i].intensity <= ballDiameter/2+0.1){
//      geometry_msgs::Point point;
//      point.x = cloud_filtered->points[i].x;
//      point.y = cloud_filtered->points[i].y;
//      point.z = cloud_filtered->points[i].z;
//      center_clean.push_back(point);
//      radius_clean->push_back(cloud_filtered->points[i].intensity);
//    }
//  }

  for(int i = 0; i<indices.size();i++){
    cout << "ind: " << cloud->points[indices[i]].intensity << ", ";
  }
    cout << endl;

  for(int i = 0; i<indices.size();i++){
    if(cloud->points[indices[i]].intensity>0 && cloud->points[indices[i]].intensity <= ballDiameter/2+0.1){
      geometry_msgs::Point point;
      point.x = cloud->points[indices[i]].x;
      point.y = cloud->points[indices[i]].y;
      point.z = cloud->points[indices[i]].z;
      center_clean.push_back(point);
      radius_clean->push_back(cloud->points[indices[i]].intensity);
      indices2.push_back(indices[i]);
    }
  }

    if(radius_clean->size()==0){
      for(int i = 0; i<cloud->points.size();i++){
        if(cloud->points[i].intensity>0 && cloud->points[i].intensity <= ballDiameter/2+0.1){
          geometry_msgs::Point point;
          point.x = cloud->points[i].x;
          point.y = cloud->points[i].y;
          point.z = cloud->points[i].z;
          center_clean.push_back(point);
          radius_clean->push_back(cloud->points[i].intensity);
          indices2.push_back(i);
        }
    }
  }
  *idx = indices2;
//  cout << "Before: " << radius.size() << endl;
//  cout << "After: " << radius_clean->size() <<endl;

  return center_clean;
}

/**
   @brief Calculation of sphere centroid
   @param[in] center coordinates of circle centers from the four layers
   @param[out] sphereCentroid coordinates of the sphere centroid
   @param[in] radius radius of the several circles
   @return void
 */
void calculateSphereCentroid(vector<geometry_msgs::Point> center, geometry_msgs::PointStamped & sphereCentroid, vector<double> radius)
{
  double d;
  sphereCentroid.point.x=0;
  sphereCentroid.point.y=0;
  sphereCentroid.point.z=0;

  cout << "calculateSphereCentroid Radius = " << endl;

  vector<double> radius_clean;
  vector<int> indices;
  vector<geometry_msgs::Point> center_clean = removeOut(center,radius, &radius_clean, &indices);

  int max_radius_index = 0;
  double max_radius = 0;
  getMax(radius_clean,&max_radius,&max_radius_index);
//  cout << "Index: "<< max_radius_index << endl;
  double ballDiameter = BALL_DIAMETER;

  geometry_msgs::Point center1;
  geometry_msgs::Point center2;
  geometry_msgs::Point center3;
  geometry_msgs::Point zOpt;
  for(int i=0; i<radius_clean.size(); i++)
  {
    double angle;
    if(i<8){
      angle = (((double)(indices[i])*30/16)-15);
    }else{
      angle = (((double)(indices[i]+1)*30/16)-15);
    }

    bool lower = false;
    if(center_clean[i].z < center_clean[max_radius_index].z){
      lower = true;
    }

    rotatePoints(center_clean[i].x,center_clean[i].y,center_clean[i].z,angle);
    d=pow(ballDiameter/2,2)-pow(radius_clean[i],2);
    if(d<0){
      d=0;
    }
    if(lower == true){
      center_clean[i].z=sqrt(d);
    }else{
      center_clean[i].z=-sqrt(d);
    }

    if(center_clean[i].z == center_clean[max_radius_index].z){
      zOpt.x = center_clean[i].x;
      zOpt.y = center_clean[i].y;
      zOpt.z = -center_clean[i].z;

      rotatePoints(zOpt.x,zOpt.y,zOpt.z,-angle);
    }

    rotatePoints(center_clean[i].x,center_clean[i].y,center_clean[i].z,-angle);

    if(lower){
      cout << "R: " << radius_clean[i] << " || Z: " << center_clean[i].z << " Lower" << endl;
    }else{
      cout << "R: " << radius_clean[i] << " || Z: " << center_clean[i].z << " Upper" << endl;
    }

//    sphereCentroid.point.x+=(center_clean[i].x);
//    sphereCentroid.point.y+=(center_clean[i].y);
//    sphereCentroid.point.z+=(center_clean[i].z);
//    count++;
  }


  for(int i = 0; i < center_clean.size();i++){
    if(i != max_radius_index ){
    center1.x +=center_clean[i].x;
    center1.y +=center_clean[i].y;
    center1.z +=center_clean[i].z;

    center2.x +=center_clean[i].x;
    center2.y +=center_clean[i].y;
    center2.z +=center_clean[i].z;

    center3.x +=center_clean[i].x;
    center3.y +=center_clean[i].y;
    center3.z +=center_clean[i].z;
    }else{
      center2.x +=zOpt.x;
      center2.y +=zOpt.y;
      center2.z +=-zOpt.z;

      center3.x +=center_clean[i].x;
      center3.y +=center_clean[i].y;
      center3.z +=center_clean[i].z;
    }
  }
  center1.x =center1.x/center_clean.size()-1;
  center1.y =center1.y/center_clean.size()-1;
  center1.z =center1.z/center_clean.size()-1;

  center2.x =center2.x/center_clean.size();
  center2.y =center2.y/center_clean.size();
  center2.z =center2.z/center_clean.size();

  center3.x =center3.x/center_clean.size();
  center3.y =center3.y/center_clean.size();
  center3.z =center3.z/center_clean.size();

  double d1 = pointsDist(center1, center2);
  double d2 = pointsDist(center1, center3);

  if(d1<d2){
    sphereCentroid.point.x=center2.x;
    sphereCentroid.point.y=center2.y;
    sphereCentroid.point.z=center2.z;
  }else{
    sphereCentroid.point.x=center3.x;
    sphereCentroid.point.y=center3.y;
    sphereCentroid.point.z=center3.z;
  }


//  sphereCentroid.point.x=sphereCentroid.point.x/count;
//  sphereCentroid.point.y=sphereCentroid.point.y/count;
//  sphereCentroid.point.z=sphereCentroid.point.z/count;
  double centre[3];
  centre[0]=sphereCentroid.point.x;
  centre[1]=sphereCentroid.point.y;
  centre[2]=sphereCentroid.point.z;

}

double pointsDist(geometry_msgs::Point center1, geometry_msgs::Point center3){

  double eu_dist, x_dist, y_dist, z_dist;
  x_dist = center1.x - center3.x;
  y_dist = center1.y - center3.y;
  z_dist = center1.z - center3.z;
  eu_dist = sqrt( pow(x_dist,2) + pow(y_dist,2) + pow(z_dist,2) );

  return eu_dist;
}

/**
   @brief Find circle on laser data
   @param[in] clusters segmented scan from the laser
   @param[out] circleP point coordinates of the circle detected for representation on rviz
   @param[in] layer number of the scan
   @return double radius of the detected circle
 */
double find_circle(vector<ClusterPtr> clusters, vector<ClusterPtr>& circleP, int ring)
{
  int count=0;
  double radius=0;
  bool checkCircle = false;
  double Angle;
  if(ring<8){
    Angle = (((double)(ring)*30/16)-15);
  }else{
    Angle = (((double)(ring+1)*30/16)-15);
  }
  //cout << "Angle = " << Angle << endl;
  for(int k=0; k<clusters.size(); k++)
  {
    ClusterPtr cluster=clusters[k];
    int segment_init, segment_end;
    for (int i=0; i<cluster->support_points.size(); i++)
    {

      segment_init=0;
      segment_end=cluster->support_points.size()-1;

      // Detect if at least 6 points
      double angle[1000];
      double vecA[3],vecB[3];
      int n=0;
      if (segment_end-segment_init>=10)
      {
        for (int j=segment_init+1; j<segment_end-1; j++)
        {
          // use 3D but actually Z = 0 because laserscan is planar
          vecA[0] = cluster->support_points[segment_init]->x - cluster->support_points[j]->x;
          vecA[1] = cluster->support_points[segment_init]->y - cluster->support_points[j]->y;
          vecA[2] = cluster->support_points[segment_init]->z - cluster->support_points[j]->z;

          vecB[0] = cluster->support_points[segment_end]->x - cluster->support_points[j]->x;
          vecB[1] = cluster->support_points[segment_end]->y - cluster->support_points[j]->y;
          vecB[2] = cluster->support_points[segment_end]->z - cluster->support_points[j]->z;

          angle[n] = acos( inner_product(vecA, vecA+3, vecB, 0.0)
                           / (sqrt(pow(vecA[0],2) + pow(vecA[1],2) + pow(vecA[2],2)) * sqrt(pow(vecB[0],2) + pow(vecB[1],2) + pow(vecB[2],2))));
          n++;
        }
        // compute average angle and std
        double m=0;
        double std=0;

        for (int j = 0; j < n; j++)
          m += angle[j];

        m = m/n;
        for ( int j = 0; j < n; j++ )
        {
          std += pow((angle[j]-m),2);
        }

        std = sqrt(std/(n-1));

        // conversion to degree
        m = m/M_PI*180;
        std = std/M_PI*180;

        //if (m>90 && m<135 && std < 8.6)
        //if (m>90 && m<145 && std < 12) // ATLASCAR
        if (m>90 && m<135 && std < 8.5)
        {
          // std::cout << "std = " << std << std::endl;
          // std::cout << "m = " << m << std::endl;
          double ma,mb,cx,x1,x2,x3,cy,y1,y2,y3,z1,z2,z3;
          x1=cluster->support_points[segment_init]->x;
          x2=cluster->support_points[round(segment_end/2)]->x;
          x3=cluster->support_points[segment_end]->x;
          y1=cluster->support_points[segment_init]->y;
          y2=cluster->support_points[round(segment_end/2)]->y;
          y3=cluster->support_points[segment_end]->y;
          z1=cluster->support_points[segment_init]->z;
          z2=cluster->support_points[round(segment_end/2)]->z;
          z3=cluster->support_points[segment_end]->z;

          //rotate da points to da plane XY
          for(int i=0; i<cluster->support_points.size(); i++)
            rotatePoints(cluster->support_points[i]->x,cluster->support_points[i]->y, cluster->support_points[i]->z, Angle);

          Point centroid;
          double R;
          CalculateCircle(cluster,R,centroid);

          double z=0;
          rotatePoints(centroid.x,centroid.y,z,-Angle);

          for(int i=0; i<cluster->support_points.size(); i++)
            rotatePoints(cluster->support_points[i]->x,cluster->support_points[i]->y, cluster->support_points[i]->z, -Angle);

          rotatePoints(x1,y1, z1, Angle);
          rotatePoints(x2,y2, z2, Angle);
          rotatePoints(x3,y3, z3, Angle);

          ma=(y2-y1)/(x2-x1);
          mb=(y3-y2)/(x3-x2);

          cx=(ma*mb*(y1-y3)+mb*(x1+x2)-ma*(x2+x3))/(2*(mb-ma));
          cy=-1/ma*(cx-(x1+x2)/2)+(y1+y2)/2;

          radius=sqrt(pow((cx-x1),2) + pow((cy-y1),2));
          radius=R;

          rotatePoints(cx,cy,z1,-Angle);
          double circle[3];
          circle[0]=centroid.x;
          circle[1]=centroid.y;
          circle[2]=z;

          double centre[3];
          centre[0] = circle[0];
          centre[1] = circle[1];
          centre[2] = circle[2];

          sphereCentroid.point.x=circle[0];
          sphereCentroid.point.y=circle[1];
          sphereCentroid.point.z=circle[2];

          circlePoints(circleP,radius,centre,20);
          if(!circleP.empty())
            circleP[count]->centroid=cluster->centroid;
          count++;
          checkCircle=true;
        }
        else
        {
          if(checkCircle==false)
          {
            sphereCentroid.point.x=-999;
            sphereCentroid.point.y=-999;
            sphereCentroid.point.z=-999;
          }
        }
      }
    }
  }
//  cout<<"R "<<radius<<endl;
  return radius;
}

/**
   @brief Rotate a point
   @param[out] x x coordinate
   @param[out] y y coordinate
   @param[out] z z coordinate
   @param[out] angle angle to rotate
   @return void
 */
void rotatePoints(double& x,double& y, double& z, double angle)
{
  double X=x,Y=y,Z=z;
  x=X*cos(angle*M_PI/180) + Z*sin(angle*M_PI/180);
  z=X*(-sin(angle*M_PI/180)) + Z*cos(angle*M_PI/180);
  y=Y;
}

/**
   @brief Detection of the ball on the Kinect data
   @param[in] Kinect_cloud point cloud from the Kinect
   @return void
 */
void sphereDetection(pcl::PointCloud<pcl::PointXYZ> Kinect_cloud)
{

  geometry_msgs::PointStamped sphereCenter;
  sphereCenter.point.x = -999;
  sphereCenter.point.y = -999;
  sphereCenter.point.z = -999;

  /* METHOD #3 ================================================================
   * Detects the ball up to 3 meters, fast. Optimized Method #1
   */
  pcl::PointCloud<pcl::PointXYZ>::Ptr Kinect_cloudPtr (new pcl::PointCloud<pcl::PointXYZ>(Kinect_cloud));

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
  seg.setDistanceThreshold (0.01); //0.0070936
  seg.setProbability(0.99);
  seg.setRadiusLimits (BALL_DIAMETER/2-1, BALL_DIAMETER/2+1);
  seg.setInputCloud (Kinect_cloudPtr);
  seg.segment (*inliers, *coefficients);

  if(inliers->indices.size ()>50)
  {
    if (coefficients->values[3]<BALL_DIAMETER/2 + 0.5*BALL_DIAMETER/2 && coefficients->values[3]>BALL_DIAMETER/2 - 0.5*BALL_DIAMETER/2)
    {
      sphereCenter.point.x = coefficients->values[0];
      sphereCenter.point.y = coefficients->values[1];
      sphereCenter.point.z = coefficients->values[2];

      cout << "Accepted: " << *coefficients << endl;
    }
  }

  // Ball detection ends here =================================================

  pcl::PointXYZ center;
  center.x = sphereCenter.point.x;
  center.y = sphereCenter.point.y;
  center.z = sphereCenter.point.z;

  sphereCenter.header.stamp = ros::Time::now();
  sphereCentroid_pub.publish(sphereCenter);

  visualization_msgs::MarkerArray targets_markers;
  targets_markers.markers = createTargetMarkers(center);
  velodyne_pub.publish(targets_markers);
}


namespace velodyne {
class velodyne_BD_RANSAC
{
public:
  velodyne_BD_RANSAC(string topicName) {
    // subscribe to VelodyneScan packets
    velodyne_scan =
      node.subscribe(topicName, 10,
                     &velodyne_BD_RANSAC::processScan, this);
    ROS_INFO("Topic %s subscribed!",topicName.c_str());
  }

private:
  NodeHandle node;
  Subscriber velodyne_scan;

  void processScan(const sensor_msgs::PointCloud2::ConstPtr &scanMsg){
    pcl::PointCloud<pcl::PointXYZ> pclCloud;
    pcl::fromROSMsg(*scanMsg, pclCloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_Ptr (new pcl::PointCloud<pcl::PointXYZ>(pclCloud));
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_filtered (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PassThrough<pcl::PointXYZ> x_filter;
    x_filter.setInputCloud(pcl_Ptr);
    x_filter.setFilterFieldName("x");
    x_filter.setFilterLimits(0,30);
    x_filter.filter(*pcl_filtered);

     // Build the condition
    pcl::ConditionOr<pcl::PointXYZ>::Ptr range_and1 (new pcl::ConditionOr<pcl::PointXYZ> ());
    range_and1->addComparison (pcl::FieldComparison<pcl::PointXYZ>::Ptr (new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::GT, 0.8)));
    range_and1->addComparison (pcl::FieldComparison<pcl::PointXYZ>::Ptr (new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::GT, 0.8)));

    pcl::ConditionOr<pcl::PointXYZ>::Ptr range_and2 (new pcl::ConditionOr<pcl::PointXYZ> ());
    range_and2->addComparison (pcl::FieldComparison<pcl::PointXYZ>::Ptr (new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::LT, -0.8)));
    range_and2->addComparison (pcl::FieldComparison<pcl::PointXYZ>::Ptr (new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::LT, -0.8)));

    pcl::ConditionOr<pcl::PointXYZ>::Ptr range_and3 (new pcl::ConditionOr<pcl::PointXYZ> ());
    range_and3->addComparison (pcl::FieldComparison<pcl::PointXYZ>::Ptr (new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::GT, 0.8)));
    range_and3->addComparison (pcl::FieldComparison<pcl::PointXYZ>::Ptr (new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::LT, -0.8)));

    pcl::ConditionOr<pcl::PointXYZ>::Ptr range_and4 (new pcl::ConditionOr<pcl::PointXYZ> ());
    range_and4->addComparison (pcl::FieldComparison<pcl::PointXYZ>::Ptr (new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::LT, -0.8)));
    range_and4->addComparison (pcl::FieldComparison<pcl::PointXYZ>::Ptr (new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::GT, 0.8)));

    pcl::ConditionOr<pcl::PointXYZ>::Ptr range (new pcl::ConditionOr<pcl::PointXYZ> ());
    range->addCondition (range_and1);
    range->addCondition (range_and2);
    range->addCondition (range_and3);
    range->addCondition (range_and4);

    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_filtered2 (new pcl::PointCloud<pcl::PointXYZ>);
    // Build the filter
     pcl::ConditionalRemoval<pcl::PointXYZ> condrem(true);
     condrem.setCondition (range);
     condrem.setInputCloud (pcl_filtered);
//     condrem.setKeepOrganized(true);
     // apply filter
     condrem.filter (*pcl_filtered2);

    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_filtered_nground (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.1);
    seg.setInputCloud (pcl_filtered2);
    seg.segment (*inliers, *coefficients);

    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZ> extract(false);
    extract.setInputCloud(pcl_filtered);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*pcl_filtered_nground);

    pcl::PointCloud<pcl::PointXYZ> pcl_clean = *pcl_filtered2;


    sphereDetection(pcl_clean);
  }
};


class velodyne_BD
{
public:
  velodyne_BD(string topicName, vector< vector<PointPtr> > laserscans) {
    this->laserscans = laserscans;

    // subscribe to VelodyneScan packets
    velodyne_scan =
      node.subscribe(topicName, 10,
                     &velodyne_BD::processScan, this);
    ROS_INFO("Topic %s subscribed!",topicName.c_str());

    used_planes.assign(16, 0);
    for(int i = 0; i<8;i++){
      used_planes[i]=1;
    }
  }

  void pcl2ToLaserPoints(const sensor_msgs::PointCloud2::ConstPtr &pcl2, vector< vector<PointPtr> > &laserscan){
    vector<PointPtr> scanPtr;
    laserscan.assign(16, scanPtr);

    pcl::PointCloud<VPoint> pcl;
    pcl::fromROSMsg(*pcl2, pcl);

    pcl::PointCloud<pcl::PointXYZI> pcl_xyz;
    pcl::copyPointCloud(pcl, pcl_xyz);
    for(int i = 0; i<pcl.size();i++){
      pcl_xyz[i].intensity = pcl[i].ring;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_Ptr (new pcl::PointCloud<pcl::PointXYZI>(pcl_xyz));
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_filtered (new pcl::PointCloud<pcl::PointXYZI>);

    pcl::PassThrough<pcl::PointXYZI> x_filter;
    x_filter.setInputCloud(pcl_Ptr);
    x_filter.setFilterFieldName("x");
    x_filter.setFilterLimits(0,30);
    x_filter.filter(*pcl_filtered);

     // Build the condition
    pcl::ConditionOr<pcl::PointXYZI>::Ptr range_and1 (new pcl::ConditionOr<pcl::PointXYZI> ());
    range_and1->addComparison (pcl::FieldComparison<pcl::PointXYZI>::Ptr (new pcl::FieldComparison<pcl::PointXYZI>("y", pcl::ComparisonOps::GT, 0.8)));
    range_and1->addComparison (pcl::FieldComparison<pcl::PointXYZI>::Ptr (new pcl::FieldComparison<pcl::PointXYZI>("x", pcl::ComparisonOps::GT, 0.8)));

    pcl::ConditionOr<pcl::PointXYZI>::Ptr range_and2 (new pcl::ConditionOr<pcl::PointXYZI> ());
    range_and2->addComparison (pcl::FieldComparison<pcl::PointXYZI>::Ptr (new pcl::FieldComparison<pcl::PointXYZI>("y", pcl::ComparisonOps::LT, -0.8)));
    range_and2->addComparison (pcl::FieldComparison<pcl::PointXYZI>::Ptr (new pcl::FieldComparison<pcl::PointXYZI>("x", pcl::ComparisonOps::LT, -0.8)));

    pcl::ConditionOr<pcl::PointXYZI>::Ptr range_and3 (new pcl::ConditionOr<pcl::PointXYZI> ());
    range_and3->addComparison (pcl::FieldComparison<pcl::PointXYZI>::Ptr (new pcl::FieldComparison<pcl::PointXYZI>("y", pcl::ComparisonOps::GT, 0.8)));
    range_and3->addComparison (pcl::FieldComparison<pcl::PointXYZI>::Ptr (new pcl::FieldComparison<pcl::PointXYZI>("x", pcl::ComparisonOps::LT, -0.8)));

    pcl::ConditionOr<pcl::PointXYZI>::Ptr range_and4 (new pcl::ConditionOr<pcl::PointXYZI> ());
    range_and4->addComparison (pcl::FieldComparison<pcl::PointXYZI>::Ptr (new pcl::FieldComparison<pcl::PointXYZI>("y", pcl::ComparisonOps::LT, -0.8)));
    range_and4->addComparison (pcl::FieldComparison<pcl::PointXYZI>::Ptr (new pcl::FieldComparison<pcl::PointXYZI>("x", pcl::ComparisonOps::GT, 0.8)));

    pcl::ConditionOr<pcl::PointXYZI>::Ptr range (new pcl::ConditionOr<pcl::PointXYZI> ());
    range->addCondition (range_and1);
    range->addCondition (range_and2);
    range->addCondition (range_and3);
    range->addCondition (range_and4);

    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_filtered2 (new pcl::PointCloud<pcl::PointXYZI>);
    // Build the filter
     pcl::ConditionalRemoval<pcl::PointXYZI> condrem(true);
     condrem.setCondition (range);
     condrem.setInputCloud (pcl_filtered);
//     condrem.setKeepOrganized(true);
     // apply filter
     condrem.filter (*pcl_filtered2);

    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_filtered_nground (new pcl::PointCloud<pcl::PointXYZI>);

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZI> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.1);
    seg.setInputCloud (pcl_filtered2);
    seg.segment (*inliers, *coefficients);

    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZI> extract(false);
    extract.setInputCloud(pcl_filtered);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*pcl_filtered_nground);

    pcl::PointCloud<pcl::PointXYZI> pcl_clean = *pcl_filtered2;

    for(int i = 0; i<pcl_clean.size();i++){
      pcl::PointXYZI point = pcl_clean.at(i);
      int ringNumber = point.intensity;
//      ROS_INFO("%d",ringNumber);
      PointPtr pointXYZ(new Point);
      pointXYZ->x = point.x;
      pointXYZ->y = point.y;
      pointXYZ->z = point.z;

      laserscan[ringNumber].push_back(pointXYZ);
    }
//    ROS_INFO("Points: %d",laserscans[0].size());
  }

  void processScan(const sensor_msgs::PointCloud2::ConstPtr &scanMsg){
    pcl2ToLaserPoints(scanMsg, laserscans);
    vector< vector<PointPtr> > laserscans_clean;
    for(int i = 0; i<laserscans.size();i++){
      if(used_planes[i]==1){
        laserscans_clean.push_back(laserscans[i]);
      }
    }

    velodyne_findBall(laserscans_clean);
  }

private:
  NodeHandle node;
  Subscriber velodyne_scan;
  vector< vector<PointPtr> > laserscans;
  vector<int> used_planes;
};
}


using namespace velodyne;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "velodyne_vlp");

  ros::NodeHandle nh("~");
  string node_ns = ros::this_node::getNamespace();
  node_ns.erase(0, 2);
  nh.getParam("ballDiameter", BALL_DIAMETER);

  cout << "Node namespace:" << node_ns << endl;
  cout << "Ball diameter:" << BALL_DIAMETER << endl;

  velodyne_pub = nh.advertise<visualization_msgs::MarkerArray>( "BallDetection", 10000);
  sphereCentroid_pub = nh.advertise<geometry_msgs::PointStamped>("SphereCentroid",1000);

  /*--------------------------------------------------------------*/
  /*----------Opção 1- Deteção da bola em scans planeres----------*/
  /*--------------------------------------------------------------*/
  vector< vector<PointPtr> > laserscans;
  velodyne_BD vlp_BD("velodyne_points",laserscans);

  /*--------------------------------------------------------------*/
  /*------------Opção 2- Deteção da bola usando RANSAC------------*/
  /*--------------------------------------------------------------*/
//  velodyne_BD_RANSAC vlp_BD_RANSAC("velodyne_points");

  ros::spin();

  return 0;
}
