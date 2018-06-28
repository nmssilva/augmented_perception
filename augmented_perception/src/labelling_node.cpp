#include <iostream>

#include <QApplication>
#include <QPushButton>
#include <QLabel>
#include <QComboBox>

#include "laser_geometry/laser_geometry.h"
#include "tf/message_filter.h"
#include <ros/package.h>

#include "mtt/TargetList.h"
#include "mtt/mtt.h"

#include "pcl_ros/transforms.h"

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <opencv2/opencv.hpp>

#include "rosbag/bag_player.h"
#include "rosbag/player.h"

#include "rqt_bag/Pause.h"

#include "common.cpp"


using namespace std;
using namespace cv;

// ROS

// Publishers
ros::Publisher pub_targets;
ros::Publisher markers_publisher;
ros::Publisher pub_targetsSug;
ros::Publisher markers_publisherSug;

ros::Publisher pub_scans;
ros::Publisher pub_scans_filtered;
ros::Publisher pub_scans_suggest;

ros::Publisher camera_lines_pub;

image_transport::Publisher pc_image_proj;
image_transport::Publisher box3d_image_proj;
image_transport::Publisher box2d_image_proj;


// Images
cv_bridge::CvImagePtr cv_ptr;
Mat image_input, imToShow, sub, projection, projectionPC;

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
int match_method =
		CV_TM_SQDIFF; // 0: CV_TM_SQDIFF 1: CV_TM_SQDIFF_NORMED 2: CV_TM_CCORR
// 3: CV_TM_CCORR_NORMED 4: CV_TM_CCOEFF 5: CV_TM_CCOEFF_NORMED
int nframes = 0;
bool capture = false;
bool drawRect = false;
bool got_last_patches = false;
std::queue<Mat> frame_array;
std::queue<Mat> previous_frames;
std::queue<Mat> first_previous_frames;
std::queue<Mat> previous_patches;

// Scanner MTT related variables
pcl::PointCloud<pcl::PointXYZ> pointDatapclSug, pointDatapcl, pointData0pcl, pointData1pcl,
		pointData2pcl, pointData3pcl, pointDataEpcl, pointDataDpcl;
sensor_msgs::PointCloud2 pointDataSug, pointData, pointData0, pointData1, pointData2,
		pointData3, pointDataE, pointDataD;

tf::StampedTransform transformD, transformE;
bool init_transforms = true;

mtt::TargetListPC targetList;

t_config config;
t_data full_data;
t_flag flags;

vector<t_clustersPtr> clusters;
vector<t_objectPtr> object;
vector<t_listPtr> list_vector;

visualization_msgs::MarkerArray markersMsg;

unsigned int tracking_bbox_id;

// Suggestion MTT related variables

mtt::TargetListPC targetListSug;

t_config configSug;
t_data full_dataSug;
t_flag flagsSug;

vector<t_clustersPtr> clustersSug;
vector<t_objectPtr> objectSug;
vector<t_listPtr> list_vectorSug;

visualization_msgs::MarkerArray markersMsgSug;

bool prevFoundSug = false;
bool manual = false;
bool full_manual = true;

// Fusion related variables

float proportion;
bool click_on = false;
uint click_count_reset = 0;

// Color segmentation related variables

float hue = 0.0, saturation = 0.0, value = 0.0;

// 3D projection
cv::Mat rvec = cv::Mat(3, 1, CV_32FC1);
cv::Mat tvec = cv::Mat(3, 1, CV_32FC1);

cv::Mat distCoeffs = cv::Mat(5, 1, CV_32FC1);
cv::Mat cameraMatrix = cv::Mat(3, 3, CV_32FC1);

// File writing related variables

struct BBox {
	int x;
	int y;
	int width;
	int height;
	int id;
	string label;
	// 3D position
	double x3d, y3d, z3d;
};

std::map<unsigned int, std::vector<BBox> > file_map;
unsigned int object_id = 0;
unsigned int first_frame_id;

// Service related variables

ros::ServiceClient client;
rqt_bag::Pause srv;

// GUI

QPushButton *button1;
QPushButton *button2;
QPushButton *button3;
QPushButton *button4;
QPushButton *button5;
QPushButton *button6;

QPushButton *button21;
QPushButton *button22;
QComboBox * cb1;

bool window2visible = false;


Mat MatchingMethod(int, void *, Mat patch_frame, Mat previous_frame) {
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

	if (match_method == TM_SQDIFF || match_method == TM_SQDIFF_NORMED) {
		matchLoc = minLoc;
	} else {
		matchLoc = maxLoc;
	}

	cv::Rect myROI(matchLoc.x, matchLoc.y, patch_frame.cols, patch_frame.rows);

	patch_frame = img_display(myROI);

	return patch_frame;
}

void MatchingMethod(int, void *) {
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

	if (match_method == TM_SQDIFF || match_method == TM_SQDIFF_NORMED) {
		matchLoc = minLoc;
	} else {
		matchLoc = maxLoc;
	}

	// update patch
	int patch_size = (50 - box_x) * 6;

	if (patch_size < 9) {
		patch_size = 9;
	}

	/*int roi_width = matchLoc.x + patch_size;
	int roi_heigth = matchLoc.y + patch_size;

	if (roi_width >= sub.cols) {
		roi_width = sub.cols - matchLoc.x;
	} else {
		roi_width = patch_size;
	}

	if (roi_heigth >= sub.rows) {
		roi_heigth = sub.rows - matchLoc.y;
	} else {
		roi_heigth = patch_size;
	}*/

	//cv::Rect myROI(matchLoc.x, matchLoc.y, roi_width, roi_heigth);
	cv::Rect myROI(matchLoc.x, matchLoc.y, 300, 300);
	patch = img_display(myROI);

	unsigned int frame_seq = cv_ptr->header.seq;

	BBox box;
	box.x = matchLoc.x;
	box.y = matchLoc.y;
	box.width = patch.cols;
	box.height = patch.rows;
	box.id = object_id;
	box.label = "DontCare";
	box.x3d = box_x;
	box.y3d = box_y;
	box.z3d = box_z;

	file_map[frame_seq].push_back(box);

	// limit 100
	if (frame_array.size() >= 100) {
		frame_array.pop();
	}

	frame_array.push(patch.clone());

	nframes++;

	rectangle(imToShow, matchLoc,
			  Point(matchLoc.x + patch.cols, matchLoc.y + patch.rows),
			  Scalar(0, 0, 255), 2, 8, 0);

	/*rectangle(result, matchLoc, Point(matchLoc.x + (50 - box_x) * 6,
	matchLoc.y + (50 - box_x) * 6), Scalar::all(0), 2, 8, 0);*/
	imshow("camera", imToShow);
	// imshow("result", result);
	return;
}

static void onMouse_TM(int event, int x, int y, int /*flags*/,
					   void * /*param*/) {
	if (event == EVENT_LBUTTONUP) {
		click_on = true;
		manual = true;
		list_vector.clear();
		// get x
		int get_x = -x + 812;
		// associate x to angle
		float angle = float(get_x) / 27.0;
		// find marker in angle interval
		proportion = tan(angle * 0.01745329252); // converting to radians

		lost = false;

		float bbox_size = (50 - box_x) * 3;

		if (bbox_size < 9.) {
			bbox_size = 9.;
		}

		pointdown = Point2f((float) x - bbox_size, (float) y - bbox_size);
		pointup = Point2f((float) x + bbox_size, (float) y + bbox_size);

		// Point verification
		if (pointup.x < 0)
			pointup.x = 0;

		if (pointup.y < 0)
			pointup.y = 0;

		if (pointdown.x < 0)
			pointdown.x = 0;

		if (pointdown.y < 0)
			pointdown.y = 0;

		if (pointdown.x >= image_input.cols)
			pointdown.x = image_input.cols - 1;

		if (pointdown.y >= image_input.rows)
			pointdown.y = image_input.rows - 1;

		if (pointup.x >= image_input.cols)
			pointup.x = image_input.cols - 1;

		if (pointup.y >= image_input.rows)
			pointup.y = image_input.rows - 1;

		capture = true;
		got_last_patches = false;
		nframes = 0;

		first_frame_id = cv_ptr->header.seq;
		object_id++;

		while (frame_array.size() > 0) {
			frame_array.pop();
		}

		pointbox = Point2f((float) x, (float) y);

		hue = getH(image_input.at<Vec3b>(x, y)[0], image_input.at<Vec3b>(x, y)[1], image_input.at<Vec3b>(x, y)[2]);
		saturation = getS(image_input.at<Vec3b>(x, y)[0], image_input.at<Vec3b>(x, y)[1],
						  image_input.at<Vec3b>(x, y)[2]);
		value = getV(image_input.at<Vec3b>(x, y)[0], image_input.at<Vec3b>(x, y)[1], image_input.at<Vec3b>(x, y)[2]);

		//cout << "hue: " << hue << " saturation: " << saturation << " value: " << value << endl;


	}
}

void filter_suggest() {
	float back_limit = 8;
	float front_limit = 19;
	float left_limit = 4;
	float right_limit = 2;

	for (int i = 0; i < pointDatapclSug.points.size(); i++) {
		if (pointDatapclSug.points[i].y > left_limit ||
			pointDatapclSug.points[i].y < -right_limit ||
			pointDatapclSug.points[i].x < back_limit |
			pointDatapclSug.points[i].x > front_limit) {
			pointDatapclSug.points[i].x = 9999;
			pointDatapclSug.points[i].y = 9999;
			pointDatapclSug.points[i].z = 9999;
		}
	}

}

void filter_pc() {
	float back_limit = 0.1;

	if (click_on) {

		click_count_reset++;
		if (click_count_reset > 2) {
			click_on = false;
			click_count_reset = 0;
		}

		for (int i = 0; i < pointDatapcl.points.size(); i++) {
			if (pointDatapcl.points[i].y >
				pointDatapcl.points[i].x * (proportion + 0.05) ||
				pointDatapcl.points[i].y <
				pointDatapcl.points[i].x * (proportion - 0.05) ||
				pointDatapcl.points[i].x < back_limit) {
				pointDatapcl.points[i].x = 9999;
				pointDatapcl.points[i].y = 9999;
				pointDatapcl.points[i].z = 9999;
			}
		}

		//cerr << "box_id: " << box_id << endl;
		tracking_bbox_id = box_id;
	}
}

void checkIfIDexist() {
	bool id_found = false;
	for (uint i = 0; i < list_vector.size(); i++) {
		if (list_vector[i]->shape.lines.size() != 0) {
			if (tracking_bbox_id == list_vector[i]->id) {
				cout << tracking_bbox_id << ":(" << box_x << ", " << box_y << ")\n";
				id_found = true;
				break;
			}
		}
	}
	if (!id_found) {
		lost = true;
	}
}

void initClouds() {
	pcl::fromROSMsg(pointData0, pointData0pcl);
	pcl::fromROSMsg(pointData1, pointData1pcl);
	pcl::fromROSMsg(pointData2, pointData2pcl);
	pcl::fromROSMsg(pointData3, pointData3pcl);
	pcl::fromROSMsg(pointDataE, pointDataEpcl);
	pcl::fromROSMsg(pointDataD, pointDataDpcl);

	if (init_transforms) {
		init_transforms = false;
		tf::TransformListener listener;

		try {
			ros::Time now = ros::Time(0);

			listener.waitForTransform("/ldmrs0", "/lms151_D", now,
									  ros::Duration(3.0));
			listener.lookupTransform("/ldmrs0", "/lms151_D", now, transformD);

			listener.waitForTransform("/ldmrs0", "/lms151_E", now,
									  ros::Duration(3.0));
			listener.lookupTransform("/ldmrs0", "/lms151_E", now, transformE);
		} catch (tf::TransformException ex) {
			ROS_ERROR("%s", ex.what());
		}
	}

	pcl_ros::transformPointCloud(pointDataDpcl, pointDataDpcl, transformD);
	pcl_ros::transformPointCloud(pointDataEpcl, pointDataEpcl, transformE);

	pointDatapcl = pointData0pcl;
	pointDatapcl += pointData1pcl;
	pointDatapcl += pointData2pcl;
	pointDatapcl += pointData3pcl;
	pointDatapcl += pointDataEpcl;
	pointDatapcl += pointDataDpcl;

	pointDatapclSug = pointData0pcl;
	pointDatapclSug += pointData1pcl;
	pointDatapclSug += pointData2pcl;
	pointDatapclSug += pointData3pcl;
	pointDatapclSug += pointDataEpcl;
	pointDatapclSug += pointDataDpcl;
}

void initMTTSuggest() {

	filter_suggest();

	pcl::toROSMsg(pointDatapclSug, pointDataSug);

	pub_scans_suggest.publish(pointDataSug);

	// Get data from PointCloud2 to full_data
	PointCloud2ToData(pointDataSug, full_dataSug);

	// clustering
	clustering(full_dataSug, clustersSug, &configSug, &flagsSug);

	// calc_cluster_props
	calc_cluster_props(clustersSug, full_dataSug);

	// clusters2objects
	clusters2objects(objectSug, clustersSug, full_dataSug, configSug);

	calc_object_props(objectSug);

	// AssociateObjects
	AssociateObjects(list_vectorSug, objectSug, configSug, flagsSug);

	// MotionModelsIteration
	MotionModelsIteration(list_vectorSug, configSug);

	// cout<<"Number of targets "<<list_vector.size() << endl;

	free_lines(objectSug); // clean current objects

	targetListSug.id.clear();
	targetListSug.obstacle_lines.clear(); // clear all lines

	pcl::PointCloud<pcl::PointXYZ> target_positionsSug;
	pcl::PointCloud<pcl::PointXYZ> velocitySug;

	target_positionsSug.header.frame_id = pointDataSug.header.frame_id;

	velocitySug.header.frame_id = pointDataSug.header.frame_id;

	targetListSug.header.stamp = ros::Time::now();
	targetListSug.header.frame_id = pointDataSug.header.frame_id;

	// cout << "list size: " << list_vector.size() << endl;

	for (uint i = 0; i < list_vectorSug.size(); i++) {
		targetListSug.id.push_back(list_vectorSug[i]->id);

		pcl::PointXYZ positionSug;

		positionSug.x = list_vectorSug[i]->position.estimated_x;
		positionSug.y = list_vectorSug[i]->position.estimated_y;
		positionSug.z = 0;

		target_positionsSug.points.push_back(positionSug);

		pcl::PointXYZ velSug;

		velSug.x = list_vectorSug[i]->velocity.velocity_x;
		velSug.y = list_vectorSug[i]->velocity.velocity_y;
		velSug.z = 0;

		velocitySug.points.push_back(velSug);

		pcl::PointCloud<pcl::PointXYZ> shapeSug;
		pcl::PointXYZ line_pointSug;

		uint j;
		for (j = 0; j < list_vectorSug[i]->shape.lines.size(); j++) {
			line_pointSug.x = list_vectorSug[i]->shape.lines[j]->xi;
			line_pointSug.y = list_vectorSug[i]->shape.lines[j]->yi;

			shapeSug.points.push_back(line_pointSug);
		}

		line_pointSug.x = list_vectorSug[i]->shape.lines[j - 1]->xf;
		line_pointSug.y = list_vectorSug[i]->shape.lines[j - 1]->yf;

		sensor_msgs::PointCloud2 shape_cloudSug;
		pcl::toROSMsg(shapeSug, shape_cloudSug);
		targetListSug.obstacle_lines.push_back(shape_cloudSug);
	}

	pcl::toROSMsg(target_positionsSug, targetListSug.position);
	pcl::toROSMsg(velocitySug, targetListSug.velocity);

	pub_targetsSug.publish(targetListSug);

	CreateMarkersSug(markersMsgSug.markers, targetListSug, list_vectorSug);

	markers_publisherSug.publish(markersMsgSug);

	flagsSug.fi = false;
}

void initMTT() {

	filter_pc();

	checkIfIDexist();

	pcl::toROSMsg(pointDatapcl, pointData);

	pub_scans_filtered.publish(pointData);

	// Get data from PointCloud2 to full_data
	PointCloud2ToData(pointData, full_data);

	// clustering
	clustering(full_data, clusters, &config, &flags);

	// calc_cluster_props
	calc_cluster_props(clusters, full_data);

	// clusters2objects
	clusters2objects(object, clusters, full_data, config);

	calc_object_props(object);

	// AssociateObjects
	AssociateObjects(list_vector, object, config, flags);

	// MotionModelsIteration
	MotionModelsIteration(list_vector, config);

	// cout<<"Number of targets "<<list_vector.size() << endl;

	free_lines(object); // clean current objects

	targetList.id.clear();
	targetList.obstacle_lines.clear(); // clear all lines

	pcl::PointCloud<pcl::PointXYZ> target_positions;
	pcl::PointCloud<pcl::PointXYZ> velocity;

	target_positions.header.frame_id = pointData.header.frame_id;

	velocity.header.frame_id = pointData.header.frame_id;

	targetList.header.stamp = ros::Time::now();
	targetList.header.frame_id = pointData.header.frame_id;

	// cout << "list size: " << list_vector.size() << endl;

	for (uint i = 0; i < list_vector.size(); i++) {
		targetList.id.push_back(list_vector[i]->id);

		pcl::PointXYZ position;

		position.x = list_vector[i]->position.estimated_x;
		position.y = list_vector[i]->position.estimated_y;
		position.z = 0;

		target_positions.points.push_back(position);

		pcl::PointXYZ vel;

		vel.x = list_vector[i]->velocity.velocity_x;
		vel.y = list_vector[i]->velocity.velocity_y;
		vel.z = 0;

		velocity.points.push_back(vel);

		pcl::PointCloud<pcl::PointXYZ> shape;
		pcl::PointXYZ line_point;

		uint j;
		for (j = 0; j < list_vector[i]->shape.lines.size(); j++) {
			line_point.x = list_vector[i]->shape.lines[j]->xi;
			line_point.y = list_vector[i]->shape.lines[j]->yi;

			shape.points.push_back(line_point);
		}

		line_point.x = list_vector[i]->shape.lines[j - 1]->xf;
		line_point.y = list_vector[i]->shape.lines[j - 1]->yf;

		sensor_msgs::PointCloud2 shape_cloud;
		pcl::toROSMsg(shape, shape_cloud);
		targetList.obstacle_lines.push_back(shape_cloud);
	}

	pcl::toROSMsg(target_positions, targetList.position);
	pcl::toROSMsg(velocity, targetList.velocity);

	pub_targets.publish(targetList);

	CreateMarkers(markersMsg.markers, targetList, list_vector);

	markers_publisher.publish(markersMsg);

	flags.fi = false;
}

void drawCameraRangeLine() {
	visualization_msgs::Marker marker;
	marker.header.frame_id = "root";
	marker.header.stamp = ros::Time();
	marker.ns = "camera_range_lines";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::LINE_STRIP;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = 0;
	marker.pose.position.y = 0;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 0.2;
	marker.scale.y = 0.2;
	marker.scale.z = 0.2;
	marker.color.a = 1.0; // Don't forget to set the alpha!
	marker.color.r = 1.0;
	marker.color.g = 1.0;
	marker.color.b = 0.0;

	geometry_msgs::Point p;

	p.x = 100;
	p.y = 60;
	p.z = 0;
	marker.points.push_back(p);

	p.x = 0;
	p.y = 0;
	marker.points.push_back(p);

	p.x = 100;
	p.y = -60;
	marker.points.push_back(p);

	camera_lines_pub.publish(marker);
}

std::vector<cv::Point3f> Generate3DPoints(float x, float y) {
	std::vector<cv::Point3f> points;
	float size = 1.5;
	points.push_back(cv::Point3f(-y * 1.3 + size, -1.5, x * 2 - size));
	points.push_back(cv::Point3f(-y * 1.3 + size, -1.5, x * 2 + size));
	points.push_back(cv::Point3f(-y * 1.3 - size, -1.5, x * 2 + size));
	points.push_back(cv::Point3f(-y * 1.3 - size, -1.5, x * 2 - size));
	points.push_back(cv::Point3f(-y * 1.3 + size, 1, x * 2 - size));
	points.push_back(cv::Point3f(-y * 1.3 + size, 1, x * 2 + size));
	points.push_back(cv::Point3f(-y * 1.3 - size, 1, x * 2 + size));
	points.push_back(cv::Point3f(-y * 1.3 - size, 1, x * 2 - size));
	return points;
}

std::vector<cv::Point3f> Generate3DPointsPC(pcl::PointCloud<pcl::PointXYZ> pcData) {
	std::vector<cv::Point3f> points;
	for (int i = 0; i < pcData.size(); i++) {
		points.push_back(cv::Point3f(pcData.at(i).y, 0.17, pcData.at(i).x));
	}
	return points;
}

Scalar ScalarHSV2BGR(uchar H, uchar S, uchar V) {
	Mat rgb;
	Mat hsv(1, 1, CV_8UC3, Scalar(H, S, V));
	cvtColor(hsv, rgb, CV_HSV2BGR);
	return Scalar(rgb.data[0], rgb.data[1], rgb.data[2]);
}

void image_cb_TemplateMatching(const sensor_msgs::ImageConstPtr &msg) {
	try {
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

		char c = (char) waitKey(10);

		if (c == 'q' || button6->isDown()) {
			exit(0);
		}

		if (c == 'p' || button5->isDown() ) {
			int last_frame_id = -1;
			int actual_frame_id;
			string path =
					ros::package::getPath("augmented_perception") + "/datasets/";
			mkdir(path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

			ofstream myfile;
			string filename =
					path + boost::lexical_cast<std::string>(std::time(NULL)) + ".txt";
			myfile.open(filename.c_str());
			myfile << "FRAME_ID\nBOX_X BOX_Y WIDTH HEIGHT LABEL ID 3D_X 3D_Y 3D_Z\n";

			for (std::map<unsigned int, std::vector<BBox> >::iterator it =
					file_map.begin();
				 it != file_map.end(); ++it) {
				// Skipped frames part
				actual_frame_id = it->first;
				if (it->first - last_frame_id > 1 && it->first - last_frame_id < 5 &&
					last_frame_id >
					-1) { // frames were skipped, need to replicate them...
					--it;       // go back
					for (int i = 1; i < (actual_frame_id - last_frame_id);
						 i++) // for each skipped frame
					{
						myfile << ((it->first) + i) << endl; // write frame id to file
						for (int i = 0; i < (it->second).size(); i++) // write skipped boxes
						{
							myfile << it->second[i].x << " " << it->second[i].y << " "
								   << it->second[i].width << " " << it->second[i].height
								   << " " << it->second[i].label << " " << it->second[i].id
								   << " " << it->second[i].x3d << " " << it->second[i].y3d
								   << " " << it->second[i].z3d << endl;
						}
					}
					++it; // return to actual state
				}

				// Actual frames part
				myfile << it->first << endl;                  // write frame id to file
				for (int i = 0; i < (it->second).size(); i++) // write actual boxes
				{
					myfile << it->second[i].x << " " << it->second[i].y << " "
						   << it->second[i].width << " " << it->second[i].height << " "
						   << it->second[i].label << " " << it->second[i].id << " "
						   << it->second[i].x3d << " " << it->second[i].y3d << " "
						   << it->second[i].z3d << endl;
				}
				last_frame_id = it->first;
			}
			myfile.close();
			ROS_INFO("Saved frames dataset to %s", filename.c_str());
		}

		if (c == 's' || button2->isDown()) {
			int gotframes = nframes;
			if (gotframes > 100)
				gotframes = 100;

			nframes = 0;

			if (gotframes == 0 || patch.empty()) {
				ROS_INFO("There are no frames to save.");
			} else {
				ROS_INFO("Loading templates to save...");

				// get 5 last patches
				if (!got_last_patches) {
					Mat previous_patch = first_patch;
					for (int i = 0; i < 5; i++) {
						previous_patch = MatchingMethod(0, 0, previous_patch,
														first_previous_frames.front());
						first_previous_frames.pop();
						previous_patches.push(previous_patch);
					}
					got_last_patches = true;
				}

				ROS_INFO("Saving %s frames. Please enter object label: (type 'exit' to "
								 "discard frames)",
						 boost::lexical_cast<std::string>(gotframes + 5).c_str());

				string path;
				path = ros::package::getPath("augmented_perception") + "/labelling/";
				mkdir(path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

				cin >> path;

				if (path.find("exit")) {
					path = ros::package::getPath("augmented_perception") + "/labelling/" +
						   path;
					mkdir(path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

					path += "/" + boost::lexical_cast<std::string>(std::time(NULL));
					mkdir(path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

					for (int i = 0; i < gotframes; i++) {
						string impath;
						impath =
								path + "/" + boost::lexical_cast<std::string>(i + 1) + ".bmp";
						imwrite(impath, frame_array.front());
						frame_array.pop();
					}

					int i = 0;
					while (!previous_patches.empty()) {
						i++;
						string impath;
						impath = path + "/previous_" + boost::lexical_cast<std::string>(i) +
								 ".bmp";
						imwrite(impath, previous_patches.front());
						previous_patches.pop();
						previous_frames.pop();
					}

					ROS_INFO("Saved %s frames to %s",
							 boost::lexical_cast<std::string>(gotframes + 5).c_str(),
							 path.c_str());
				} else {
					cout << "Did not save\n";
				}
			}
		}

		string label = "";
		if (c == 'c' || button3->isDown()) {
			patch = Mat();
			ROS_INFO("Image cleared.");
			label = "DontCare";
			c = 'l';
		}

		/*if (lost && patch.cols > 0) {
			ROS_INFO("Lost Track of object.");
			c = 'l';
		}*/

		if((!foundSug && prevFoundSug && !manual && !full_manual)||( changeID && !manual && !full_manual)){
			ROS_INFO("Lost Track of object.");
			c = 'l';
		}

		prevFoundSug = foundSug;

		if (c == 'm' || button4->isDown()) {
			full_manual = !full_manual;
			if(full_manual){
				button4->setText("Semi-automatic Mode");
				ROS_INFO("Manual Mode.");
			}
			else{
				button4->setText("Manual Mode");
				ROS_INFO("Semi-Automatic Mode");
			}
		}

		if (c == 'l' || button1->isDown()) {

			manual = false;
			patch = Mat();

			srv.request.control = "Pause";
			client.call(srv);

			QWidget window2;
			window2.setWindowTitle("Tracking Complete");
			window2.setFixedSize(200, 100);

			cb1 = new QComboBox(&window2);
			QLabel *qlabel = new QLabel("Label: ", &window2);


			button21 = new QPushButton("Discard", &window2);
			button22 = new QPushButton("Save", &window2);

			button21->setGeometry(10, 60, 80, 30);
			button22->setGeometry(110, 60, 80, 30);

			cb1->addItem("car");
			cb1->addItem("van");
			cb1->addItem("people");
			cb1->addItem("bicycle");
			cb1->addItem("sign");
			cb1->addItem("misc");

			qlabel->setGeometry(50, 10, 80, 30);
			cb1->setGeometry(110, 10, 80, 30);

			window2.setVisible(true);

			QEventLoop loop;
			QObject::connect(button21, SIGNAL(clicked()), &loop, SLOT(quit()));
			QObject::connect(button22, SIGNAL(clicked()), &loop, SLOT(quit()));
			loop.exec();

			bool btn21check;
			bool btn22check;
			button21->clicked(btn21check);
			button22->clicked(btn22check);

			if(btn21check || btn22check){

				if(btn21check){
					label = "DontCare";
				}
				if(btn22check){
					label = cb1->currentText().toStdString();
				}

				unsigned int actual_frame_id = cv_ptr->header.seq;

				if (actual_frame_id < first_frame_id) {
					actual_frame_id = first_frame_id + 99999;
				}

				ROS_INFO("Label: %s",label.c_str());

				int count = 0;
				for (std::map<unsigned int, std::vector<BBox> >::iterator it = file_map.begin();
					 it != file_map.end(); ++it) {
					if (it->first >= first_frame_id && it->first <= actual_frame_id)
						for (int i = 0; i < (it->second).size(); i++) {
							(it->second)[i].label = label;
							count ++;
						}
				}
				ROS_INFO("Frame count: %d",count);

				srv.request.control = "Resume";
				client.call(srv);
			}
		}



		// Show image_input
		image_input = cv_ptr->image;
		imToShow = image_input.clone();
		sub = image_input.clone();

		// previous 5 frames
		if (previous_frames.size() >= 5) {
			previous_frames.pop();
		}

		previous_frames.push(image_input);

		if (drawRect) {
			// Draw area-to-crop rectangle (green)
			float max_x, min_x, max_y, min_y;
			if (pointbox.x > pointdown.x) {
				min_x = pointdown.x;
				max_x = pointbox.x;
			} else {
				max_x = pointdown.x;
				min_x = pointbox.x;
			}
			if (pointbox.y > pointdown.y) {
				min_y = pointdown.y;
				max_y = pointbox.y;
			} else {
				max_y = pointdown.y;
				min_y = pointbox.y;
			}

			cv::rectangle(imToShow, Point((int) min_x, (int) min_y),
						  Point((int) max_x, (int) max_y), Scalar(0, 255, 0), 3);
		}

		cv::imshow("camera", imToShow);
		cvSetMouseCallback("camera", onMouse_TM, 0);
	} catch (cv_bridge::Exception &e) {
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
	}

	drawCameraRangeLine();

	initClouds();

	initMTTSuggest();

	// needed
	pcl::toROSMsg(pointDatapcl, pointData);
	pub_scans.publish(pointData);

	if (!lost) {
		initMTT();
	}

	// Draw red rectangle (tracker) positions
	float max_x, min_x, max_y, min_y;
	if (pointup.x > pointdown.x) {
		min_x = pointdown.x;
		max_x = pointup.x;
	} else {
		max_x = pointdown.x;
		min_x = pointup.x;
	}
	if (pointup.y > pointdown.y) {
		min_y = pointdown.y;
		max_y = pointup.y;
	} else {
		max_y = pointdown.y;
		min_y = pointup.y;
	}

	// Draw blue rectangle (suggestion) positions
	if((!manual && !full_manual)){
		float angleSug = atan(box_ySug / box_xSug) * 0.9;
		int xSug = -(angleSug / 0.01745329252 * 27.0) + 812;

		if (foundSug) {
			float size = 500 - 12.5 * distanceSug;
			rectangle(imToShow, Point(xSug - size / 2, 693 - size / 2),
					  Point(xSug + size / 2, 693 + size / 2), Scalar(255, 0, 0), 3);
			imshow("camera", imToShow);
		}
	}

	// get first patch and previous frames
	if(foundSug && changeID && !manual && !full_manual){
		ROS_INFO("Tracking found object.");
		first_frame_id = cv_ptr->header.seq;
		object_id++;
	}
	if ((max_x - min_x > 0 && max_y - min_y > 0 && capture)) {
		cv::Rect myROI(min_x, min_y, max_x - min_x, max_y - min_y);
		patch = image_input(myROI);
		first_patch = patch.clone();
		while (!previous_frames.empty()) {
			first_previous_frames.push(previous_frames.front());
			previous_frames.pop();
		}

		capture = false;
		drawRect = false;
	}
	if(foundSug && !manual && !full_manual){

		unsigned int frame_seq = cv_ptr->header.seq;

		float angleSug = atan(box_ySug / box_xSug) * 0.9;
		int xSug = -(angleSug / 0.01745329252 * 27.0) + 812;
		float size = 500 - 12.5 * distanceSug;

		BBox box;
		box.x = xSug - size / 2;
		box.y = 693 - size / 2;
		box.width = size;
		box.height = size;
		box.id = object_id;
		box.label = "DontCare";
		box.x3d = box_xSug;
		box.y3d = box_ySug;
		box.z3d = box_zSug;

		file_map[frame_seq].push_back(box);
	}

	// up half ignore
	for (int y = 0; y < sub.rows / 3; y++) {
		for (int x = 0; x < sub.cols; x++) {
			sub.at<Vec3b>(Point(x, y))[0] = 0;
			sub.at<Vec3b>(Point(x, y))[1] = 0;
			sub.at<Vec3b>(Point(x, y))[2] = 0;
		}
	}

	if (!patch.empty()) {
		//imshow("crop", patch);
		MatchingMethod(0, 0);
	}

	// 3D Box reprojection part
	image_input.copyTo(projection);

	for (int i = 0; i < 3; i++) {
		rvec.at<float>(i) = 0;
		tvec.at<float>(i) = 0;
	}

	//TODO: read from file
	distCoeffs.at<float>(0) = -0.2015966527847064;
	distCoeffs.at<float>(1) = 0.1516937421259596;
	distCoeffs.at<float>(2) = -0.0009340794635090795;
	distCoeffs.at<float>(3) = -0.0006787308984611241;
	distCoeffs.at<float>(4) = 0;

	cameraMatrix.at<float>(0) = 1454.423376687359; //fx
	cameraMatrix.at<float>(4) = 1458.005828758985; //fy
	cameraMatrix.at<float>(2) = 822.9545738617143; //cx
	cameraMatrix.at<float>(5) = 590.5652711935882; //cy
	cameraMatrix.at<float>(8) = 1;

	cameraMatrix.at<float>(1) = 0;
	cameraMatrix.at<float>(3) = 0;
	cameraMatrix.at<float>(6) = 0;
	cameraMatrix.at<float>(7) = 0;

	if (foundSug) {
		// create cube points
		std::vector<cv::Point3f> o_points = Generate3DPoints(box_xSug, box_ySug);
		// position cube
		std::vector<cv::Point2f> projectedPoints;
		cv::projectPoints(o_points, rvec, tvec, cameraMatrix, distCoeffs, projectedPoints);

		for (int i = 0; i < projectedPoints.size(); i++) {
			projectedPoints.at(i).y += cameraMatrix.at<float>(5) * 0.2;
			projectedPoints.at(i).x -= cameraMatrix.at<float>(5) * 0.09;
		}

		//draw cube lines
		cv::line(projection, projectedPoints.at(0), projectedPoints.at(1), cv::Scalar(255, 0, 0), 2, 8); // blue base
		cv::line(projection, projectedPoints.at(1), projectedPoints.at(2), cv::Scalar(255, 0, 0), 2, 8);
		cv::line(projection, projectedPoints.at(2), projectedPoints.at(3), cv::Scalar(255, 0, 0), 2, 8);
		cv::line(projection, projectedPoints.at(3), projectedPoints.at(0), cv::Scalar(255, 0, 0), 2, 8);
		cv::line(projection, projectedPoints.at(1), projectedPoints.at(5), cv::Scalar(0, 255, 0), 2, 8); // green lines
		cv::line(projection, projectedPoints.at(2), projectedPoints.at(6), cv::Scalar(0, 255, 0), 2, 8);
		cv::line(projection, projectedPoints.at(4), projectedPoints.at(0), cv::Scalar(0, 255, 0), 2, 8);
		cv::line(projection, projectedPoints.at(7), projectedPoints.at(3), cv::Scalar(0, 255, 0), 2, 8);
		cv::line(projection, projectedPoints.at(5), projectedPoints.at(6), cv::Scalar(0, 0, 255), 2, 8); // red top
		cv::line(projection, projectedPoints.at(6), projectedPoints.at(7), cv::Scalar(0, 0, 255), 2, 8);
		cv::line(projection, projectedPoints.at(5), projectedPoints.at(4), cv::Scalar(0, 0, 255), 2, 8);
		cv::line(projection, projectedPoints.at(4), projectedPoints.at(7), cv::Scalar(0, 0, 255), 2, 8);
	}

	// imshow("projection", projection);

	// Publish the data.
	cv_bridge::CvImage out_msg;
	out_msg.header = msg->header;                           // Same timestamp and tf frame as input image
	out_msg.encoding = sensor_msgs::image_encodings::BGR8;  // Or whatever
	out_msg.image = projection;                           // Your cv::Mat

	box3d_image_proj.publish(out_msg.toImageMsg());

	// 3D PC reprojection part

	image_input.copyTo(projectionPC);

	for (int i = 0; i < 3; i++) {
		rvec.at<float>(i) = 0;
		tvec.at<float>(i) = 0;
	}

	// create cube points
	std::vector<cv::Point3f> o_points = Generate3DPointsPC(pointDatapcl);

	// position cube
	std::vector<cv::Point2f> projectedPoints;
	cv::projectPoints(o_points, rvec, tvec, cameraMatrix, distCoeffs, projectedPoints);


	for (int i = 0; i < projectedPoints.size(); i++) {
		projectedPoints.at(i).y += cameraMatrix.at<float>(5) * 0.2;
		projectedPoints.at(i).x -= cameraMatrix.at<float>(5) * 0.09;
	}

	//draw points
	for (int i = 0; i < projectedPoints.size(); i++) {
		if (projectedPoints.at(i).x >= 0 && projectedPoints.at(i).y - 15 >= 0
			&& projectedPoints.at(i).x < projectionPC.cols && projectedPoints.at(i).y - 15 < projectionPC.rows) {
			circle(projectionPC, Point(-projectedPoints.at(i).x + projectionPC.cols, projectedPoints.at(i).y - 15), 3,
				   ScalarHSV2BGR(o_points.at(i).z, 255, 255), -1);

		}
	}

	// Publish the data.
	out_msg.header = msg->header;                           // Same timestamp and tf frame as input image
	out_msg.encoding = sensor_msgs::image_encodings::BGR8;  // Or whatever
	out_msg.image = projectionPC;                           // Your cv::Mat

	pc_image_proj.publish(out_msg.toImageMsg());

}

void laserToPC2(const sensor_msgs::LaserScan::ConstPtr &input) {

	laser_geometry::LaserProjection projector;

	if (input->header.frame_id == "/ldmrs0") {
		projector.projectLaser(*input, pointData0);
	}
	if (input->header.frame_id == "/ldmrs1") {
		projector.projectLaser(*input, pointData1);
	}
	if (input->header.frame_id == "/ldmrs2") {
		projector.projectLaser(*input, pointData2);
	}
	if (input->header.frame_id == "/ldmrs3") {
		projector.projectLaser(*input, pointData3);
	}
	if (input->header.frame_id == "lms151_E") {
		projector.projectLaser(*input, pointDataE);
	}
	if (input->header.frame_id == "lms151_D") {
		projector.projectLaser(*input, pointDataD);
	}
}

int main(int argc, char **argv) {

	// Create Camera Windows
	cv::namedWindow("camera", CV_WINDOW_NORMAL);
	cv::resizeWindow("camera", 800, 666);
	cv::startWindowThread();

	/*cv::namedWindow("projection", CV_WINDOW_NORMAL);
	cv::resizeWindow("projection", 800, 666);
	cv::startWindowThread();


	cv::namedWindow("Full Pointcloud Data", CV_WINDOW_NORMAL);
	cv::resizeWindow("Full Pointcloud Data", 800, 666);
	cv::startWindowThread();

	cv::namedWindow("crop", CV_WINDOW_NORMAL);
	cv::startWindowThread();*/

	QWidget window;
	window.setWindowTitle("Labelling Tool");
	window.setFixedSize(800, 50);

	button1 = new QPushButton("Label Object", &window);
	button2 = new QPushButton("Save Templates", &window);
	button3 = new QPushButton("Clear Image", &window);
	button4 = new QPushButton("Semi-automatic Mode", &window);
	button5 = new QPushButton("Print Dataset", &window);
	button6 = new QPushButton("Quit", &window);

	button1->setGeometry(10, 10, 100, 30);
	button2->setGeometry(130, 10, 130, 30);
	button3->setGeometry(280, 10, 100, 30);
	button4->setGeometry(400, 10, 160, 30);
	button5->setGeometry(580, 10, 100, 30);
	button6->setGeometry(700, 10, 80, 30);

	window.show();



	// Initialize ROS
	ros::init(argc, argv, "labelling_node");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);

	client = nh.serviceClient<rqt_bag::Pause>("pause");

	// Create a ROS publisher for the output point cloud
	pub_scans = nh.advertise<sensor_msgs::PointCloud2>("/pointcloud/all", 1000);
	pub_scans_filtered = nh.advertise<sensor_msgs::PointCloud2>("/pointcloud/filtered", 1000);
	pub_scans_suggest = nh.advertise<sensor_msgs::PointCloud2>("/pointcloud/suggest", 1000);
	pub_targets = nh.advertise<mtt::TargetListPC>("/targets", 1000);
	markers_publisher = nh.advertise<visualization_msgs::MarkerArray>("/markers", 1000);
	pub_targetsSug = nh.advertise<mtt::TargetListPC>("/targetsSug", 1000);
	markers_publisherSug = nh.advertise<visualization_msgs::MarkerArray>("/markersSug", 1000);
	camera_lines_pub = nh.advertise<visualization_msgs::Marker>("/camera_range_lines", 0);

	pc_image_proj = it.advertise("image/pc_projection", 1);
	box3d_image_proj = it.advertise("image/box3d_projection", 1);
	box2d_image_proj = it.advertise("image/box2d_projection", 1);

	// Create a ROS subscriber for the inputs
	ros::Subscriber sub_scan_0 = nh.subscribe("/ld_rms/scan0", 1, laserToPC2);
	ros::Subscriber sub_scan_1 = nh.subscribe("/ld_rms/scan1", 1, laserToPC2);
	ros::Subscriber sub_scan_2 = nh.subscribe("/ld_rms/scan2", 1, laserToPC2);
	ros::Subscriber sub_scan_3 = nh.subscribe("/ld_rms/scan3", 1, laserToPC2);
	ros::Subscriber sub_scan_D = nh.subscribe("/lms151_D_scan", 1, laserToPC2);
	ros::Subscriber sub_scan_E = nh.subscribe("/lms151_E_scan", 1, laserToPC2);

	image_transport::Subscriber sub_image = it.subscribe("/camera/image_color", 1, image_cb_TemplateMatching);

	init_flags(&flags);   // Inits flags values
	init_config(&config); // Inits configuration values

	init_flags(&flagsSug);   // Inits flags values
	init_config(&configSug); // Inits configuration values


	cout << "Keyboard Controls:\n";
	cout << "[Q]uit\n[C]lear image\n[L]abel object\n[S]ave templates\n[M]anual Mode On/Off\n[P]rint "
			"File\n";

	// Spin
	ros::spin();

	cv::destroyAllWindows();
}