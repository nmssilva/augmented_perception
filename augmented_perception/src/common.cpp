
#include "mtt/mtt.h"
#include "mtt/mtt_clustering.h"

void PointCloud2ToData(sensor_msgs::PointCloud2 &cloud, t_data &data)  // this function will convert the point cloud
// data into a laser scan type structure
{
	pcl::PointCloud<pcl::PointXYZ> PclCloud;
	pcl::PCLPointCloud2 pcl_pc;
	pcl_conversions::toPCL(cloud, pcl_pc);
	pcl::fromPCLPointCloud2(pcl_pc, PclCloud);

	double theta;
	map<double, pair<uint, double> > grid;
	map<double, pair<uint, double> >::iterator
			it;

	double spacing = 1. * M_PI / 180.;

	double rightBorder;
	double leftBorder;

	double r;

	// cout << "mtt Input size:" << PclCloud.points.size() << endl;
	// get points into grid to reorder them
	for (uint i = 0; i < PclCloud.points.size(); i++) {
		theta = atan2(PclCloud.points[i].y, PclCloud.points[i].x);
		r = sqrt(pow(PclCloud.points[i].x, 2) + pow(PclCloud.points[i].y, 2));

		// get closest theta, given a predefined precision
		rightBorder = spacing * ceil(theta / spacing);
		leftBorder = rightBorder - spacing;

		if (fabs(rightBorder - theta) < fabs(leftBorder - theta))
			theta = rightBorder;
		else
			theta = leftBorder;

		if (grid.find(theta) != grid.end()) {
			if (r < grid[theta].second)  // using closest measurement
			{
				grid[theta].first = i;
				grid[theta].second = r;
			}
		} else {
			grid[theta].first = i;
			grid[theta].second = r;
		}
	}

	uint i = 0;
	for (it = grid.begin(); it != grid.end(); it++) {
		// the map auto orders the theta values
		data.x[i] = PclCloud.points[(*it).second.first].x;
		data.y[i] = PclCloud.points[(*it).second.first].y;
		data.r[i] = sqrt(pow(data.x[i], 2) + pow(data.y[i], 2));
		data.t[i] = atan2(data.y[i], data.x[i]);

		i++;
	}

	data.n_points = grid.size();
	// cout << "mtt Size of data:" << data.n_points << endl;
}

double ClusteringThreshold(double r1, double t1, double r2, double t2, t_config *config) {
	double min_dist;
	double Ax, Ay, Bx, By;

	if (r1 < r2) {
		Ax = r1 * cos(t1);
		Ay = r1 * sin(t1);

		Bx = r1 * cos(t1 + deg2rad(0.5));
		By = r1 * sin(t1 + deg2rad(0.5));
	} else {
		Ax = r2 * cos(t2 + deg2rad(0.5));
		Ay = r2 * sin(t2 + deg2rad(0.5));

		Bx = r2 * cos(t2);
		By = r2 * sin(t2);
	}

	min_dist = sqrt(pow(Ax - Bx, 2) + pow(Ay - By, 2));

	// 	printf("Cos beta %f\n",cos(config->beta));
	// 	printf("MD %f\n",min_dist);

	return config->C0 + min_dist / cos(config->beta);
}

bool clustering(t_data &data, vector<t_clustersPtr> &clustersPtr, t_config *config, t_flag *flags) {
	int i;
	double x, y, xold = 0, yold = 0;
	double dist, threshold;

	t_clustersPtr cluster(new t_cluster);

	clustersPtr.clear();

	cluster->id = clustersPtr.size();

	for (i = 0; i < data.n_points; i++) {
		x = data.x[i];
		y = data.y[i];

		if (i > 0) {
			dist = sqrt((x - xold) * (x - xold) + (y - yold) * (y - yold));  // compute distance
			threshold = ClusteringThreshold(data.r[i - 1], data.t[i - 1], data.r[i], data.t[i], config);

			if (dist > threshold) {
				cluster->enp = i - 1;                             // set the last cluster endpoint
				cluster->n_points = cluster->enp - cluster->stp;  // sets the number of points in the cluster
				cluster->partialy_occluded = false;
				clustersPtr.push_back(cluster);

				cluster.reset(new t_cluster);

				cluster->id = clustersPtr.size();
				cluster->stp = i;  // sets the new cluster start and end point
				cluster->lenght = 0;
			} else {
				cluster->lenght += dist;
				if (cluster->lenght > config->cluster_break_distance) {
					cluster->enp = i - 1;                             // set the last cluster endpoint
					cluster->n_points = cluster->enp - cluster->stp;  // sets the number of points in the cluster
					cluster->partialy_occluded = false;
					clustersPtr.push_back(cluster);

					cluster.reset(new t_cluster);

					cluster->id = clustersPtr.size();
					cluster->stp = i;  // sets the new cluster start and end point
					cluster->lenght = 0;
				}
			}

			if (i == (data.n_points - 1))  // last point
			{
				// in case all points are in the same cluster
				cluster->enp = i;
				cluster->n_points = cluster->enp - cluster->stp;  // sets the number of points in the cluster
			}
		} else  // first point
		{
			flags->fp_s = 0;  // negates the first point of scan flag
			cluster->stp = 0;
			cluster->enp = 0;
			cluster->lenght = 0;
		}

		xold = x;
		yold = y;
	}

	return true;
}

void calc_cluster_props(vector<t_clustersPtr> &clusters, t_data &data) {
	double rmin;
	int e;

	for (uint i = 0; i < clusters.size(); i++) {
		rmin = 1e12;
		clusters[i]->lenght = 0;
		for (e = clusters[i]->stp; e < clusters[i]->enp; e++) {
			if (e < clusters[i]->enp - 1)
				clusters[i]->lenght += point2point_distance(data.x[e], data.y[e], data.x[e + 1], data.y[e + 1]);

			if (data.r[e] < rmin)
				rmin = data.r[e];
		}

		clusters[i]->rmin = rmin;
		clusters[i]->tm = (data.t[clusters[i]->stp] + data.t[clusters[i]->enp]) / 2;
	}
}

double point2point_distance(double xi, double yi, double xf, double yf) {
	return sqrt((xi - xf) * (xi - xf) + (yi - yf) * (yi - yf));
}

double point2line_distance(double alpha, double ro, double x, double y) {
	return ro - x * cos(alpha) - y * sin(alpha);
}

bool
clusters2objects(vector<t_objectPtr> &objectsPtr, vector<t_clustersPtr> &clusters, t_data &data, t_config &config) {
	t_objectPtr object(new t_object);

	objectsPtr.clear();

	for (uint i = 0; i < clusters.size(); i++) {
		object->rmin = clusters[i]->rmin;
		object->tm = clusters[i]->tm;
		object->object_found = false;

		object->partialy_occluded = clusters[i]->partialy_occluded;

		recursive_line_fitting(object, *clusters[i], data, config);

		objectsPtr.push_back(object);
		object.reset(new t_object);
	}

	return true;
}

void recursive_IEPF(t_objectPtr &object, t_data &data, int start, int end, t_config &config) {
	/**This functions malloc a line to work with, each time it mallocs a line it increments the number of lines object
	 * data*/

	int i, index = 0;
	double mean_variance, max_variance, current_variance;

	t_linePtr line(new t_line);

	line->alpha = atan2(data.x[start] - data.x[end], data.y[end] - data.y[start]) + M_PI;
	line->ro = data.x[start] * cos(line->alpha) + data.y[start] * sin(line->alpha);
	line->xi = data.x[start];
	line->yi = data.y[start];
	line->xf = data.x[end];
	line->yf = data.y[end];

	mean_variance = 0;
	max_variance = 0;
	for (i = start; i < end; i++) {
		current_variance = pow(point2line_distance(line->alpha, line->ro, data.x[i], data.y[i]), 2);
		mean_variance += current_variance;

		if (current_variance > max_variance) {
			max_variance = current_variance;
			index = i;
		}
	}

	mean_variance /= end - start;
	mean_variance = sqrt(mean_variance);

	// 	if(object->lines.size()>20)
	// 		goto F2;

	if (mean_variance > config.max_mean_variance) {
		recursive_IEPF(object, data, start, index, config);
		recursive_IEPF(object, data, index, end, config);
		return;
	}

	// 	F2:

	object->lines.push_back(line);
	// 	object->line[object->lines.size()]=(t_line*)malloc(sizeof(t_line));
	// 	memcpy(object->line[object->lines.size()],&line,sizeof(t_line));
	// 	object->n_lines++;

	return;
}

void recursive_line_fitting(t_objectPtr &object, t_cluster &cluster, t_data &data, t_config &config) {
	if (!data.n_points)
		return;

	recursive_IEPF(object, data, cluster.stp, cluster.enp, config);
}

void calc_object_props(vector<t_objectPtr> &objects) {
	uint e;
	double r, t;
	double xi, yi, xf, yf;
	double rmin;

	for (uint i = 0; i < objects.size(); i++) {
		t = objects[i]->tm;

		rmin = 1e12;

		for (e = 0; e < objects[i]->lines.size(); e++) {
			r = sqrt(pow(objects[i]->lines[e]->xi, 2) + pow(objects[i]->lines[e]->yi, 2));

			if (r < rmin)
				rmin = r;
		}

		r = sqrt(pow(objects[i]->lines[objects[i]->lines.size() - 1]->xf, 2) +
				 pow(objects[i]->lines[objects[i]->lines.size() - 1]->yf, 2));
		if (r < rmin)
			rmin = r;

		r = rmin;

		objects[i]->cx = r * cos(t);
		objects[i]->cy = r * sin(t);

		xi = objects[i]->lines[0]->xi;
		yi = objects[i]->lines[0]->yi;

		xf = objects[i]->lines[objects[i]->lines.size() - 1]->xf;
		yf = objects[i]->lines[objects[i]->lines.size() - 1]->yf;

		objects[i]->size = point2point_distance(xi, yi, xf, yf);
	}
}

void AssociateObjects(vector<t_listPtr> &list, vector<t_objectPtr> &objects, t_config &config, t_flag &flags) {
	for (uint i = 0; i < objects.size(); i++)
		objects[i]->object_found = false;

	double min_ret = 1;
	int min_index = -1;
	double ret = 1;
	bool association_found;
	double remove_threshold;

	/// Make the static objects association
	for (uint i = 0; i < list.size(); i++) {
		min_ret = 1;
		association_found = false;

		for (uint h = 0; h < objects.size(); h++)  /// Run throu all the new objects
		{
			ret = CheckAssociationCriteria(*list[i], *objects[h]);

			if (ret < min_ret && ret < 0)  /// Inside ellipse
			{
				min_ret = ret;
				min_index = h;
				association_found = true;
			}
		}

		// PFLN
		if (association_found) {
			// 			double lret;
			// 			double min_lret=1;
			double dist;
			double min_dist = 1e6;

			int index_e = -1;
			for (uint e = 0; e < list.size(); e++) {
				if (i == e)
					continue;

				dist = point2point_distance(objects[min_index]->cx, objects[min_index]->cy,
											list[e]->position.predicted_x,
											list[e]->position.predicted_y);

				if (dist < min_dist) {
					min_dist = dist;
					index_e = e;
				}
			}

			// Exclusion zone B
			// 			printf("%d %2.2f %2.2f\n",mlist->id,min_lret,min_ret);

			if (min_dist < config.ezB && list[index_e]->timers.lifetime > list[i]->timers.lifetime) {
				// 				printf("C%d A%d C%2.2f A%2.2f\n",mlist->id,aux2->id,min_ret,min_lret);
				association_found = false;
			}
		}

		// PFLN
		if (association_found && objects[min_index]->object_found == false)  /// Object found
		{
			if (list[i]->classification.partialy_occluded == false && objects[min_index]->partialy_occluded) {
				objects[min_index]->cx = (objects[min_index]->cx + list[i]->position.predicted_x) / 2;
				objects[min_index]->cy = (objects[min_index]->cy + list[i]->position.predicted_y) / 2;
			}
			// PFLN
			SingleObjectAssociation(*list[i], *objects[min_index]);
			objects[min_index]->object_found = true;
			list[i]->classification.occluded = false;
			list[i]->classification.partialy_occluded = objects[min_index]->partialy_occluded;

			// PFLN
		} else  /// Object not found
		{
			// PFLN

			list[i]->classification.occluded = true;
			list[i]->timers.occludedtime++;

			list[i]->measurements.x = list[i]->position.predicted_x;
			list[i]->measurements.y = list[i]->position.predicted_y;

			remove_threshold = list[i]->timers.lifetime;

			if (remove_threshold > config.max_missing_iterations)
				remove_threshold = config.max_missing_iterations;

			if (list[i]->timers.occludedtime > remove_threshold)
				RemoveFromList(list, list[i]->id);
		}
	}

	/// Add not found objects to list
	double dist_to_object = 1e6;
	for (uint g = 0; g < objects.size(); g++) {
		/// Calc min_distance_to_existing_object
		objects[g]->min_distance_to_existing_object = 1e6;

		for (uint i = 0; i < list.size(); i++) {
			dist_to_object =
					point2point_distance(objects[g]->cx, objects[g]->cy, list[i]->measurements.x,
										 list[i]->measurements.y);

			if (dist_to_object < objects[g]->min_distance_to_existing_object)
				objects[g]->min_distance_to_existing_object = dist_to_object;
		}

		// Exclusion zone A
		if (objects[g]->min_distance_to_existing_object < config.ezA && flags.fi == false)
			continue;

		if (objects[g]->object_found == false)
			AddObjectToList(list, *objects[g], config);
	}

	return;
}

double CheckAssociationCriteria(t_list &list, t_object &object) {
	double ox = object.cx;
	double oy = object.cy;

	double angle = -list.search_area.angle;
	double s = list.search_area.ellipse_B;
	double r = list.search_area.ellipse_A;
	double M = cos(angle);
	double N = sin(angle);
	double c = list.search_area.center_x;
	double d = list.search_area.center_y;
	double tx = ox - c;
	double ty = oy - d;
	double A = (M * tx - N * ty) * (M * tx - N * ty);
	double B = (N * tx + M * ty) * (N * tx + M * ty);
	double Z = s * s * A + r * r * B - r * r * s * s;

	if (Z < 0)
		return Z;
	else
		return 1;
}

void SingleObjectAssociation(t_list &list, t_object &object) {
	list.measurements.x = object.cx;
	list.measurements.y = object.cy;
	list.shape = object;

	SetOjectMorphology(list, object);
	object.object_found = true;
	object.id = list.id;

	list.timers.lifetime++;

	list.timers.occludedtime -= 1;
	if (list.timers.occludedtime < 0)
		list.timers.occludedtime = 0;
}

void RemoveFromList(vector<t_listPtr> &list, unsigned int id) {
	vector<t_listPtr>::iterator it;

	for (it = list.begin(); it != list.end(); it++) {
		if ((*it)->id == id) {
			/// Free kalman motion models
			cvReleaseKalman(&((*it)->motion_models.cv_x_kalman));
			cvReleaseKalman(&((*it)->motion_models.cv_y_kalman));
			cvReleaseKalman(&((*it)->motion_models.ca_x_kalman));
			cvReleaseKalman(&((*it)->motion_models.ca_y_kalman));

			free((*it)->errors_cv.x_innovation);
			free((*it)->errors_cv.x_residue);
			free((*it)->errors_cv.y_innovation);
			free((*it)->errors_cv.y_residue);
			free((*it)->errors_cv.lateral_error);
			// PFLN
			free((*it)->errors_ca.x_innovation);
			free((*it)->errors_ca.x_residue);
			free((*it)->errors_ca.y_innovation);
			free((*it)->errors_ca.y_residue);
			free((*it)->errors_ca.lateral_error);

			/// Free error vectors
			// PFLN
			/// Free path
			free((*it)->path_cv.x);
			free((*it)->path_cv.y);
			// PFLN
			/// Free path
			free((*it)->path_ca.x);
			free((*it)->path_ca.y);

			list.erase(it);
			return;
		}
	}
}

unsigned int last_id = 0;

void AddObjectToList(vector<t_listPtr> &list, t_object &object, t_config &config) {
	t_listPtr element(new t_list);

	AllocMotionModels(*element, config);

	AllocPath(&(element->path_cv), config);
	AllocPath(&(element->path_ca), config);

	AllocErrors(&(element->errors_cv), config);
	AllocErrors(&(element->errors_ca), config);

	element->measurements.x = object.cx;
	element->measurements.y = object.cy;
	element->position.estimated_x = object.cx;
	element->position.estimated_y = object.cy;

	InitialiseSearchArea(*element, config);
	InitialiseTimers(&(element->timers));
	InitialiseClassification(&(element->classification));

	element->object_morphology.apparent_size = object.size;

	element->model = CV;
	element->shape = object;

	element->id = last_id;

	last_id++;

	list.push_back(element);

	return;
}

void AllocMotionModels(t_list &list, t_config &config) {
	list.motion_models.cv_x_kalman = cvCreateKalman(2, 1, 0);
	list.motion_models.cv_y_kalman = cvCreateKalman(2, 1, 0);

	list.motion_models.ca_x_kalman = cvCreateKalman(3, 1, 0);
	list.motion_models.ca_y_kalman = cvCreateKalman(3, 1, 0);

	float a = 20;
	float dt = config.dt;

	float Acv[] = {1, (float) dt, 0, 1};

	float dt_2 = dt * dt;
	float Aca[] = {1, dt, (float) (0.5) * dt_2, 0, 1, dt, 0, 0, 1.0};

	double Qcv[] = {a * a * pow(dt, 3) / 3, a * a * pow(dt, 2) / 2, a * a * pow(dt, 2) / 2, a * a * pow(dt, 1)};

	double Qca[] = {pow(a, 2) * pow(dt, 5) / 20, pow(a, 2) * pow(dt, 4) / 8, pow(a, 2) * pow(dt, 3) / 6,
					pow(a, 2) * pow(dt, 4) / 8, pow(a, 2) * pow(dt, 3) / 3, pow(a, 2) * pow(dt, 2) / 2,
					pow(a, 2) * pow(dt, 3) / 6, pow(a, 2) * pow(dt, 2) / 2, pow(a, 2) * pow(dt, 1) / 1};

	memcpy(list.motion_models.cv_x_kalman->transition_matrix->data.fl, Acv, sizeof(Acv));
	memcpy(list.motion_models.cv_y_kalman->transition_matrix->data.fl, Acv, sizeof(Acv));

	memcpy(list.motion_models.ca_x_kalman->transition_matrix->data.fl, Aca, sizeof(Aca));
	memcpy(list.motion_models.ca_y_kalman->transition_matrix->data.fl, Aca, sizeof(Aca));

	cvSetIdentity(list.motion_models.cv_x_kalman->measurement_matrix, cvRealScalar(1));
	cvSetIdentity(list.motion_models.cv_y_kalman->measurement_matrix, cvRealScalar(1));

	cvSetIdentity(list.motion_models.ca_x_kalman->measurement_matrix, cvRealScalar(1));
	cvSetIdentity(list.motion_models.ca_y_kalman->measurement_matrix, cvRealScalar(1));

	cvSetIdentity(list.motion_models.cv_x_kalman->measurement_noise_cov, cvRealScalar(0.04 * 0.04));
	cvSetIdentity(list.motion_models.cv_x_kalman->error_cov_post, cvRealScalar(1));

	cvSetIdentity(list.motion_models.ca_x_kalman->measurement_noise_cov, cvRealScalar(0.04 * 0.04));
	cvSetIdentity(list.motion_models.ca_x_kalman->error_cov_post, cvRealScalar(1));

	cvSetIdentity(list.motion_models.cv_y_kalman->measurement_noise_cov, cvRealScalar(0.04 * 0.04));
	cvSetIdentity(list.motion_models.cv_y_kalman->error_cov_post, cvRealScalar(1));

	cvSetIdentity(list.motion_models.ca_y_kalman->measurement_noise_cov, cvRealScalar(0.04 * 0.04));
	cvSetIdentity(list.motion_models.ca_y_kalman->error_cov_post, cvRealScalar(1));

	cvSetReal2D(list.motion_models.cv_x_kalman->process_noise_cov, 0, 0, Qcv[0]);
	cvSetReal2D(list.motion_models.cv_x_kalman->process_noise_cov, 0, 1, Qcv[1]);
	cvSetReal2D(list.motion_models.cv_x_kalman->process_noise_cov, 1, 0, Qcv[2]);
	cvSetReal2D(list.motion_models.cv_x_kalman->process_noise_cov, 1, 1, Qcv[3]);

	cvSetReal2D(list.motion_models.cv_y_kalman->process_noise_cov, 0, 0, Qcv[0]);
	cvSetReal2D(list.motion_models.cv_y_kalman->process_noise_cov, 0, 1, Qcv[1]);
	cvSetReal2D(list.motion_models.cv_y_kalman->process_noise_cov, 1, 0, Qcv[2]);
	cvSetReal2D(list.motion_models.cv_y_kalman->process_noise_cov, 1, 1, Qcv[3]);

	cvSetReal2D(list.motion_models.ca_x_kalman->process_noise_cov, 0, 0, Qca[0]);
	cvSetReal2D(list.motion_models.ca_x_kalman->process_noise_cov, 0, 1, Qca[1]);
	cvSetReal2D(list.motion_models.ca_x_kalman->process_noise_cov, 0, 2, Qca[2]);
	cvSetReal2D(list.motion_models.ca_x_kalman->process_noise_cov, 1, 0, Qca[3]);
	cvSetReal2D(list.motion_models.ca_x_kalman->process_noise_cov, 1, 1, Qca[4]);
	cvSetReal2D(list.motion_models.ca_x_kalman->process_noise_cov, 1, 2, Qca[5]);
	cvSetReal2D(list.motion_models.ca_x_kalman->process_noise_cov, 2, 0, Qca[6]);
	cvSetReal2D(list.motion_models.ca_x_kalman->process_noise_cov, 2, 1, Qca[7]);
	cvSetReal2D(list.motion_models.ca_x_kalman->process_noise_cov, 2, 2, Qca[8]);

	cvSetReal2D(list.motion_models.ca_y_kalman->process_noise_cov, 0, 0, Qca[0]);
	cvSetReal2D(list.motion_models.ca_y_kalman->process_noise_cov, 0, 1, Qca[1]);
	cvSetReal2D(list.motion_models.ca_y_kalman->process_noise_cov, 0, 2, Qca[2]);
	cvSetReal2D(list.motion_models.ca_y_kalman->process_noise_cov, 1, 0, Qca[3]);
	cvSetReal2D(list.motion_models.ca_y_kalman->process_noise_cov, 1, 1, Qca[4]);
	cvSetReal2D(list.motion_models.ca_y_kalman->process_noise_cov, 1, 2, Qca[5]);
	cvSetReal2D(list.motion_models.ca_y_kalman->process_noise_cov, 2, 0, Qca[6]);
	cvSetReal2D(list.motion_models.ca_y_kalman->process_noise_cov, 2, 1, Qca[7]);
	cvSetReal2D(list.motion_models.ca_y_kalman->process_noise_cov, 2, 2, Qca[8]);

	cvZero(list.motion_models.cv_x_kalman->state_post);
	cvZero(list.motion_models.cv_y_kalman->state_post);

	cvZero(list.motion_models.ca_x_kalman->state_post);
	cvZero(list.motion_models.ca_y_kalman->state_post);

	return;
}

void AllocPath(t_path *path, t_config &config) {
	path->x = (double *) malloc(config.path_lenght * sizeof(double));

	path->y = (double *) malloc(config.path_lenght * sizeof(double));

	path->max_number_points = config.path_lenght;
	path->number_points = 0;
	path->position = 0;
	path->latest = 0;
	path->next = 1;

	return;
}

void AllocErrors(t_errors *error, t_config &config) {
	error->x_innovation = (double *) malloc(config.estimation_window_size * sizeof(double));

	error->x_residue = (double *) malloc(config.estimation_window_size * sizeof(double));

	error->y_innovation = (double *) malloc(config.estimation_window_size * sizeof(double));

	error->y_residue = (double *) malloc(config.estimation_window_size * sizeof(double));

	error->lateral_error = (double *) malloc(config.estimation_window_size * sizeof(double));

	error->x_inno_cov = 0;
	error->x_resi_cov = 0;
	error->y_inno_cov = 0;
	error->y_resi_cov = 0;
	error->lateral_error_cov = 0;

	error->max_number_points = config.estimation_window_size;
	error->number_points = 0;
	error->position = 0;
	error->latest = 0;
	error->next = 1;
}

void InitialiseSearchArea(t_list &list, t_config &config) {
	list.search_area.ellipse_A = config.max_ellipse_axis;
	list.search_area.ellipse_B = config.max_ellipse_axis;
	list.search_area.angle = 0;
	list.search_area.center_x = list.position.estimated_x;
	list.search_area.center_y = list.position.estimated_y;
}

void SetOjectMorphology(t_list &list, t_object &object) {
	list.object_morphology.apparent_size = object.size;
}

void InitialiseTimers(t_timers *timer) {
	timer->occludedtime = 0;
	timer->lifetime = 0;
}

void InitialiseClassification(t_classification *classification) {
	classification->velocity_classification = MOVING;
	classification->occluded = false;
	classification->partialy_occluded = false;
}

int select_object;

void AddPointErrorVectors(t_errors *error, double x_inno, double y_inno, double x_resi, double y_resi,
						  double lateral_error) {
	error->x_innovation[error->position] = x_inno;
	error->y_innovation[error->position] = y_inno;

	error->x_residue[error->position] = x_resi;
	error->y_residue[error->position] = y_resi;

	error->lateral_error[error->position] = lateral_error;

	error->latest = error->position;
	error->position++;

	GetErrorConvariance(error);

	if (error->position == error->max_number_points)
		error->position = 0;

	error->next = error->position;

	if (error->number_points < error->max_number_points)
		error->number_points++;
}

void MotionModelsIteration(vector<t_listPtr> &list, t_config &config) {
	CvMat *x_measurement = cvCreateMat(1, 1, CV_32FC1);
	CvMat *y_measurement = cvCreateMat(1, 1, CV_32FC1);

	double x_estimated_last = 0, y_estimated_last = 0;

	static bool initialise = true;
	static FILE *fp;
	if (initialise) {
		fp = fopen("data", "w");
		if (!fp) {
			perror("Open data");
			printf("Cannot save objects data to disk\nPlease do not chose an object it will cause a segmentation fault\n");
		}

		initialise = false;
	}

	for (uint i = 0; i < list.size(); i++) {
		/// If the object is new, we set the prestate and poststate, given that this is not a new measurement, of the filter
		/// to the current measurement
		if (list[i]->timers.lifetime == 0) {
			cvSetReal2D(list[i]->motion_models.cv_x_kalman->state_pre, 0, 0, list[i]->measurements.x);
			cvSetReal2D(list[i]->motion_models.cv_y_kalman->state_pre, 0, 0, list[i]->measurements.y);

			cvSetReal2D(list[i]->motion_models.ca_x_kalman->state_pre, 0, 0, list[i]->measurements.x);
			cvSetReal2D(list[i]->motion_models.ca_y_kalman->state_pre, 0, 0, list[i]->measurements.y);

			cvSetReal2D(list[i]->motion_models.cv_x_kalman->state_post, 0, 0, list[i]->measurements.x);
			cvSetReal2D(list[i]->motion_models.cv_y_kalman->state_post, 0, 0, list[i]->measurements.y);

			cvSetReal2D(list[i]->motion_models.ca_x_kalman->state_post, 0, 0, list[i]->measurements.x);
			cvSetReal2D(list[i]->motion_models.ca_y_kalman->state_post, 0, 0, list[i]->measurements.y);
		} else {
			/// Get the new measurement into the filter and correct, t=k

			cvSetReal2D(x_measurement, 0, 0, list[i]->measurements.x);
			cvSetReal2D(y_measurement, 0, 0, list[i]->measurements.y);

			x_estimated_last = list[i]->position.estimated_x;
			y_estimated_last = list[i]->position.estimated_y;

			cvKalmanCorrect(list[i]->motion_models.cv_x_kalman, x_measurement);
			cvKalmanCorrect(list[i]->motion_models.cv_y_kalman, y_measurement);

			cvKalmanCorrect(list[i]->motion_models.ca_x_kalman, x_measurement);
			cvKalmanCorrect(list[i]->motion_models.ca_y_kalman, y_measurement);
		}

		if ((int) list[i]->id == select_object)
			fprintf(fp, "%2.6f %2.6f ", list[i]->measurements.x, list[i]->measurements.y);

		/// Extract the correct state from the filter and put it to path[k]

		double cv_pestimated_x = cvGetReal2D(list[i]->motion_models.cv_x_kalman->state_post, 0, 0);
		double cv_pestimated_y = cvGetReal2D(list[i]->motion_models.cv_y_kalman->state_post, 0, 0);

		double ca_pestimated_x = cvGetReal2D(list[i]->motion_models.ca_x_kalman->state_post, 0, 0);
		double ca_pestimated_y = cvGetReal2D(list[i]->motion_models.ca_y_kalman->state_post, 0, 0);

		if ((int) list[i]->id == select_object)
			fprintf(fp, "%2.6f %2.6f %2.6f %2.6f ", cv_pestimated_x, cv_pestimated_y, ca_pestimated_x, ca_pestimated_y);

		/// After correction iterate the filter to make a prediction, t=k+1
		cvKalmanPredict(list[i]->motion_models.cv_x_kalman);
		cvKalmanPredict(list[i]->motion_models.cv_y_kalman);

		cvKalmanPredict(list[i]->motion_models.ca_x_kalman);
		cvKalmanPredict(list[i]->motion_models.ca_y_kalman);

		/// Extract the prediction from the filter

		double cv_ppredicted_x = cvGetReal2D(list[i]->motion_models.cv_x_kalman->state_pre, 0, 0);
		double cv_ppredicted_y = cvGetReal2D(list[i]->motion_models.cv_y_kalman->state_pre, 0, 0);

		double ca_ppredicted_x = cvGetReal2D(list[i]->motion_models.ca_x_kalman->state_pre, 0, 0);
		double ca_ppredicted_y = cvGetReal2D(list[i]->motion_models.ca_y_kalman->state_pre, 0, 0);

		if ((int) list[i]->id == select_object)
			fprintf(fp, "%2.6f %2.6f %2.6f %2.6f ", cv_ppredicted_x, cv_ppredicted_y, ca_ppredicted_x, ca_ppredicted_y);

		// 		CvPoint a=cvPoint(real2print(x_ca_pre,config),real2print(y_ca_pre,config));

		// 		cvLine(img,a,a,CV_RGB(0,255,0),2, 8, 0 );

		/// Obtain error vectors

		double cv_inno_x = list[i]->measurements.x - cv_ppredicted_x;
		double cv_inno_y = list[i]->measurements.y - cv_ppredicted_y;

		double ca_inno_x = list[i]->measurements.x - ca_ppredicted_x;
		double ca_inno_y = list[i]->measurements.y - ca_ppredicted_y;

		double cv_resi_x = list[i]->measurements.x - cv_pestimated_x;
		double cv_resi_y = list[i]->measurements.y - cv_pestimated_y;

		double ca_resi_x = list[i]->measurements.x - ca_pestimated_x;
		double ca_resi_y = list[i]->measurements.y - ca_pestimated_y;

		// 		double scvi=sqrt(pow(cv_inno_x,2)+pow(cv_inno_y,2));
		// 		double scai=sqrt(pow(ca_inno_x,2)+pow(ca_inno_y,2));

		if ((int) list[i]->id == select_object)
			fprintf(fp, "%2.6f %2.6f %2.6f %2.6f ", cv_inno_x, cv_inno_y, ca_inno_x, ca_inno_y);

		if ((int) list[i]->id == select_object)
			fprintf(fp, "%2.6f %2.6f %2.6f %2.6f ", cv_resi_x, cv_resi_y, ca_resi_x, ca_resi_y);

		// 		if(scvi<scai)
		// 			list[i]->model=CV;
		// 		else

		// 		list[i]->model=CA;
		// 		if(list[i]->timers.lifetime<30)
		// 			list[i]->model=CV;
		// //
		// 			list[i]->model=MIX;

		// 		if(list[i]->classification.occluded)
		// 			list[i]->model=CV;

		// 		list[i]->model=CA;
		list[i]->model = CV;

		switch (list[i]->model) {
			case CV:
				list[i]->position.estimated_x = cv_pestimated_x;
				list[i]->position.estimated_y = cv_pestimated_y;
				list[i]->position.predicted_x = cv_ppredicted_x;
				list[i]->position.predicted_y = cv_ppredicted_y;
				break;
			case CA:
				list[i]->position.estimated_x = ca_pestimated_x;
				list[i]->position.estimated_y = ca_pestimated_y;
				list[i]->position.predicted_x = ca_ppredicted_x;
				list[i]->position.predicted_y = ca_ppredicted_y;
				break;
			case MIX:
				list[i]->position.estimated_x = (ca_pestimated_x + cv_pestimated_x) / 2;
				list[i]->position.estimated_y = (ca_pestimated_y + cv_pestimated_y) / 2;
				list[i]->position.predicted_x = (ca_ppredicted_x + cv_ppredicted_x) / 2;
				list[i]->position.predicted_y = (ca_ppredicted_y + cv_ppredicted_y) / 2;
				break;
		}

		/// Add estimated point to path
		AddPointPath(&(list[i]->path_cv), list[i]->position.estimated_x, list[i]->position.estimated_y);
		AddPointPath(&(list[i]->path_ca), list[i]->position.estimated_x, list[i]->position.estimated_y);

		/// Put velocity into data file
		if ((int) list[i]->id == select_object) {
			double cvvx, cvvy, cavx, cavy;
			cvvx = cvGetReal2D(list[i]->motion_models.cv_x_kalman->state_post, 1, 0);
			cvvy = cvGetReal2D(list[i]->motion_models.cv_y_kalman->state_post, 1, 0);
			cavx = cvGetReal2D(list[i]->motion_models.ca_x_kalman->state_post, 1, 0);
			cavy = cvGetReal2D(list[i]->motion_models.ca_y_kalman->state_post, 1, 0);

			fprintf(fp, "%2.6f %2.6f %2.6f %2.6f ", cvvx, cvvy, cavx, cavy);
		}

		/// Obtain velocity

		double lateral_error;

		switch (list[i]->model) {
			case CV:
				list[i]->velocity.velocity_x = cvGetReal2D(list[i]->motion_models.cv_x_kalman->state_post, 1, 0);
				list[i]->velocity.velocity_y = cvGetReal2D(list[i]->motion_models.cv_y_kalman->state_post, 1, 0);
				break;
			case CA:
				list[i]->velocity.velocity_x = cvGetReal2D(list[i]->motion_models.ca_x_kalman->state_post, 1, 0);
				list[i]->velocity.velocity_y = cvGetReal2D(list[i]->motion_models.ca_y_kalman->state_post, 1, 0);
				break;
			default:
				list[i]->velocity.velocity_x = cvGetReal2D(list[i]->motion_models.cv_x_kalman->state_post, 1, 0);
				list[i]->velocity.velocity_y = cvGetReal2D(list[i]->motion_models.cv_y_kalman->state_post, 1, 0);
				break;
		}

		list[i]->velocity.velocity_module =
				sqrt(pow(list[i]->velocity.velocity_x, 2) + pow(list[i]->velocity.velocity_y, 2));
		list[i]->velocity.velocity_angle = atan2(-list[i]->velocity.velocity_y, list[i]->velocity.velocity_x);

		double xi, xf, yi, yf;

		xi = x_estimated_last;
		yi = y_estimated_last;
		xf = xi + list[i]->velocity.velocity_x;
		yf = yi + list[i]->velocity.velocity_y;

		double alpha = atan2(xi - xf, yf - yi) + M_PI;
		double ro = xi * cos(alpha) + yi * sin(alpha);

		double x_m = cvGetReal2D(x_measurement, 0, 0);
		double y_m = cvGetReal2D(y_measurement, 0, 0);

		lateral_error = point2line_distance(alpha, ro, x_m, y_m);

		AddPointErrorVectors(&(list[i]->errors_cv), cv_inno_x, cv_inno_y, cv_resi_x, cv_resi_y, lateral_error);
		AddPointErrorVectors(&(list[i]->errors_ca), ca_inno_x, ca_inno_y, ca_resi_x, ca_resi_y, lateral_error);

		/// Put lateral error into data file
		if ((int) list[i]->id == select_object)
			fprintf(fp, "%2.6f ", lateral_error);

		/// Put error cov and AKF gains into data file
		if ((int) list[i]->id == select_object) {
			double cvcovx, cvcovy, cacovx, cacovy;
			cvcovx = list[i]->errors_cv.x_inno_cov;
			cvcovy = list[i]->errors_cv.y_inno_cov;
			cacovx = list[i]->errors_ca.x_inno_cov;
			cacovy = list[i]->errors_ca.y_inno_cov;

			fprintf(fp, "%2.6f %2.6f %2.6f %2.6f ", cvcovx, cvcovy, cacovx, cacovy);

			double cvgpx, cvgpy, cvgvx, cvgvy, cagpx, cagpy, cagvx, cagvy;
			cvgpx = cvGetReal2D(list[i]->motion_models.cv_x_kalman->gain, 0, 0);
			cvgpy = cvGetReal2D(list[i]->motion_models.cv_y_kalman->gain, 0, 0);
			cvgvx = cvGetReal2D(list[i]->motion_models.cv_x_kalman->gain, 1, 0);
			cvgvy = cvGetReal2D(list[i]->motion_models.cv_y_kalman->gain, 1, 0);

			cagpx = cvGetReal2D(list[i]->motion_models.ca_x_kalman->gain, 0, 0);
			cagpy = cvGetReal2D(list[i]->motion_models.ca_y_kalman->gain, 0, 0);
			cagvx = cvGetReal2D(list[i]->motion_models.ca_x_kalman->gain, 1, 0);
			cagvy = cvGetReal2D(list[i]->motion_models.ca_y_kalman->gain, 1, 0);

			fprintf(fp, "%2.6f %2.6f %2.6f %2.6f %2.6f %2.6f %2.6f %2.6f ", cvgpx, cvgpy, cvgvx, cvgvy, cagpx, cagpy,
					cagvx,
					cagvy);
		}

		double previous_factor = 1;

		if (list[i]->classification.velocity_classification == STATIONARY)
			previous_factor = 2;
		else if (list[i]->classification.velocity_classification == MOVING)
			previous_factor = 1. / 2.;

		if (list[i]->velocity.velocity_module < config.max_stationary_velocity * previous_factor) {
			list[i]->classification.velocity_classification = STATIONARY;
			// 			printf("Id %d Stationary\n",objects[i]->id);
		}
		if (list[i]->velocity.velocity_module > config.min_moving_velocity * previous_factor ||
			list[i]->timers.lifetime < 5) {
			list[i]->classification.velocity_classification = MOVING;
			// 			printf("Id %d Moving\n",objects[i]->id);
		}

		/// Define a new search area based on the kalman errors and predicted position

		SetSearchArea(*list[i], config);

		/// Do some tweaks on the kalman errors

		if (list[i]->classification.velocity_classification == STATIONARY) {
			cvSetIdentity(list[i]->motion_models.cv_x_kalman->measurement_noise_cov, cvRealScalar(0.1 * 0.1));
			cvSetIdentity(list[i]->motion_models.cv_y_kalman->measurement_noise_cov, cvRealScalar(0.1 * 0.1));

			cvSetIdentity(list[i]->motion_models.ca_x_kalman->measurement_noise_cov, cvRealScalar(0.1 * 0.1));
			cvSetIdentity(list[i]->motion_models.ca_y_kalman->measurement_noise_cov, cvRealScalar(0.1 * 0.1));
		} else {
			cvSetIdentity(list[i]->motion_models.cv_x_kalman->measurement_noise_cov, cvRealScalar(0.05 * 0.05));
			cvSetIdentity(list[i]->motion_models.cv_y_kalman->measurement_noise_cov, cvRealScalar(0.05 * 0.05));

			cvSetIdentity(list[i]->motion_models.ca_x_kalman->measurement_noise_cov, cvRealScalar(0.05 * 0.05));
			cvSetIdentity(list[i]->motion_models.ca_y_kalman->measurement_noise_cov, cvRealScalar(0.05 * 0.05));
		}

		///* Update white noise scale **************************/
		double xAcv = sqrt(list[i]->errors_cv.x_inno_cov);
		double yAcv = sqrt(list[i]->errors_cv.y_inno_cov);
		double xAca = sqrt(list[i]->errors_ca.x_inno_cov);
		double yAca = sqrt(list[i]->errors_ca.y_inno_cov);

		if (xAcv < 0.001)
			xAcv = 0.001;
		if (yAcv < 0.001)
			yAcv = 0.001;

		if (xAcv > 2)
			xAcv = 2;
		if (yAcv > 2)
			yAcv = 2;

		if (xAca < 0.001)
			xAca = 0.001;
		if (yAca < 0.001)
			yAca = 0.001;

		if (xAca > 2)
			xAca = 2;
		if (yAca > 2)
			yAca = 2;

		double dt = config.dt;
		// 			printf("Id %d xA %2.2f yA %2.2f\n",objects[i]->id,xA,yA);

		if (list[i]->classification.occluded == false)  /// Only update if the object is visible
		{
			double xQcv[] = {pow(xAcv, 2) * pow(dt, 3) / 3, pow(xAcv, 2) * pow(dt, 2) / 2,
							 pow(xAcv, 2) * pow(dt, 2) / 2,
							 pow(xAcv, 2) * pow(dt, 1)};
			double yQcv[] = {pow(yAcv, 2) * pow(dt, 3) / 3, pow(yAcv, 2) * pow(dt, 2) / 2,
							 pow(yAcv, 2) * pow(dt, 2) / 2,
							 pow(yAcv, 2) * pow(dt, 1)};

			double xQca[] = {pow(xAca, 2) * pow(dt, 5) / 20, pow(xAca, 2) * pow(dt, 4) / 8,
							 pow(xAca, 2) * pow(dt, 3) / 6,
							 pow(xAca, 2) * pow(dt, 4) / 8, pow(xAca, 2) * pow(dt, 3) / 3,
							 pow(xAca, 2) * pow(dt, 2) / 2,
							 pow(xAca, 2) * pow(dt, 3) / 6, pow(xAca, 2) * pow(dt, 2) / 2,
							 pow(xAca, 2) * pow(dt, 1) / 1};

			double yQca[] = {pow(yAca, 2) * pow(dt, 5) / 20, pow(yAca, 2) * pow(dt, 4) / 8,
							 pow(yAca, 2) * pow(dt, 3) / 6,
							 pow(yAca, 2) * pow(dt, 4) / 8, pow(yAca, 2) * pow(dt, 3) / 3,
							 pow(yAca, 2) * pow(dt, 2) / 2,
							 pow(yAca, 2) * pow(dt, 3) / 6, pow(yAca, 2) * pow(dt, 2) / 2,
							 pow(yAca, 2) * pow(dt, 1) / 1};

			cvSetReal2D(list[i]->motion_models.cv_x_kalman->process_noise_cov, 0, 0, xQcv[0]);
			cvSetReal2D(list[i]->motion_models.cv_x_kalman->process_noise_cov, 0, 1, xQcv[1]);
			cvSetReal2D(list[i]->motion_models.cv_x_kalman->process_noise_cov, 1, 0, xQcv[2]);
			cvSetReal2D(list[i]->motion_models.cv_x_kalman->process_noise_cov, 1, 1, xQcv[3]);

			cvSetReal2D(list[i]->motion_models.cv_y_kalman->process_noise_cov, 0, 0, yQcv[0]);
			cvSetReal2D(list[i]->motion_models.cv_y_kalman->process_noise_cov, 0, 1, yQcv[1]);
			cvSetReal2D(list[i]->motion_models.cv_y_kalman->process_noise_cov, 1, 0, yQcv[2]);
			cvSetReal2D(list[i]->motion_models.cv_y_kalman->process_noise_cov, 1, 1, yQcv[3]);

			cvSetReal2D(list[i]->motion_models.ca_x_kalman->process_noise_cov, 0, 0, xQca[0]);
			cvSetReal2D(list[i]->motion_models.ca_x_kalman->process_noise_cov, 0, 1, xQca[1]);
			cvSetReal2D(list[i]->motion_models.ca_x_kalman->process_noise_cov, 0, 2, xQca[2]);
			cvSetReal2D(list[i]->motion_models.ca_x_kalman->process_noise_cov, 1, 0, xQca[3]);
			cvSetReal2D(list[i]->motion_models.ca_x_kalman->process_noise_cov, 1, 1, xQca[4]);
			cvSetReal2D(list[i]->motion_models.ca_x_kalman->process_noise_cov, 1, 2, xQca[5]);
			cvSetReal2D(list[i]->motion_models.ca_x_kalman->process_noise_cov, 2, 0, xQca[6]);
			cvSetReal2D(list[i]->motion_models.ca_x_kalman->process_noise_cov, 2, 1, xQca[7]);
			cvSetReal2D(list[i]->motion_models.ca_x_kalman->process_noise_cov, 2, 2, xQca[8]);

			cvSetReal2D(list[i]->motion_models.ca_y_kalman->process_noise_cov, 0, 0, yQca[0]);
			cvSetReal2D(list[i]->motion_models.ca_y_kalman->process_noise_cov, 0, 1, yQca[1]);
			cvSetReal2D(list[i]->motion_models.ca_y_kalman->process_noise_cov, 0, 2, yQca[2]);
			cvSetReal2D(list[i]->motion_models.ca_y_kalman->process_noise_cov, 1, 0, yQca[3]);
			cvSetReal2D(list[i]->motion_models.ca_y_kalman->process_noise_cov, 1, 1, yQca[4]);
			cvSetReal2D(list[i]->motion_models.ca_y_kalman->process_noise_cov, 1, 2, yQca[5]);
			cvSetReal2D(list[i]->motion_models.ca_y_kalman->process_noise_cov, 2, 0, yQca[6]);
			cvSetReal2D(list[i]->motion_models.ca_y_kalman->process_noise_cov, 2, 1, yQca[7]);
			cvSetReal2D(list[i]->motion_models.ca_y_kalman->process_noise_cov, 2, 2, yQca[8]);
		}

		if ((int) list[i]->id == select_object)
			fprintf(fp, "\n");
	}

	cvReleaseMat(&(x_measurement));
	cvReleaseMat(&(y_measurement));
}

void AddPointPath(t_path *path, double x, double y) {
	path->x[path->position] = x;
	path->y[path->position] = y;

	path->latest = path->position;
	path->position++;

	if (path->position == path->max_number_points)
		path->position = 0;

	path->next = path->position;

	if (path->number_points < path->max_number_points)
		path->number_points++;
}

void SetSearchArea(t_list &list, t_config &config) {
	list.search_area.angle = list.velocity.velocity_angle;
	list.search_area.center_x = list.position.predicted_x;
	list.search_area.center_y = list.position.predicted_y;

	// 	printf("%d %p Xp %2.2f Yp %2.2f\n",list->id,list,list->search_area.center_x,list->search_area.center_y);

	// 	printf("%d EA %2.2f EB %2.2f\n",list->id,list->search_area.ellipse_A,list->search_area.ellipse_B);

	double size = list.object_morphology.apparent_size;
	double size_factor = 0.1;

	double default_size = config.min_ellipse_axis;

	double xIl, yIl;

	if (list.model == CV) {
		xIl = list.errors_cv.x_inno_cov;
		yIl = list.errors_cv.y_inno_cov;
	} else {
		xIl = list.errors_ca.x_inno_cov;
		yIl = list.errors_ca.y_inno_cov;
	}

	int not_found_counter = list.timers.occludedtime;
	double not_found_factor = 2;

	double innovation_error = sqrt(sqrt(xIl * xIl + yIl * yIl));
	double lateral_error = sqrt(list.errors_cv.lateral_error_cov);
	double A_innovation_factor, B_innovation_factor;
	double lateral_factor;

	if (list.velocity.velocity_module < 0.200) {
		A_innovation_factor = 5;
		B_innovation_factor = 4;
		lateral_factor = 1;
	} else {
		A_innovation_factor = 5;
		B_innovation_factor = 0.5;
		lateral_factor = 4;
	}

	if (innovation_error < 0.001)
		innovation_error = 0.001;

	list.search_area.ellipse_A = not_found_factor * pow(not_found_counter, 2) + A_innovation_factor * innovation_error +
								 default_size + size_factor * size;
	list.search_area.ellipse_B = not_found_factor * pow(not_found_counter, 2) + B_innovation_factor * innovation_error +
								 default_size + size_factor * size + lateral_error * lateral_factor;

	if (list.search_area.ellipse_A > config.max_ellipse_axis)
		list.search_area.ellipse_A = config.max_ellipse_axis + default_size + size_factor * size;

	if (list.search_area.ellipse_B > config.max_ellipse_axis)
		list.search_area.ellipse_B = config.max_ellipse_axis + default_size + size_factor * size;
}

void GetErrorConvariance(t_errors *error) {
	/// Initalise auxiliar matrixes

	CvMat *xi_error_element = cvCreateMat(2, 2, CV_64FC1);
	CvMat *xr_error_element = cvCreateMat(2, 2, CV_64FC1);
	CvMat *yi_error_element = cvCreateMat(2, 2, CV_64FC1);
	CvMat *yr_error_element = cvCreateMat(2, 2, CV_64FC1);
	CvMat *lateral_error_element = cvCreateMat(2, 2, CV_64FC1);

	CvMat *xi_element = cvCreateMat(2, 1, CV_64FC1);
	CvMat *xr_element = cvCreateMat(2, 1, CV_64FC1);
	CvMat *yi_element = cvCreateMat(2, 1, CV_64FC1);
	CvMat *yr_element = cvCreateMat(2, 1, CV_64FC1);
	CvMat *lateral_element = cvCreateMat(2, 1, CV_64FC1);

	CvMat *xi_element_t = cvCreateMat(1, 2, CV_64FC1);
	CvMat *xr_element_t = cvCreateMat(1, 2, CV_64FC1);
	CvMat *yi_element_t = cvCreateMat(1, 2, CV_64FC1);
	CvMat *yr_element_t = cvCreateMat(1, 2, CV_64FC1);
	CvMat *lateral_element_t = cvCreateMat(1, 2, CV_64FC1);

	CvMat *xi_error_accumulator = cvCreateMat(2, 2, CV_64FC1);
	CvMat *xr_error_accumulator = cvCreateMat(2, 2, CV_64FC1);
	CvMat *yi_error_accumulator = cvCreateMat(2, 2, CV_64FC1);
	CvMat *yr_error_accumulator = cvCreateMat(2, 2, CV_64FC1);
	CvMat *lateral_error_accumulator = cvCreateMat(2, 2, CV_64FC1);

	CvMat *ones = cvCreateMat(2, 2, CV_64FC1);

	double multiplier = 1;

	/// Set accumulators to 0

	cvSet(xi_error_accumulator, cvScalar(0), NULL);
	cvSet(xr_error_accumulator, cvScalar(0), NULL);
	cvSet(yi_error_accumulator, cvScalar(0), NULL);
	cvSet(yr_error_accumulator, cvScalar(0), NULL);
	cvSet(lateral_error_accumulator, cvScalar(0), NULL);

	/// Do Cov(Innovation) = 1/m * S(i=0,i< m-1,d[k-i]*d[k-i]') and Cov(Residue) = 1/m * S(i=0,i< m-1,e[k-i]*e[k-i]')
	for (unsigned int i = 0; i < error->number_points; i++) {
		// 		printf("x_cv_innovation %2.2f\n",error->x_cv_innovation[i]);
		cvmSet(xi_element, 0, 0, error->x_innovation[i]);
		cvmSet(xi_element, 1, 0, 0);

		cvmSet(xr_element, 0, 0, error->x_residue[i]);
		cvmSet(xr_element, 1, 0, 0);

		cvmSet(yi_element, 0, 0, error->y_innovation[i]);
		cvmSet(yi_element, 1, 0, 0);

		cvmSet(yr_element, 0, 0, error->y_residue[i]);
		cvmSet(yr_element, 1, 0, 0);

		cvmSet(lateral_element, 0, 0, error->lateral_error[i]);
		cvmSet(lateral_element, 1, 0, 0);

		cvTranspose(xi_element, xi_element_t);
		cvTranspose(xr_element, xr_element_t);
		cvTranspose(yi_element, yi_element_t);
		cvTranspose(yr_element, yr_element_t);
		cvTranspose(lateral_element, lateral_element_t);

		cvMatMul(xi_element, xi_element_t, xi_error_element);
		cvMatMul(xr_element, xr_element_t, xr_error_element);
		cvMatMul(yi_element, yi_element_t, yi_error_element);
		cvMatMul(yr_element, yr_element_t, yr_error_element);
		cvMatMul(lateral_element, lateral_element_t, lateral_error_element);

		cvAdd(xi_error_element, xi_error_accumulator, xi_error_accumulator, NULL);
		cvAdd(xr_error_element, xr_error_accumulator, xr_error_accumulator, NULL);
		cvAdd(yi_error_element, yi_error_accumulator, yi_error_accumulator, NULL);
		cvAdd(yr_error_element, yr_error_accumulator, yr_error_accumulator, NULL);
		cvAdd(lateral_error_element, lateral_error_accumulator, lateral_error_accumulator, NULL);
	}

	if (error->number_points == 0)
		goto gecEND;

	cvSet(ones, cvScalar(1));
	multiplier = 1. / (double) error->number_points;
	// printf("Muli %2.8f\n",multiplier);
	///@todo Wend the number of points is growing the covariance is way to big, i don't realy know why

	cvMul(xi_error_accumulator, ones, xi_error_accumulator, multiplier);
	cvMul(xr_error_accumulator, ones, xr_error_accumulator, multiplier);
	cvMul(yi_error_accumulator, ones, yi_error_accumulator, multiplier);
	cvMul(yr_error_accumulator, ones, yr_error_accumulator, multiplier);
	cvMul(lateral_error_accumulator, ones, lateral_error_accumulator, multiplier);

	error->x_inno_cov = cvGetReal2D(xi_error_accumulator, 0, 0);
	error->x_resi_cov = cvGetReal2D(xr_error_accumulator, 0, 0);
	error->y_inno_cov = cvGetReal2D(yi_error_accumulator, 0, 0);
	error->y_resi_cov = cvGetReal2D(yr_error_accumulator, 0, 0);
	error->lateral_error_cov = cvGetReal2D(lateral_error_accumulator, 0, 0);

	gecEND:

	cvReleaseMat(&(xi_error_element));
	cvReleaseMat(&(xr_error_element));
	cvReleaseMat(&(yi_error_element));
	cvReleaseMat(&(yr_error_element));
	cvReleaseMat(&(lateral_error_element));

	cvReleaseMat(&(xi_element));
	cvReleaseMat(&(xr_element));
	cvReleaseMat(&(yi_element));
	cvReleaseMat(&(yr_element));
	cvReleaseMat(&(lateral_element));

	cvReleaseMat(&(xi_element_t));
	cvReleaseMat(&(xr_element_t));
	cvReleaseMat(&(yi_element_t));
	cvReleaseMat(&(yr_element_t));
	cvReleaseMat(&(lateral_element_t));

	cvReleaseMat(&(xi_error_accumulator));
	cvReleaseMat(&(xr_error_accumulator));
	cvReleaseMat(&(yi_error_accumulator));
	cvReleaseMat(&(yr_error_accumulator));
	cvReleaseMat(&(lateral_error_accumulator));
	cvReleaseMat(&(ones));

	return;
}

void free_lines(vector<t_objectPtr> &objects) {
	for (uint i = 0; i < objects.size(); i++)
		objects[i]->lines.clear();
}

void init_flags(t_flag *flags) {
	flags->fp_s = true;
	flags->fi = true;
}

void init_config(t_config *config) {
	config->maxr = 50.;  // in meters

	config->in_clip = 0.1;  // in meters
	config->out_clip = config->maxr;

	config->in_angle = 0;          // Radians
	config->out_angle = 2 * M_PI;  // Radians

	// 	config->cluster_break_distance=10;
	// 	config->cluster_break_distance=100;
	// 		config->cluster_break_distance=10.;
	// 	config->cluster_break_distance=100;
	config->cluster_break_distance = 10000;
	// 	config->cluster_break_distance=0.5;

	// 	config->fi=5.0*(M_PI/180.);//clustering parameters
	// 	config->fi=90*(M_PI/180.);//clustering parameters
	config->beta = 80 * (M_PI / 180.);
	// 		config->C0=200.;
	config->C0 = 0.5;
	// 	config->C0=0.2;

	config->filter_max_variation = 0.001;  // used in raw data filter, in m

	// 	config->excluding_dr=2.50;
	config->excluding_dr = 250;

	// 	config->point_occlusion_distance=500.; //used in point occlusion calculation
	config->point_occlusion_distance = 2000;  // used in point occlusion calculation
	// 	config->point_occlusion_distance=250; //used in point occlusion calculation

	config->max_line_per_object = 1000;  // used in lines
	config->max_mean_variance = 0.1;     // used in lines
	// 	config->max_mean_variance=5;//used in lines

	// 	config->data_acc_scans=20;
	config->data_acc_scans = 10;

	// 	config->max_missing_iterations=30;
	config->max_missing_iterations = 50;

	config->overlap_max = 5;
	config->overlap_distance = 0.2;

	config->display_min_lifetime = 10;

	// 	config->dt=1./13.;
	// 	config->dt=1./30.;
	// 	config->dt=1./40.;
	// 	config->dt=1./50.;
	config->dt = 1. / 50.;

	config->estimation_window_size = 2;  // estimation window size for all the error vectors

	config->max_stationary_velocity = 0.00030;
	config->min_moving_velocity = 0.000100;

	config->velocity_acc_size = 10;

	config->path_lenght = 1000;

	// 	config->default_ellipse_axis=0.500;
	// 	config->max_ellipse_axis=2.000;
	// 	config->min_ellipse_axis=0.050;

	config->default_ellipse_axis = 1.000;
	config->max_ellipse_axis = 2.500;
	config->min_ellipse_axis = 0.500;

	config->ezA = 2.0;
	config->ezB = 1.0;
}

double box_x = 0, box_y = 0, box_z = 0;
unsigned int box_id;
bool lost = true;

void CreateMarkers(vector<visualization_msgs::Marker> &marker_vector, mtt::TargetListPC &target_msg,
				   vector<t_listPtr> &list) {
	static map<pair<string, int>, pair<visualization_msgs::Marker, int> > marker_map;
	map<pair<string, int>, pair<visualization_msgs::Marker, int> >::iterator
			it;

	// limpar o vector todo
	marker_vector.clear();
	// colocar todos os elementos em processo de elemincação
	for (it = marker_map.begin(); it != marker_map.end(); it++)
		it->second.second--;

	std_msgs::ColorRGBA color;
	class_colormap colormap("hsv", 10, 1, false);

	visualization_msgs::Marker marker;

	marker.header.frame_id = target_msg.header.frame_id;
	// cout << "fid:" << target_msg.header.frame_id << endl;

	marker.header.stamp = ros::Time::now();
	marker.ns = "ids";
	marker.action = visualization_msgs::Marker::ADD;

	marker.pose.position.z = 0.3;

	marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

	marker.scale.x = 1;
	marker.scale.y = 1;
	marker.scale.z = 1;

	marker.color.r = 1;
	marker.color.g = 1;
	marker.color.b = 1;
	marker.color.a = 1;

	marker.id = 0;
	int distance = 3000;

	if (list.size() > 0) {
		for (uint i = 0; i < list.size(); i++) {
			if (list[i]->shape.lines.size() != 0) {
				if (sqrt(pow(list[i]->position.estimated_x, 2) + pow(list[i]->position.estimated_y, 2)) < distance) {
					distance = sqrt(pow(list[i]->position.estimated_x, 2) + pow(list[i]->position.estimated_y, 2));
					box_id = list[i]->id;
					box_x = list[i]->position.estimated_x;
					box_y = list[i]->position.estimated_y;
					box_z = 0.5;
				}
			}
		}
	}

	//cout << box_id << ":(" << box_x << ", " << box_y << ")\n";

	for (uint i = 0; i < list.size(); i++) {
		if (list[i]->shape.lines.size() != 0) {
			marker.pose.position.x = list[i]->position.estimated_x;
			marker.pose.position.y = list[i]->position.estimated_y;

			marker.text = boost::lexical_cast<string>(list[i]->id);

			marker.id++;

			marker_map[make_pair(marker.ns, marker.id)] = make_pair(marker, 1);
			// isto substitui ou cria o novo marker no maplist
		}
	}

	marker.pose.position.x = 0;
	marker.pose.position.y = 0;

	marker.text = "origin";

	marker.id++;

	// end of text markers
	// begin line objects

	marker_map[make_pair(marker.ns, marker.id)] = make_pair(marker, 1);  // isto substitui ou cria o novo marker no map

	// Markers for Line objects
	marker.type = visualization_msgs::Marker::LINE_STRIP;
	marker.ns = "objects";

	marker.pose.position.x = 0;
	marker.pose.position.y = 0;
	marker.pose.position.z = 0;

	marker.scale.x = 0.02;
	marker.scale.y = 0.1;
	marker.scale.z = 0.1;

	for (uint i = 0; i < list.size(); i++) {
		marker.color = colormap.color(list[i]->id);

		geometry_msgs::Point p;
		p.z = 0.1;

		marker.points.clear();

		uint l;
		for (l = 0; l < list[i]->shape.lines.size(); l++) {
			p.x = list[i]->shape.lines[l]->xi;
			p.y = list[i]->shape.lines[l]->yi;

			marker.points.push_back(p);
		}

		p.x = list[i]->shape.lines[l - 1]->xf;
		p.y = list[i]->shape.lines[l - 1]->yf;

		marker.points.push_back(p);

		marker.id++;

		// isto substitui ou cria o novo marker no map
		marker_map[make_pair(marker.ns, marker.id)] = make_pair(marker, 1);
	}

	// begin 3D BBox markers
	marker_map[make_pair(marker.ns, marker.id)] = make_pair(marker, 1);  // isto substitui ou cria o novo marker no map

	// Markers for Line objects
	marker.type = visualization_msgs::Marker::CUBE;
	marker.ns = "boxes";

	marker.pose.position.z = 0.5;

	marker.scale.x = 3;
	marker.scale.y = 2;
	marker.scale.z = 1;

	for (uint i = 0; i < list.size(); i++) {
		if (list[i]->shape.lines.size() != 0) {
			marker.pose.position.x = list[i]->position.estimated_x;
			marker.pose.position.y = list[i]->position.estimated_y;
			marker.color.a = 0.5;
			marker.color.r = 0;
			marker.color.g = 1;
			marker.color.b = 0;

			marker.id++;

			marker_map[make_pair(marker.ns, marker.id)] = make_pair(marker,
																	1);  // isto substitui ou cria o novo marker no
			// map
		}
	}

	// para o map todo envio tudo, e meto tudo a false
	// envio todo e tudo o que ainda estiver a false vai com operaração de delete
	for (it = marker_map.begin(); it != marker_map.end(); it++) {
		if (it->second.second == 0)  // se for falso é para apagar
			it->second.first.action = visualization_msgs::Marker::DELETE;
		else if (it->second.second <= -1)  // já foi apagado
		{
			/**
			\todo This should be erased but it is not working now
			marker_map.erase(it);
			*/
			continue;
		}

		marker_vector.push_back(it->second.first);
	}
}


double box_xSug = 0, box_ySug = 0, box_zSug = 0;
unsigned int box_idSug;
bool foundSug = false;
float distanceSug = 3000;
bool changeID = false;
unsigned int prevID = 0;
bool firstID = true;

void CreateMarkersSug(vector<visualization_msgs::Marker> &marker_vector, mtt::TargetListPC &target_msg,
				   vector<t_listPtr> &list) {
	static map<pair<string, int>, pair<visualization_msgs::Marker, int> > marker_map;
	map<pair<string, int>, pair<visualization_msgs::Marker, int> >::iterator
			it;

	// limpar o vector todo
	marker_vector.clear();
	// colocar todos os elementos em processo de elemincação
	for (it = marker_map.begin(); it != marker_map.end(); it++)
		it->second.second--;

	std_msgs::ColorRGBA color;
	class_colormap colormap("hsv", 10, 1, false);

	visualization_msgs::Marker marker;

	marker.header.frame_id = target_msg.header.frame_id;
	// cout << "fid:" << target_msg.header.frame_id << endl;

	marker.header.stamp = ros::Time::now();
	marker.ns = "ids";
	marker.action = visualization_msgs::Marker::ADD;

	marker.pose.position.z = 0.3;

	marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

	marker.scale.x = 1;
	marker.scale.y = 1;
	marker.scale.z = 1;

	marker.color.r = 1;
	marker.color.g = 1;
	marker.color.b = 1;
	marker.color.a = 1;

	marker.id = 0;

	distanceSug = 3000;
	if (list.size() > 0) {
		foundSug = true;
		for (uint i = 0; i < list.size(); i++) {
			if (list[i]->shape.lines.size() != 0) {
				if (sqrt(pow(list[i]->position.estimated_x, 2) + pow(list[i]->position.estimated_y, 2)) < distanceSug) {
					distanceSug = sqrt(pow(list[i]->position.estimated_x, 2) + pow(list[i]->position.estimated_y, 2));
					box_idSug = list[i]->id;
					if(firstID){
						prevID = box_idSug;
					}
					firstID = false;
					box_xSug = list[i]->position.estimated_x;
					box_ySug = list[i]->position.estimated_y;
					box_zSug = 0.5;
				}
			}
		}
		if(prevID != box_idSug){
			changeID = true;
		}
		else{
			changeID = false;
		}
		prevID = box_idSug;
	}else{
		foundSug = false;
		changeID = false;
	}

	//cout << box_id << ":(" << box_x << ", " << box_y << ")\n";

	for (uint i = 0; i < list.size(); i++) {
		if (list[i]->shape.lines.size() != 0) {
			marker.pose.position.x = list[i]->position.estimated_x;
			marker.pose.position.y = list[i]->position.estimated_y;

			marker.text = boost::lexical_cast<string>(list[i]->id);

			marker.id++;

			marker_map[make_pair(marker.ns, marker.id)] = make_pair(marker, 1);
			// isto substitui ou cria o novo marker no maplist
		}
	}

	marker.pose.position.x = 0;
	marker.pose.position.y = 0;

	marker.text = "origin";

	marker.id++;

	// end of text markers
	// begin line objects

	marker_map[make_pair(marker.ns, marker.id)] = make_pair(marker, 1);  // isto substitui ou cria o novo marker no map

	// Markers for Line objects
	marker.type = visualization_msgs::Marker::LINE_STRIP;
	marker.ns = "objects";

	marker.pose.position.x = 0;
	marker.pose.position.y = 0;
	marker.pose.position.z = 0;

	marker.scale.x = 0.02;
	marker.scale.y = 0.1;
	marker.scale.z = 0.1;

	for (uint i = 0; i < list.size(); i++) {
		marker.color = colormap.color(list[i]->id);

		geometry_msgs::Point p;
		p.z = 0.1;

		marker.points.clear();

		uint l;
		for (l = 0; l < list[i]->shape.lines.size(); l++) {
			p.x = list[i]->shape.lines[l]->xi;
			p.y = list[i]->shape.lines[l]->yi;

			marker.points.push_back(p);
		}

		p.x = list[i]->shape.lines[l - 1]->xf;
		p.y = list[i]->shape.lines[l - 1]->yf;

		marker.points.push_back(p);

		marker.id++;

		marker_map[make_pair(marker.ns, marker.id)] = make_pair(marker,
																1);  // isto substitui ou cria o novo marker no map
	}

	// begin 3D BBox markers

	marker_map[make_pair(marker.ns, marker.id)] = make_pair(marker, 1);  // isto substitui ou cria o novo marker no map

	// Markers for Line objects
	marker.type = visualization_msgs::Marker::CUBE;
	marker.ns = "boxes";

	marker.pose.position.z = 0.5;

	marker.scale.x = 3;
	marker.scale.y = 2;
	marker.scale.z = 1;

	for (uint i = 0; i < list.size(); i++) {
		if (list[i]->shape.lines.size() != 0) {
			marker.pose.position.x = list[i]->position.estimated_x;
			marker.pose.position.y = list[i]->position.estimated_y;
			marker.color.a = 0.5;
			marker.color.r = 0;
			marker.color.g = 1;
			marker.color.b = 0;

			marker.id++;

			marker_map[make_pair(marker.ns, marker.id)] = make_pair(marker,
																	1);  // isto substitui ou cria o novo marker no
			// map
		}
	}

	// para o map todo envio tudo, e meto tudo a false
	// envio todo e tudo o que ainda estiver a false vai com operaração de delete
	for (it = marker_map.begin(); it != marker_map.end(); it++) {
		if (it->second.second == 0)  // se for falso é para apagar
			it->second.first.action = visualization_msgs::Marker::DELETE;
		else if (it->second.second <= -1)  // já foi apagado
		{
			/**
			\todo This should be erased but it is not working now
			marker_map.erase(it);
			*/
			continue;
		}

		marker_vector.push_back(it->second.first);
	}
}

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

int getS(int r, int g, int b)
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

	if (max == 0)
	{
		return 0;
	}
	else
	{
		return (int)((delta * 1.0 / max) * 255);
	}
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