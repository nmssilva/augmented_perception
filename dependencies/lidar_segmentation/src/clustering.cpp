/**************************************************************************************************
 Software License Agreement (BSD License)

 Copyright (c) 2011-2013, LAR toolkit developers - University of Aveiro - http://lars.mec.ua.pt
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
\file  clustering.cpp 
\brief Clustering related functions.
\author Daniel Coimbra
*/

#include "lidar_segmentation/clustering.h"
#include "lidar_segmentation/lidar_segmentation.h"


/**
@brief Auxiliary function to the nnClustering - Recursive function
@param points incoming Laser Points
@param point_association pair with the output vector of clusters and its state of association
@param threshold distance value used to break clusters
@param idx index of the scan point
@return void
*/

void recursiveClustering(vector<PointPtr>& points, vector<pair<ClusterPtr,bool> >& point_association, double threshold , uint idx);


int simpleClustering(vector<PointPtr>& points, double threshold, vector<ClusterPtr>& clusters)
{
	
//  ros::Time tic = ros::Time::now();
	
	double euclidean_distance;
	geometry_msgs:: Point cc;
	uint idx;
	
	int idc = 1;	
	ClusterPtr cluster(new Cluster);
		
// 	Add tge firts point to the cluster
	cluster->ranges.push_back(points[0]->range);
	cluster->support_points.push_back(points[0]);
	points[0]->cluster_id = idc;		
	cluster->centroid = calculateClusterCentroid( cluster->support_points );
	cluster->central_point = calculateClusterMedian(cluster->support_points);
	cluster->id = idc;	
	
// 	Determining the number of clusters	
	for( idx = 1; idx < points.size(); idx++)
	{
		
// 		euclidean distance between the current and the previous point.
		euclidean_distance = sqrt( pow( points[idx]->x - points[idx-1]->x ,2) + pow( points[idx]->y - points[idx-1]->y  ,2) ); 
		
// 		if the euclidean distance is bigger than a given threshold, add new cluster.
		if (euclidean_distance > threshold)
		{
			if(cluster->ranges.size()>0)
				clusters.push_back(cluster);
		
// 			make cluster point to a new Cluster.
			cluster.reset(new Cluster);	
			
			idc++;
			
			cluster->ranges.push_back(points[idx]->range);
			cluster->support_points.push_back(points[idx]);
			points[idx]->cluster_id = idc;			
			cluster->centroid = calculateClusterCentroid( cluster->support_points );
			cluster->central_point = calculateClusterMedian(cluster->support_points);
			 
			cluster->id = idc;
				
		}else
		{
			cluster->ranges.push_back(points[idx]->range);
			cluster->support_points.push_back(points[idx]);
			points[idx]->cluster_id = idc;		
			cluster->centroid = calculateClusterCentroid( cluster->support_points );
			cluster->central_point = calculateClusterMedian(cluster->support_points);
			
			cluster->id = idc;
		}
	
		
	}//end for
	
	if(cluster->ranges.size()>0)
		clusters.push_back(cluster);

//----------------------------------------------------------------------------	
//  Just for debug purposes	
// 	for(uint i =  0 ; i< clusters.size(); i++ )
// 	{
// 		ClusterPtr fk = clusters[i];
// 		
// 		cout<< "cluster Simple numero "<< i << endl;
// 		
// 		for(uint j = 0 ; j < fk->support_points.size() ; j++ )
// 		cout<<"valor de xS: "<< fk->support_points[j]->x <<" valor de yS: " <<fk->support_points[j]->y << endl;
// 	}
//----------------------------------------------------------------------------	
//----------------------------------------------------------------------------
// 	ros::Time toc = ros::Time::now();
// 	double duration = (toc-tic).toSec();
 	
// 	stringstream ss;
// 	ss << "src/durations/simple_DUR.txt";
// 	ofstream fpc(ss.str().c_str(), ios::app);
// 					
// 	if (!fpc.is_open()) 
// 	{
// 		cout << "Couldn't open fpc" << endl;
// 		return 1;
// 	}
// 	fpc<< duration << endl;			
// 	fpc.close();	
//----------------------------------------------------------------------------
	
	cout<<"number of clusters simple: "<<clusters.size()<<endl;	
		
	return clusters.size();
} //end functions


int dietmayerClustering( vector<PointPtr>& points, double C0 ,vector<ClusterPtr>& clusters_Dietmayer)
{
// 	ros::Time tic = ros::Time::now();
	
	ClusterPtr cluster_diet(new Cluster); 
	
	double C1, min_distance, threshold_diet, euclidean_distance;
	double r1, r2;
	uint idx;
	int idc = 1;
	
// 	Add tge firts point to the cluster
	cluster_diet->ranges.push_back(points[0]->range);
	cluster_diet->support_points.push_back(points[0]);
	points[0]->cluster_id = idc;		
	cluster_diet->centroid = calculateClusterCentroid(cluster_diet->support_points );
	cluster_diet->central_point = calculateClusterMedian(cluster_diet->support_points);
	cluster_diet->id = idc;	
	
	for( idx = 1; idx < points.size(); idx++)
	{	
		r1 = points[idx-1]->range;  // ri-1
		r2 = points[idx]->range;    //ri
	
		if(r1 < r2)
		{ 
			min_distance = r1; 	
		}
		else
		{ 
			min_distance = r2;	
		}
		
		double delta_ang = (points[idx]->theta - points[idx-1]->theta);
		
		C1 = sqrt(2* (1-cos(delta_ang)) );
		threshold_diet = C0 + C1*min_distance;
		
		euclidean_distance = sqrt( pow( points[idx]->x - points[idx-1]->x ,2) + pow( points[idx]->y - points[idx-1]->y  ,2) ); 
		
		if( euclidean_distance > threshold_diet )
		{
			if(cluster_diet->ranges.size()>0)
				clusters_Dietmayer.push_back(cluster_diet);
			
			cluster_diet.reset(new Cluster);
			idc++;
			
			cluster_diet->ranges.push_back(points[idx]->range);
			cluster_diet->support_points.push_back(points[idx]);
			points[idx]->cluster_id = idc;
			cluster_diet->centroid = calculateClusterCentroid( cluster_diet->support_points );
			cluster_diet->central_point = calculateClusterMedian(cluster_diet->support_points);
			
			cluster_diet->id = idc;
			
		}else
		{
			cluster_diet->ranges.push_back(points[idx]->range);
			cluster_diet->support_points.push_back(points[idx]);
			points[idx]->cluster_id = idc;
			cluster_diet->centroid = calculateClusterCentroid( cluster_diet->support_points );
			cluster_diet->central_point = calculateClusterMedian(cluster_diet->support_points);
			
			cluster_diet->id = idc;
		}		
	}//end for
	
	if(cluster_diet->ranges.size()>0)
		clusters_Dietmayer.push_back(cluster_diet);
			
	
	
// 	----------------------------------------------------------------------------	
// 	ros::Time toc = ros::Time::now();
// 	double duration = (toc-tic).toSec();

// 	stringstream ss;
// 	ss << "src/durations/diet_DUR.txt";
// 	ofstream fpc(ss.str().c_str(), ios::app);
// 					
// 	if (!fpc.is_open()) 
// 	{
// 		cout << "Couldn't open fpc" << endl;
// 		return 1;
// 	}
// 	fpc<< duration << endl;			
// 	fpc.close();	
// 	----------------------------------------------------------------------------
	
	cout<<"number of clusters Dietmayer: "<<clusters_Dietmayer.size()<<endl;
		
	return clusters_Dietmayer.size();
} //end function


int premebidaClustering( vector<PointPtr>& points, double threshold_prem , vector<ClusterPtr>& clusters_Premebida)
{
// 	ros::Time tic = ros::Time::now();
	
	ClusterPtr cluster_prem(new Cluster); 
	
	uint idx;
	int idc = 1;
	vector<double> si;
	vector<double> si_next;
 	double dmax = 3.0;      //huge value
	double euclidean_distance;
	
// 	Add tge firts point to the cluster
	cluster_prem->ranges.push_back(points[0]->range);
	cluster_prem->support_points.push_back(points[0]);		
	cluster_prem->centroid = calculateClusterCentroid(cluster_prem->support_points );
	cluster_prem->central_point = calculateClusterMedian(cluster_prem->support_points);
	cluster_prem->id = idc;	
	
	
	euclidean_distance = sqrt( pow( points[1]->x - points[0]->x ,2) + pow( points[1]->y - points[0]->y  ,2) ); 
		
	// push_back to form the 1st pair
	if(dmax > euclidean_distance )	
	{
		cluster_prem->ranges.push_back(points[1]->range);
		cluster_prem->support_points.push_back(points[1]);		
		cluster_prem->centroid = calculateClusterCentroid(cluster_prem->support_points );
		cluster_prem->central_point = calculateClusterMedian(cluster_prem->support_points);
		cluster_prem->id = idc;	
	}
	
	//Work with pairs of points
	for( idx = 1; idx < points.size()-1 ; idx++)
	{
		//See if the next point is too distanced
		euclidean_distance = sqrt( pow( points[idx]->x - points[idx-1]->x ,2) + pow( points[idx]->y - points[idx-1]->y  ,2) ); 
		
		if(dmax < euclidean_distance )	
		{
			//a Break-point is detected
			if(cluster_prem->ranges.size()>0)
				clusters_Premebida.push_back(cluster_prem);
			
			cluster_prem.reset(new Cluster);
			idc++;
			
			cluster_prem->ranges.push_back(points[idx]->range);
			cluster_prem->support_points.push_back(points[idx]);	
			
			cluster_prem->centroid = calculateClusterCentroid( cluster_prem->support_points );
			cluster_prem->central_point = calculateClusterMedian(cluster_prem ->support_points );	
			cluster_prem->id = idc;
		
			// push_back to form the 1st pair
			double euclidean_distance_condition = sqrt( pow( points[idx]->x - points[idx+1]->x ,2) + pow( points[idx]->y - points[idx+1]->y  ,2) ); 
			
			if(dmax > euclidean_distance_condition )	
			{
				//pair is formed
				cluster_prem->ranges.push_back(points[idx+1]->range);
				cluster_prem->support_points.push_back(points[idx+1]);		
				cluster_prem->centroid = calculateClusterCentroid( cluster_prem->support_points );
				cluster_prem->central_point = calculateClusterMedian(cluster_prem->support_points);
				cluster_prem->id = idc;	
			}	
			
			continue;
		}	

		//Calculate the features
		si = rangeFeatures(points[idx-1]->range, points[idx]->range, points[idx-1] , points[idx]);
		si_next = rangeFeatures(points[idx]->range, points[idx+1]->range, points[idx] , points[idx+1]);	
				
		//Calculate the Coise Distance
		double CosDi = cosineDistance( si , si_next );
		
		//See if p+1 is part of the segment
		if(CosDi < threshold_prem)
		{	
 			if(cluster_prem->ranges.size() > 0 )
				clusters_Premebida.push_back(cluster_prem);
			
			cluster_prem.reset(new Cluster);
			idc++;
			
			cluster_prem->ranges.push_back(points[idx+1]->range);
			cluster_prem->support_points.push_back(points[idx+1]);		
			cluster_prem->centroid = calculateClusterCentroid( cluster_prem->support_points );
			cluster_prem->central_point = calculateClusterMedian(cluster_prem->support_points);
			cluster_prem->id = idc;	
			
			//increment idx value
			idx++;
			
			//Check if it is possible to form a pair
			double euclidean_distance_condition = sqrt( pow( points[idx]->x - points[idx+1]->x ,2) + pow( points[idx]->y - points[idx+1]->y  ,2) ); 
			
			if(dmax > euclidean_distance_condition )	
			{
				cluster_prem->ranges.push_back(points[idx+1]->range);
				cluster_prem->support_points.push_back(points[idx+1]);		
				cluster_prem->centroid = calculateClusterCentroid( cluster_prem->support_points );
				cluster_prem->central_point = calculateClusterMedian(cluster_prem->support_points);
				cluster_prem->id = idc;	
			}	
				
		}else
		{	
			cluster_prem->ranges.push_back(points[idx+1]->range);
			cluster_prem->support_points.push_back(points[idx+1]);		
			cluster_prem->centroid = calculateClusterCentroid( cluster_prem->support_points );
			cluster_prem->central_point = calculateClusterMedian(cluster_prem->support_points);
			cluster_prem->id = idc;	
		}	
	
	}//end for
	
	if(cluster_prem->ranges.size()>0)
		clusters_Premebida.push_back(cluster_prem);
			
//----------------------------------------------------------------------------	
// 	ros::Time toc = ros::Time::now();
// 	double duration = (toc-tic).toSec();
	
// 	stringstream ss;
// 	ss << "src/durations/prem_DUR.txt";
// 	ofstream fpc(ss.str().c_str(), ios::app);
// 					
// 	if (!fpc.is_open()) 
// 	{
// 		cout << "Couldn't open fpc" << endl;
// 		return 1;
// 	}
// 	fpc<< duration << endl;			
// 	fpc.close();	
//----------------------------------------------------------------------------
	
	cout<<"number of clusters Premebida: "<<clusters_Premebida.size()<<endl;	
		
	return clusters_Premebida.size();
} //end function
 

int abdClustering( vector<PointPtr>& points , double lambda ,vector<ClusterPtr>& clusters_ABD)
{
// 	ros::Time tic = ros::Time::now();
	
	ClusterPtr cluster_abd(new Cluster);
	
	uint idx;
	int idc= 1;
	double gr = 0.03; //[m] -> see laser datasheet
	double Dmax , euclidean_distance;
	
// 	Add tge firts point to the cluster
	cluster_abd->ranges.push_back(points[0]->range);
	cluster_abd->support_points.push_back(points[0]);		
	cluster_abd->centroid = calculateClusterCentroid(cluster_abd->support_points );
	cluster_abd->central_point = calculateClusterMedian(cluster_abd->support_points);
	cluster_abd->id = idc;	
	
	//Determining the number of clusters	
	for( idx = 1; idx < points.size(); idx++)
	{
		           
		double delta_ang = (points[idx]->theta - points[idx-1]->theta);
		
		Dmax = ( (points[idx-1]->range) * ( sin(delta_ang ) /  sin(lambda - delta_ang ) ) ) + 3*gr;
		
		euclidean_distance = sqrt( pow( points[idx]->x - points[idx-1]->x ,2) + pow( points[idx]->y - points[idx-1]->y  ,2) ); 
		
		if(euclidean_distance  > Dmax)
		{
			if(cluster_abd->ranges.size()>0)
				clusters_ABD.push_back(cluster_abd);
			
			cluster_abd.reset(new Cluster);
			idc++;
			
			cluster_abd->ranges.push_back(points[idx]->range);
			cluster_abd->support_points.push_back(points[idx]);
			
			cluster_abd->centroid = calculateClusterCentroid( cluster_abd->support_points );
			cluster_abd->central_point = calculateClusterMedian(cluster_abd->support_points);
			
			cluster_abd->id = idc;
			
		}else	
		{
			cluster_abd->ranges.push_back(points[idx]->range);
			cluster_abd->support_points.push_back(points[idx]);
			
			cluster_abd->centroid = calculateClusterCentroid( cluster_abd->support_points );
			cluster_abd->central_point = calculateClusterMedian(cluster_abd->support_points);
			
			cluster_abd->id = idc;
		}			
		
	} //end for

		
	if(cluster_abd->ranges.size()>0)
		clusters_ABD.push_back(cluster_abd);
			
//----------------------------------------------------------------------------	
// 	ros::Time toc = ros::Time::now();
// 	double duration = (toc-tic).toSec();

// 	stringstream ss;
// 	ss << "src/durations/abd_DUR.txt";
// 	ofstream fpc(ss.str().c_str(), ios::app);
// 					
// 	if (!fpc.is_open()) 
// 	{
// 		cout << "Couldn't open fpc" << endl;
// 		return 1;
// 	}
// 	fpc<< duration << endl;			
// 	fpc.close();	
//----------------------------------------------------------------------------
	
	cout<<"number of clusters ABD: "<<clusters_ABD.size()<<endl;
	
	return clusters_ABD.size();
} //end function
// 
// 
int nnClustering( vector<PointPtr>& points, double threshold , vector<ClusterPtr>& clusters_nn)
{
	/* Note to myself:
		The make_pair STL (Standard Template Library) function creates a pair structure that contains two data elements of any type
	*/
	
	//Create empty point associations
// 	ros::Time tic = ros::Time::now();
	
	vector<pair<ClusterPtr,bool> > point_association;
	point_association.resize( points.size(), make_pair(ClusterPtr(), false));  
	
	int idc = 0;
	
	//Determining the number of clusters	
	for(uint idx = 1; idx < points.size(); idx++)
	{
		//If the point wasn't yet associated with any cluster
		if( point_association[idx].second == false )
		{
			//Create new cluster_nn
			ClusterPtr cluster_nn(new Cluster);
			idc++;
			
			cluster_nn->ranges.push_back(points[idx]->range);
			cluster_nn->support_points.push_back(points[idx]);
			cluster_nn->id = idc;
			
			point_association[idx].first = cluster_nn;
			point_association[idx].second = true;
			
			clusters_nn.push_back(cluster_nn);
		}
		
		//Go though all the other points and see which ones associate with this measurement	
		recursiveClustering(points, point_association , threshold , idx );
		
	} //end for
	
	for(uint m=0; m<clusters_nn.size() ;m++)
	{
		clusters_nn[m]->centroid = calculateClusterCentroid(clusters_nn[m]->support_points);
		clusters_nn[m]->central_point = calculateClusterMedian( clusters_nn[m]->support_points );
	}
	
	
//----------------------------------------------------------------------------		
// 	ros::Time toc = ros::Time::now();
// 	double duration = (toc-tic).toSec();

// 	stringstream ss;
// 	ss << "src/durations/SNN_DUR.txt";
// 	ofstream fpc(ss.str().c_str(), ios::app);
// 					
// 	if (!fpc.is_open()) 
// 	{
// 		cout << "Couldn't open fpc" << endl;
// 		return 1;
// 	}
// 	fpc<< duration << endl;			
// 	fpc.close();	
//----------------------------------------------------------------------------
	
	//cout<<"number of clusters nearest neighbour: "<<clusters_nn.size()<<endl;
	
	return clusters_nn.size();
	
} //end function


int santosClustering( vector<PointPtr>& points, double C0, double beta, vector<ClusterPtr>& clusters_Santos)
{
	
// 	ros::Time tic = ros::Time::now();
	
	ClusterPtr cluster_Santos(new Cluster); 
	
	//Calculate the threshold
	double C1, min_distance, threshold_Santos, euclidean_distance;
	double r1 , r2;
	uint idx;
	int idc = 1;
	
// 	Add tge firts point to the cluster
	cluster_Santos->ranges.push_back(points[0]->range);
	cluster_Santos->support_points.push_back(points[0]);		
	cluster_Santos->centroid = calculateClusterCentroid(cluster_Santos->support_points );
	cluster_Santos->central_point = calculateClusterMedian(cluster_Santos->support_points);
	cluster_Santos->id = idc;		

	for( idx = 1; idx < points.size(); idx++)
	{	
		r1 = points[idx-1]->range;  // ri-1
		r2 = points[idx]->range; //ri
	
		if(r1 < r2)
		{ 
			min_distance = r1; 	
		}
		else
		{ 
			min_distance = r2;	
		}
		
		double delta_ang = (points[idx]->theta - points[idx-1]->theta);
		
		C1 = sqrt(2* (1-cos(delta_ang)) );
	
		threshold_Santos = C0 + ((C1*min_distance)/( (1/tan(beta))*(cos( delta_ang /2) - sin( delta_ang /2) ) ) );  
		
		euclidean_distance = sqrt( pow( points[idx]->x - points[idx-1]->x ,2) + pow( points[idx]->y - points[idx-1]->y  ,2) ); 
		
		if( euclidean_distance > threshold_Santos )
		{
			if(cluster_Santos->ranges.size()>0)
				clusters_Santos.push_back(cluster_Santos);
			
			cluster_Santos.reset(new Cluster);
			idc++;
			
			cluster_Santos->ranges.push_back(points[idx]->range);
			cluster_Santos->support_points.push_back(points[idx]);
			
			cluster_Santos->centroid = calculateClusterCentroid( cluster_Santos->support_points );
			cluster_Santos->central_point = calculateClusterMedian(cluster_Santos->support_points);
			
			cluster_Santos->id = idc;
			
		}else
		{
			cluster_Santos->ranges.push_back(points[idx]->range);
			cluster_Santos->support_points.push_back(points[idx]);
			
			cluster_Santos->centroid = calculateClusterCentroid( cluster_Santos->support_points );
			cluster_Santos->central_point = calculateClusterMedian(cluster_Santos->support_points);
			
			cluster_Santos->id = idc;
		}		
	}//end for
	
	if(cluster_Santos->ranges.size()>0)
		clusters_Santos.push_back(cluster_Santos);
			
	
//----------------------------------------------------------------------------		
// 	ros::Time toc = ros::Time::now();
// 	double duration = (toc-tic).toSec();

// 	stringstream ss;
// 	ss << "src/durations/santos_DUR.txt";
// 	ofstream fpc(ss.str().c_str(), ios::app);
// 					
// 	if (!fpc.is_open()) 
// 	{
// 		cout << "Couldn't open fpc" << endl;
// 		return 1;
// 	}
// 	fpc<< duration << endl;			
// 	fpc.close();	
// 	//----------------------------------------------------------------------------
	
	cout<<"number of clusters Santos: "<<clusters_Santos.size()<<endl;	
		
	return clusters_Santos.size();
} //end function*/


vector<double>  rangeFeatures( double range1, double range2, PointPtr range1cart, PointPtr range2cart)
{
	double delta_x , delta_y;
	double f1,f2,f3,f4,f5,f6;
	vector<double> si;
	
	delta_x = fabs(range1cart->x - range2cart->x);
	delta_y = fabs(range1cart->y - range2cart->y);
	
	f1 = sqrt( pow(delta_x, 2) + pow(delta_y, 2) );
	si.push_back(f1);
	
	f2 = (range1 + range2)/2;
	si.push_back(f2);
	
	f3 = f2*delta_x;
	si.push_back(f3);
		
	f4 = f2*delta_y;
	si.push_back(f4);
	
	f5 = sqrt( pow(range1-f2 ,2) + pow(range2-f2,2));  
	si.push_back(f5);

	f6 = pow(f5 ,2);
	si.push_back(f6);
	
	return si;	
}

double cosineDistance(vector<double>&  vect1, vector<double>& vect2)
{
	double sum0 = 0.0, sum1= 0.0, sum2 = 0.0;
	uint i;
	
	for(i = 0; i < vect1.size(); i++ )
	{
		sum0 += (vect1[i]*vect2[i]); 
		
		sum1 += (vect1[i]*vect1[i]);
		
		sum2 += (vect2[i]*vect2[i]);
	}
	
	double cosD =  sum0 / ( sqrt(sum1) * sqrt(sum2) );
	
// 	cout << "cos distance " << cosD << endl; 
	
	return cosD;
}


double deg2rad(double angle)
{
	return M_PI * angle/180.0;
}

double euclideanDistance(geometry_msgs::Point& point1 , geometry_msgs::Point& point2 )
{
	double dist;
	
	dist = sqrt( pow( point2.x - point1.x ,2) + pow( point2.y - point1.y  ,2) ); 

	return dist;
	
} //end function

PointPtr calculateClusterCentroid( vector<PointPtr> support_points )
{
		
	double sum_x = 0.0;
	double sum_y = 0.0;
	
	for (uint idx = 0; idx< support_points.size() ; idx++)
	{
		sum_x += support_points[idx]->x;
		sum_y += support_points[idx]->y;
	}
	
	PointPtr centroid(new Point); 
	
	centroid->x= sum_x / support_points.size();
	centroid->y= sum_y / support_points.size();  
	
	return centroid;	
}


geometry_msgs::Point  calculateClusterCentroid( vector<geometry_msgs::Point> support_points )
{
		
	double sum_x = 0.0;
	double sum_y = 0.0;
	
	for (uint idx = 0; idx< support_points.size() ; idx++)
	{
		sum_x += support_points[idx].x;
		sum_y += support_points[idx].y;
	}
	
	geometry_msgs::Point centroid;
			
	centroid.x= sum_x / support_points.size();
	centroid.y= sum_y / support_points.size();  
	centroid.z= 0;
	
//	cout<< "centroidesao" << centroid.x << centroid.y <<endl;
	
	return centroid;
	
} //end function

PointPtr calculateClusterMedian(vector<PointPtr> support_points)
{
	PointPtr median(new Point);
	
	vector<double>  x_values;
	vector<double>  y_values;
	double x, y;	
	
	for (uint idx = 0; idx < support_points.size() ; idx++)
	{		
	 	x = support_points[idx]->x;
		y = support_points[idx]->y;
		
		x_values.push_back(x);
		y_values.push_back(y);
	}
	
	sort(x_values.begin() ,x_values.end());
	sort(y_values.begin() ,y_values.end());
	
	if ( support_points.size() % 2 == 0 ) //its even
	{
		median->x = ( support_points[support_points.size()/2- 1]->x +  support_points[support_points.size()/2]->x )/2;
		median->y = ( support_points[support_points.size()/2- 1]->y +  support_points[support_points.size()/2]->y )/2; 
	}
	else //its odd
	{
		median->x = support_points[support_points.size()/2]->x;  
		median->y = support_points[support_points.size()/2]->y;
	}
	
	return median;
	
} //end functions

geometry_msgs::Point calculateClusterMedian(vector<geometry_msgs::Point> support_points)
{
	geometry_msgs::Point median;
	
	vector<double>  x_values;
	vector<double>  y_values;
	double x, y;
	
	
	for (uint idx = 0; idx < support_points.size() ; idx++)
	{		
	 	x = support_points[idx].x;
		y = support_points[idx].y;
		
		x_values.push_back(x);
		y_values.push_back(y);
	}
	
	sort(x_values.begin() ,x_values.end());
	sort(y_values.begin() ,y_values.end());
	
	if ( support_points.size() % 2 == 0 ) //its even
	{
		median.x = ( support_points[support_points.size()/2- 1].x +  support_points[support_points.size()/2].x )/2;
		median.y = ( support_points[support_points.size()/2- 1].y +  support_points[support_points.size()/2].y )/2; 
	}
	else //its odd
	{
		median.x = support_points[support_points.size()/2].x;  
		median.y = support_points[support_points.size()/2].y;
	}
	
	return median;
	
} //end function

/* auxiliary function of nnClustering. */
void recursiveClustering(vector<PointPtr>& points, vector<pair<ClusterPtr,bool> >& point_association, double threshold , uint idx)
{
	ClusterPtr null_ptr;
	
	for( uint i = 0; i< points.size() ; i++)
	{
		   
		//if the point wasn't yet associated with any cluster
		if(point_association[i].second == false )
		{

// 			double dist = sqrt( pow( points[idx]->range, 2) + pow( points[i]->range, 2) );
					
			double dist = 	sqrt( pow( points[idx]->x - points[i]->x ,2) + pow( points[idx]->y - points[i]->y  ,2) ); 	
					
			//This point associates with the point idx !
			if(dist < threshold )
			{
				//Put the association pointing to the existing measurement

				point_association[i].first = point_association[idx].first;
				point_association[i].second = true;	
				
				//Go to the existing measurement and add this new point
				point_association[idx].first->support_points.push_back(points[i]); 
				
				recursiveClustering(points, point_association, threshold ,i );
			}
			
		} //end if 
		
	} //end for
	
} //end function
