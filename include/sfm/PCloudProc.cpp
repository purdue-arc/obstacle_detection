/*
 * PCloudProc.cpp
 *
 *  Created on: Feb 26, 2018
 *      Author: jasoncd00
 */

#include "SFM.h"
#include "PCloudProc.h"
#include "Feature.h"
#include "Frame.h"
#include <boost/foreach.hpp>
#include <sensor_msgs/PointCloud2.h>

PCloudProc::PCloudProc() {

	ros::NodeHandle nh; // we all know what this is

	this->tf_listener = new tf::TransformListener(nh);

	//ros::param::param<std::string>("~cam0_topic", CAM0_TOPIC, D_CAM0_TOPIC);
	//ros::param::param<double>("~canny_blur_sigma",CANNY_BLUR_SIGMA,D_CANNY_BLUR_SIGMA);
	//ros::param::param<double>("~canny_thresh1",CANNY_THRESH1,D_CANNY_THRESH1);
	//ros::param::param<double>("~canny_thresh2",CANNY_THRESH2,D_CANNY_THRESH2);
	//ros::param::param<int>("~max_features",MAX_FEATURES,D_MAX_FEATURES);
	//image_transport::ImageTransport it(nh);
	//image_transport::CameraSubscriber cam0_sub = it.subscribeCamera(CAM0_TOPIC, 10, &PCloudProc::camera_callback, this);

	//ros::param::param("ec_cluster_tolerance", cluster_tolerance, 0.01);
	//ros::param::param("ec_min_cluster_size", min_cluster_size, 10);
	//ros::param::param("ec_max_cluster_size", max_cluster_size, 10000);
	this->pcp_sub = nh.subscribe<sensor_msgs::PointCloud2>("points2",100,&PCloudProc::pcp_callback,this);
	this->pcp_pub = nh.advertise<PointCloud>("points2out",1);
	ROS_INFO_STREAM("After subscriber");
	ros::spin();

}

PCloudProc::~PCloudProc() {
	delete this->tf_listener;
}


typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

void PCloudProc::pcp_callback(const sensor_msgs::PointCloud2ConstPtr& msg) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr copy(new pcl::PointCloud<pcl::PointXYZ>);
	//copy = pcl::PointCloud(*msg);
	//copyPointCloud(*msg,*copy);
	//pcl_conversions::toPCL(*msg.px,*copy);

	pcl::PCLPointCloud2 pcl_pc2;
	pcl_conversions::toPCL(*msg,pcl_pc2);
	pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);

	ROS_INFO_STREAM("Cloud: width = " << msg->width);
	ROS_INFO_STREAM("Cloud: height = " << msg->height);
	ROS_INFO_STREAM("copy: width = " << temp_cloud->width);
	ROS_INFO_STREAM("copy: height = " << temp_cloud->height);
	//BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points)
	//	ROS_INFO_STREAM("(" << pt.x << "," << pt.y << "," << pt.z <<")");

	ground_filter(*temp_cloud);

	//ROS_INFO_STREAM("frame dt in ms: " << (ros::Time::now() - start).toSec() * 1000.0);

}

void PCloudProc::ground_filter(const PointCloud cloud){ 		//subtracts the ground from the point cloud (exclude everything below z=15cm?)
	cloud_no_ground =  pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

	ROS_INFO_STREAM("IN GROUND FILTER");
	for (size_t i = 0; i < cloud.points.size(); i++)
	  {
		//ROS_INFO_STREAM(cloud->points.size());
		//ROS_INFO_STREAM(i);
		if(cloud.points[i].z > 0.15)	{						//if (it's above 15cm) need to use correct reference frame.
			cloud_no_ground->points.push_back(cloud.points[i]);

		}
	    else{
	    	//i--;
	    	//ROS_INFO_STREAM("help");
	    }

	  }
	cluster_extraction(cloud_no_ground);
}


void PCloudProc::cluster_extraction(const PointCloud::Ptr cloud_no_ground){
	//ROS_INFO_STREAM("cluster extraction" << cloud_no_ground->points.size());

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud (cloud_no_ground);
	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;


	ec.setClusterTolerance (0.02); // 2cm
	ec.setMinClusterSize (100);
	ec.setMaxClusterSize (25000);
	ec.setSearchMethod (tree);
	ec.setInputCloud (cloud_no_ground);
	ec.extract (cluster_indices);

	int j = 0;
	  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	  {
	    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
	    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
	      cloud_cluster->points.push_back (cloud_no_ground->points[*pit]); //*
	    cloud_cluster->width = cloud_cluster->points.size ();
	    cloud_cluster->height = 1;
	    cloud_cluster->is_dense = true;

	    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
	    //pcp_pub.publish(cloud_cluster);
	    std::stringstream ss;
	    ss << "cloud_cluster_" << j << ".pcd";
	    writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
	    pcl::compute3DCentroid(*cloud_cluster,centroid);
	    centroids.push_back(centroid);
	    ROS_INFO_STREAM(centroid);
	    j++;
	  }
}





