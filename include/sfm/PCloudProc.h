/*
 * PCloudProc.h
 *
 *  Created on: Feb 26, 2018
 *      Author: jasoncd00
 */

#ifndef OBSTACLE_DETECTION_INCLUDE_SFM_PCLOUDPROC_H_
#define OBSTACLE_DETECTION_INCLUDE_SFM_PCLOUDPROC_H_


#include <ros/ros.h>

#include <sfm/Config.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/video.hpp"
#include <vector>
#include <string>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <tf/tfMessage.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <Eigen/Core>
#include "Feature.h"
#include "Frame.h"
#include "SFM.h"

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class PCloudProc {
public:
	PCloudProc();
	virtual ~PCloudProc();


	tf::TransformListener* tf_listener;
	ros::Subscriber pcp_sub;
	ros::Publisher pcp_pub;
	void pcp_callback(const sensor_msgs::PointCloud2ConstPtr& msg);
	void feature_tracking(Frame& old_im, Frame& new_im);
	void feature_replenish(cv::Mat last_img, cv::Mat cur_img);
	void feature_update(cv::Mat last_img, cv::Mat cur_img);
	void feature_view(cv::Mat temp);
	void cluster_extraction(const PointCloud::Ptr cloud_no_ground);
	void ground_filter(const PointCloud cloud);
	std::vector<Feature> features;
	std::deque<Frame> images;
	double cluster_tolerance;
	int min_cluster_size, max_cluster_size;
	pcl::PCDWriter writer;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_no_ground;
	std::vector<Eigen::Vector4f> centroids;
	Eigen::Vector4f centroid;
};



#endif /* OBSTACLE_DETECTION_INCLUDE_SFM_PCLOUDPROC_H_ */
