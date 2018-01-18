/*
 * SFM.cpp
 *
 *  Created on: Jan 18, 2018
 *      Author: jason
 */

#include "SFM.h"

SFM::SFM() {

	ros::NodeHandle nh; // we all know what this is


	std::string CAMERA_TOPIC;
	ros::param::param<std::string>("~camera_topic", CAMERA_TOPIC, D_CAMERA_TOPIC);

	image_transport::ImageTransport it(nh);
	image_transport::CameraSubscriber bottom_cam_sub = it.subscribeCamera(
			CAMERA_TOPIC, 10, &SFM::camera_callback, this);

	ros::spin();

}

SFM::~SFM() {
	// TODO Auto-generated destructor stub
}


void SFM::camera_callback(const sensor_msgs::ImageConstPtr& img,
		const sensor_msgs::CameraInfoConstPtr& cam) {
	ros::Time start = ros::Time::now();

	cv::Mat temp = cv_bridge::toCvShare(img, img->encoding)->image.clone();

	ROS_INFO_STREAM("frame dt in ms: " << (ros::Time::now() - start).toSec() * 1000.0);

}

