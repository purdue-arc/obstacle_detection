/*
 * SFM.cpp
 *
 *  Created on: Jan 18, 2018
 *      Author: jason
 */

#include "SFM.h"

#include "Frame.h"
SFM::SFM() {

	ros::NodeHandle nh; // we all know what this is

	this->tf_listener = new tf::TransformListener(nh);

	ros::param::param<std::string>("~camera_topic", CAMERA_TOPIC, D_CAMERA_TOPIC);

	image_transport::ImageTransport it(nh);
	image_transport::CameraSubscriber bottom_cam_sub = it.subscribeCamera(
			CAMERA_TOPIC, 10, &SFM::camera_callback, this);

	ros::spin();

}

SFM::~SFM() {
	delete this->tf_listener;
}


void SFM::camera_callback(const sensor_msgs::ImageConstPtr& img,const sensor_msgs::CameraInfoConstPtr& cam) {
	ros::Time start = ros::Time::now();
	cv::Mat temp = cv_bridge::toCvShare(img, img->encoding)->image.clone();

	ROS_INFO_STREAM("frame dt in ms: " << (ros::Time::now() - start).toSec() * 1000.0);

	Frame image;
	image.img=temp;
	image.image_K = cam->K;
	image.time = cam->header.stamp;
	images.push_back(image);
	if(!images.empty()){ //
		feature_tracking(images.back() ,image);								//do klt feature tracking

	}

}

void SFM::image_processing(cv::Mat temp){



}

void SFM::feature_tracking(Frame& old_im,Frame& current_im){
	std::vector<cv::Point2f> oldPoints;
	std::vector<cv::Point2f> nextPoints;
	std::vector<cv::Point2f> status;
	cv::Mat err;

	oldPoints.at(2);

	cv::calcOpticalFlowPyrLK(old_im.img,current_im.img,oldPoints, nextPoints,status, err, cv::Size(21, 21), 3,cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS,30, 0.01));

}




