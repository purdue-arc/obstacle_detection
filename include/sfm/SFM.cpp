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
	ros::param::param<double>("~canny_blur_sigma",CANNY_BLUR_SIGMA,D_CANNY_BLUR_SIGMA);
	ros::param::param<double>("~canny_thresh1",CANNY_THRESH1,D_CANNY_THRESH1);
	ros::param::param<double>("~canny_thresh2",CANNY_THRESH2,D_CANNY_THRESH2);

	image_transport::ImageTransport it(nh);
	image_transport::CameraSubscriber bottom_cam_sub = it.subscribeCamera(CAMERA_TOPIC, 10, &SFM::camera_callback, this);

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
	image.image_K = (cv::Mat_<float>(3, 3) << cam->K.at(0), cam->K.at(1), cam->K.at(2), cam->K.at(3), cam->K.at(4), cam->K.at(5), cam->K.at(6), cam->K.at(7), cam->K.at(8));
	image.time = cam->header.stamp;
	feature_detection(image.img);
	if(!images.empty()){ //
		images.push_back(image);
		//feature_tracking(images.back(),image);								//do klt feature tracking

	}
	else{
		images.push_back(image);

	}
	cv::imshow("raw image", temp);
	cv::waitKey(30);
}

void SFM::feature_detection(cv::Mat temp){
	cv::Mat blurred_img, img;
	img = temp;
	cv::GaussianBlur(img, blurred_img, cv::Size(0, 0), CANNY_BLUR_SIGMA);
	cv::Mat canny;
	cv::Canny(blurred_img, canny, CANNY_THRESH1, CANNY_THRESH2);
	cv::imshow("canny", canny);
	cv::waitKey(30);

}

void SFM::feature_tracking(Frame& old_im,Frame& current_im){
	std::vector<cv::Point2f> oldPoints;
	std::vector<cv::Point2f> nextPoints;
	std::vector<cv::Point2f> status;
	cv::Mat err;


	cv::calcOpticalFlowPyrLK(old_im.img,current_im.img,oldPoints, nextPoints,status, err, cv::Size(21, 21), 3,cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS,30, 0.01));

}




