/*
 * SFM.cpp
 *
 *  Created on: Jan 18, 2018
 *      Author: jason
 */

#include "SFM.h"
#include "Feature.h"
#include "Frame.h"
SFM::SFM() {

	ros::NodeHandle nh; // we all know what this is

	this->tf_listener = new tf::TransformListener(nh);

	ros::param::param<std::string>("~camera_topic", CAMERA_TOPIC, D_CAMERA_TOPIC);
	ros::param::param<double>("~canny_blur_sigma",CANNY_BLUR_SIGMA,D_CANNY_BLUR_SIGMA);
	ros::param::param<double>("~canny_thresh1",CANNY_THRESH1,D_CANNY_THRESH1);
	ros::param::param<double>("~canny_thresh2",CANNY_THRESH2,D_CANNY_THRESH2);
	ros::param::param<int>("~max_features",MAX_FEATURES,D_MAX_FEATURES);
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

	Frame last_image;

	int features_needed = MAX_FEATURES - image.features.size();

	if(images.empty()){
		last_image = image;
		images.push_back(image);

	}
	else{
		last_image = images.back();
		images.push_back(image);
		feature_update(last_image.img,image.img);
	}

	feature_replenish(last_image.img,image.img);

	cv::imshow("raw image", temp);
	cv::waitKey(30);
	feature_view(temp);


}

void SFM::feature_detection(cv::Mat temp){


}

void SFM::feature_update(cv::Mat last_img,cv::Mat cur_img){
	std::vector<cv::Point2f> oldPoints;
	std::vector<cv::Point2f> nextPoints;
	std::vector<uchar> status;
	cv::Mat err;
	std::vector<Feature> new_features;
	for (auto e : features){
		 oldPoints.push_back(e.px);
	}
	cv::calcOpticalFlowPyrLK(last_img, cur_img, oldPoints, nextPoints,status, err, cv::Size(21, 21), 3,cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS,30, 0.01));

	int lostFeatures = 0;
	for (int i = 0; i < status.size(); i++) {
		if (status.at(i) == 1) {
			Feature updated_feature = features.at(i);
			updated_feature.px = nextPoints.at(i);
			new_features.push_back(updated_feature);
		}
		else {
			lostFeatures++;
		}
	}
	features = new_features;
	ROS_INFO_STREAM("new_feature_size: " << new_features.size());
	ROS_INFO_STREAM("lostFeatures: " << lostFeatures);

}

void SFM::feature_replenish(cv::Mat last_img, cv::Mat cur_img){
	cv::Mat blurred_img, img;
	img = cur_img;
	cv::GaussianBlur(img, blurred_img, cv::Size(0, 0), CANNY_BLUR_SIGMA);
	cv::Mat canny;
	cv::Canny(blurred_img, canny, CANNY_THRESH1, CANNY_THRESH2);


	if(features.size()<MAX_FEATURES){										//If there are not enough features being tracked
		cv::Mat checkImg = cv::Mat::zeros(img.size(), CV_8U); 				//Used to check for features that are too closed
		for(auto e : features)
		{
			cv::circle(checkImg,e.px,sqrt((img.rows*img.cols)/MAX_FEATURES), cv::Scalar(255), cv::FILLED);
		}

		for(int i = 0;i<img.rows;i++){
			for(int j = 0;j<img.cols;j++){
				if (features.size()>=MAX_FEATURES){
					break;
				}
				if (canny.at<uchar>(i,j)!=0 && checkImg.at<uchar>(i,j)==0){    //if this point is a feature AND not too close to an existing one
					Feature feature;
					feature.px.x = j;
					feature.px.y = i;
					features.push_back(feature);
					cv::circle(checkImg,feature.px,sqrt((img.rows*img.cols)/MAX_FEATURES), cv::Scalar(255), cv::FILLED);

				}
			}
		}

		ROS_INFO_STREAM("feature_size: " << features.size());
		cv::imshow("canny", canny);
		cv::waitKey(30);
		cv::imshow("checkImg",checkImg);
		cv::waitKey(30);

	}
	//do nothing if maximum feature is already satisfied
}

void SFM::feature_view(cv::Mat img){
	cv::Mat img_gray = img;
	cv::Mat img_rgb(img_gray.size(), CV_8UC3);
	  // convert grayscale to color image
	cv::cvtColor(img_gray, img_rgb, CV_GRAY2RGB);
	for(auto e : features){
		cv::drawMarker(img_rgb, e.px, cv::Scalar(255,0,0), cv::MARKER_SQUARE, sqrt((img.rows*img.cols)/MAX_FEATURES), 2);
	}
	cv::imshow("img_rgb",img_rgb);
	cv::waitKey(30);
}

void SFM::feature_tracking(Frame& old_im,Frame& current_im){
	std::vector<cv::Point2f> oldPoints;
	std::vector<cv::Point2f> nextPoints;
	std::vector<cv::Point2f> status;
	cv::Mat err;
	cv::calcOpticalFlowPyrLK(old_im.img,current_im.img,oldPoints, nextPoints,status, err, cv::Size(21, 21), 3,cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS,30, 0.01));

}



