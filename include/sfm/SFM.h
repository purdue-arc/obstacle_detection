/*
 * SFM.h
 *
 *  Created on: Jan 18, 2018
 *      Author: jason
 */

#ifndef OBSTACLE_DETECTION_INCLUDE_SFM_SFM_H_
#define OBSTACLE_DETECTION_INCLUDE_SFM_SFM_H_

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

class SFM {
public:
	SFM();
	virtual ~SFM();


	void camera_callback(const sensor_msgs::ImageConstPtr& img, const sensor_msgs::CameraInfoConstPtr& cam);
};

#endif /* OBSTACLE_DETECTION_INCLUDE_SFM_SFM_H_ */