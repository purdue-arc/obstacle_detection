/*
 * Feature.h
 *
 *  Created on: Feb 2, 2018
 *      Author: jason
 */

#ifndef OBSTACLE_DETECTION_INCLUDE_SFM_FEATURE_H_
#define OBSTACLE_DETECTION_INCLUDE_SFM_FEATURE_H_
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
namespace std {

class Feature {
public:
	Feature();
	virtual ~Feature();
	Frame og_image;
	cv::Mat patch;


};

} /* namespace std */

#endif /* OBSTACLE_DETECTION_INCLUDE_SFM_FEATURE_H_ */
