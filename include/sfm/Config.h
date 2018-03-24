/*
 * Config.h
 *
 *  Created on: Jan 18, 2018
 *      Author: jason
 */

#ifndef OBSTACLE_DETECTION_INCLUDE_SFM_CONFIG_H_
#define OBSTACLE_DETECTION_INCLUDE_SFM_CONFIG_H_

#include <ros/ros.h>
#include <string.h>

#define D_CAMERA_TOPIC "camera/image_rect"
#define D_CAM0_TOPIC "cam0/image_raw"
#define D_CAM1_TOPIC "cam1/image_raw"
#define D_CANNY_BLUR_SIGMA 1
#define D_CANNY_THRESH1 20
#define D_CANNY_THRESH2 200
#define D_MAX_FEATURES 50000
#define D_STEREO_CAM_TOPIC "points2"

extern std::string CAMERA_TOPIC;
extern std::string CAM0_TOPIC;
extern std::string CAM1_TOPIC;
extern double CANNY_BLUR_SIGMA;
extern double CANNY_THRESH1;
extern double CANNY_THRESH2;
extern int MAX_FEATURES;
extern std::string STEREO_CAM_TOPIC;
#endif /* OBSTACLE_DETECTION_INCLUDE_SFM_CONFIG_H_ */
