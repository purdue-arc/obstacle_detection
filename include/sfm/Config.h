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
#define D_CANNY_BLUR_SIGMA 2
#define D_CANNY_THRESH1 50
#define D_CANNY_THRESH2 200


extern std::string CAMERA_TOPIC;
extern double CANNY_BLUR_SIGMA;
extern double CANNY_THRESH1;
extern double CANNY_THRESH2;

#endif /* OBSTACLE_DETECTION_INCLUDE_SFM_CONFIG_H_ */
