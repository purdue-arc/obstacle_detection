/*
 * Frames.h
 *
 *  Created on: Jan 29, 2018
 *      Author: jason
 */

#ifndef OBSTACLE_DETECTION_INCLUDE_SFM_FRAME_H_
#define OBSTACLE_DETECTION_INCLUDE_SFM_FRAME_H_

namespace std {

class Frame {
public:
	Frame();
	virtual ~Frame();
	struct Frame{
			tf::TransformListener* tf_listener;
			cv::Size image_size;
			cv::Mat_<float> image_K;
			cv::Mat img;
		};
};

} /* namespace std */

#endif /* OBSTACLE_DETECTION_INCLUDE_SFM_FRAME_H_ */
