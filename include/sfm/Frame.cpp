/*
 * Frames.cpp
 *
 *  Created on: Jan 29, 2018
 *      Author: jason
 */

#include "Frame.h"

namespace std {

Frame::Frame() {
	// TODO Auto-generated constructor stub
	struct Frame{
		tf::TransformListener* tf_listener;
		cv::Size image_size;
		cv::Mat_<float> image_K;
		cv::Mat img;
	};

}

Frame::~Frame() {
	// TODO Auto-generated destructor stub
}

} /* namespace std */
