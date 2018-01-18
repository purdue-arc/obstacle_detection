/*
 * sfm_node.cpp
 *
 *  Created on: Jan 18, 2018
 *      Author: jason
 */


#include <ros/ros.h>

#include <sfm/SFM.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "sfm_node"); // initializes ros

	SFM sfm;

	return 0;
}

