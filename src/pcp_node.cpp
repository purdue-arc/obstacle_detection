/*
 * pcp_node.cpp
 *
 *  Created on: Mar 1, 2018
 *      Author: jasoncd00
 */


#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <sfm/PCloudProc.h>
#include <sfm/SFM.h>



int main(int argc, char** argv){

	ros::init(argc, argv, "pcp_node");

	PCloudProc pcp;

	return 0;

}
