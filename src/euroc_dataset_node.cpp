/*
 * pauvsi_vio_node.cpp
 *
 *  Created on: Aug 2, 2017
 *      Author: kevin
 */



#include <ros/ros.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf/transform_broadcaster.h>


#define PI 3.1415

ros::Publisher* cam0infopub;
ros::Publisher* cam1infopub;
ros::Subscriber* cam0sub;
ros::Subscriber* cam1sub;
ros::Subscriber* gtSub;

tf::TransformBroadcaster* tfbr;

tf::StampedTransform imu2cam0, imu2cam1;

void gtCallback(const geometry_msgs::TransformStampedConstPtr& msg){

	imu2cam1.stamp_ = msg->header.stamp;
	tfbr->sendTransform(imu2cam1);

	imu2cam0.stamp_ = msg->header.stamp;
	tfbr->sendTransform(imu2cam0);

	tf::StampedTransform base2vicon, world2vicon, base2imu;

	base2vicon.child_frame_id_ = "base_link";
	base2vicon.frame_id_ = "vicon";
	base2vicon.setRotation(tf::Quaternion(0, 0, 0, 1));
	base2vicon.setOrigin(tf::Vector3(0, 0, 0.1));
	base2vicon.stamp_ = msg->header.stamp;

	tfbr->sendTransform(base2vicon);

	base2imu.child_frame_id_ = "imu4";
	base2imu.frame_id_ = "base_link";
	tf::Quaternion quat = tf::Quaternion(PI/2, 0, PI);
	base2imu.setRotation(quat);
	base2imu.setOrigin(tf::Vector3(0.1, 0, 0));
	base2imu.stamp_ = msg->header.stamp;

	tfbr->sendTransform(base2imu);

	world2vicon.child_frame_id_ = "vicon";
	world2vicon.frame_id_ = "world";

	world2vicon.stamp_ = msg->header.stamp;

	world2vicon.setRotation(tf::Quaternion(msg->transform.rotation.x, msg->transform.rotation.y, msg->transform.rotation.z, msg->transform.rotation.w));
	world2vicon.setOrigin(tf::Vector3(msg->transform.translation.x, msg->transform.translation.y, msg->transform.translation.z));

	tfbr->sendTransform(world2vicon);
}

void cam0callback(const sensor_msgs::ImageConstPtr& img){
	ROS_INFO_STREAM(img->header.frame_id);

	sensor_msgs::CameraInfo cinfo;

	cinfo.width = 752;
	cinfo.height = 480;

	cinfo.distortion_model = "plumb_bob";
	cinfo.header.frame_id = "cam0";
	cinfo.header.stamp = img->header.stamp;

	//[462.4320125192078, 0.0, 365.8042303851132, 0.0, 461.33975660880367, 246.7980144778439, 0.0, 0.0, 1.0]
	cinfo.K = {462.4320125192078, 0.0, 365.8042303851132, 0.0, 461.33975660880367, 246.7980144778439, 0.0, 0.0, 1.0};


	//[-0.2831655780347889, 0.07303323470781335, 0.00025978688200910266, -8.56982167579089e-05, 0.0]

	cinfo.D = {-0.2831655780347889, 0.07303323470781335, 0.00025978688200910266, -8.56982167579089e-05, 0.0};


	//R = [0.999997467850001, -0.0015797027152921652, 0.001602757909816239, 0.0015672116562275395, 0.9999686235917913, 0.007765029269924631, -0.0016149740588518122, -0.0077624977468274065, 0.9999685672497509]

	cinfo.R = {0.999997467850001, -0.0015797027152921652, 0.001602757909816239, 0.0015672116562275395, 0.9999686235917913, 0.007765029269924631, -0.0016149740588518122, -0.0077624977468274065, 0.9999685672497509};

	//P = [439.179921496411, 0.0, 371.79670333862305, 0.0, 0.0, 439.179921496411, 250.36704444885254, 0.0, 0.0, 0.0, 1.0, 0.0]

	cinfo.P = {439.179921496411, 0.0, 371.79670333862305, 0.0, 0.0, 439.179921496411, 250.36704444885254, 0.0, 0.0, 0.0, 1.0, 0.0};

	cam0infopub->publish(cinfo);
}

void cam1callback(const sensor_msgs::ImageConstPtr& img){
	ROS_INFO_STREAM(img->header.frame_id);

	sensor_msgs::CameraInfo cinfo;

	cinfo.width = 752;
	cinfo.height = 480;

	cinfo.distortion_model = "plumb_bob";
	cinfo.header.frame_id = "cam1";
	cinfo.header.stamp = img->header.stamp;


	/*Right:
	('D = ', [-0.28420720659643284, 0.07417445670456398, -0.00021395359716104665, -3.952578643202418e-05, 0.0])
	('K = ', [461.517209771111, 0.0, 374.0919183432524, 0.0, 460.67143556918955, 252.9759000440529, 0.0, 0.0, 1.0])
	('R = ', [0.9999543035551469, -0.004097347041567991, -0.008637276698246182, 0.004030164968665368, 0.9999616041262375, -0.007781262340210749, 0.008668827594689763, 0.007746097114211383, 0.9999324224204507])
	('P = ', [439.179921496411, 0.0, 371.79670333862305, -48.147541659059854, 0.0, 439.179921496411, 250.36704444885254, 0.0, 0.0, 0.0, 1.0, 0.0])
	('self.T ', [-0.10962555237847108, 0.00044919445930828583, 0.0009469094994889898])
	('self.R ', [0.9999440877106874, 0.002383116233399811, 0.010302534128024433, -0.002542694930602169, 0.9998765724516047, 0.015504017702544231, -0.010264314635224894, -0.015529347038720628, 0.9998267266009746])*/



	cinfo.K = {461.517209771111, 0.0, 374.0919183432524, 0.0, 460.67143556918955, 252.9759000440529, 0.0, 0.0, 1.0};

	cinfo.D = {-0.28420720659643284, 0.07417445670456398, -0.00021395359716104665, -3.952578643202418e-05, 0.0};

	cinfo.R = {0.9999543035551469, -0.004097347041567991, -0.008637276698246182, 0.004030164968665368, 0.9999616041262375, -0.007781262340210749, 0.008668827594689763, 0.007746097114211383, 0.9999324224204507};

	cinfo.P = {439.179921496411, 0.0, 371.79670333862305, -48.147541659059854, 0.0, 439.179921496411, 250.36704444885254, 0.0, 0.0, 0.0, 1.0, 0.0};

	cam1infopub->publish(cinfo);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "euroc_fixer"); // initializes ros

	ros::NodeHandle nh;

	tf::Matrix3x3 R = tf::Matrix3x3(1, 0, 0, 0, 1, 0, 0, 0, 1);
	tf::Vector3 t = tf::Vector3(0, 0, 0.05);


	//cam0
	imu2cam0.child_frame_id_ = "cam0";
	imu2cam0.frame_id_ = "imu4";

	R = tf::Matrix3x3(0.0148655429818, -0.999880929698, 0.00414029679422, 0.999557249008, 0.0149672133247, 0.025715529948, -0.0257744366974, 0.00375618835797, 0.999660727178);
	t = tf::Vector3(-0.0216401454975, -0.064676986768, 0.00981073058949);

	imu2cam0.setOrigin(t);
	imu2cam0.setBasis(R);


	//cam1
	imu2cam1.child_frame_id_ = "cam1";
	imu2cam1.frame_id_ = "imu4";

	R = tf::Matrix3x3(0.0125552670891, -0.999755099723, 0.0182237714554, 0.999598781151, 0.0130119051815, 0.0251588363115, -0.0253898008918, 0.0179005838253, 0.999517347078);
	t = tf::Vector3(-0.0198435579556, 0.0453689425024, 0.00786212447038);


	imu2cam1.setOrigin(t);
	imu2cam1.setBasis(R);


	//subs and pubs
	cam0sub = new ros::Subscriber(nh.subscribe("cam0/image_raw", 2, cam0callback));
	cam1sub = new ros::Subscriber(nh.subscribe("cam1/image_raw", 2, cam1callback));
	gtSub = new ros::Subscriber(nh.subscribe("vicon/firefly_sbx/firefly_sbx", 2, gtCallback));

	cam0infopub = new ros::Publisher(nh.advertise<sensor_msgs::CameraInfo>("cam0/camera_info", 2));
	cam1infopub = new ros::Publisher(nh.advertise<sensor_msgs::CameraInfo>("cam1/camera_info", 2));

	tfbr = new tf::TransformBroadcaster();

	ros::spin();


	return 0;
}
