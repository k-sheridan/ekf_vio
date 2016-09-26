#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include "pauvsi_vio/vio.h"

VIO vio; // create an instance of the visual odometry algorithm

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	cv::Mat temp = cv_bridge::toCvShare(msg, "mono8")->image.clone();
	vio.setCurrentFrame(temp, cv_bridge::toCvCopy(msg, "mono8")->header.stamp); //set the current frame and its time created
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pauvsi_vio", ros::init_options::AnonymousName); // initializes with a randomish name
	ros::NodeHandle nh;

	//set up image transport
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber imageSub;
	imageSub = it.subscribe(vio.cameraTopic, 1, imageCallback);

	ros::spin();

	return 0;
}
