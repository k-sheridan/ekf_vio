#include <ros/ros.h>
#include "pauvsi_vio/vio.h"


int main(int argc, char **argv)
{
	ros::init(argc, argv, "pauvsi_vio", ros::init_options::AnonymousName); // initializes with a randomish name

	VIO vio; // create an instance of the visual odometry algorithm

	//start the callbacks when messages are ready
	ros::spin();

	return 0;
}
