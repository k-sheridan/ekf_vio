/*
 * pauvsi_vio_node.cpp
 *
 *  Created on: Aug 2, 2017
 *      Author: kevin
 */



#include <ros/ros.h>
#include <pauvsi_vio/VIO.h>

//#define ROSCONSOLE_MIN_SEVERITY ROSCONSOLE_SEVERITY_INFO

VIO vio; // create an instance of the visual inertial odometry algorithm

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pauvsi_vio"); // initializes ros

	//start the callbacks when messages are ready
	ros::spin();

	return 0;
}

