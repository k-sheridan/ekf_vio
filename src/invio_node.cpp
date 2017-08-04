/*
 * pauvsi_vio_node.cpp
 *
 *  Created on: Aug 2, 2017
 *      Author: kevin
 */



#include <ros/ros.h>

#include "../include/invio/VIO.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "invio_node"); // initializes ros

	VIO vio; // create an instance of the visual inertial odometry algorithm

	return 0;
}

