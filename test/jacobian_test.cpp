/*
 * jacobian_test.cpp
 *
 *  Created on: Jan 1, 2018
 *      Author: kevin
 */


#include <ros/ros.h>

#include "../include/invio/VIO.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "jacobian test"); // initializes ros

	VIO vio; // create an instance of the visual inertial odometry algorithm

	return 0;
}

