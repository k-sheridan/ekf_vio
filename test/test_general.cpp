/*
 * test_general.cpp
 *
 *  Created on: Jan 3, 2018
 *      Author: kevin
 */


#include <ros/ros.h>

#include "../include/invio/VIO.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "general_test"); // initializes ros

	Eigen::MatrixXf A;

	A = Eigen::MatrixXf::Identity(2, 2);
	A(0, 1) = 2;
	A(1, 0) = 3;

	Eigen::MatrixXf B = A;

	B.conservativeResize(4, 4);

	ROS_ASSERT(A == (B.block<2, 2>(0, 0)));

	ROS_INFO_STREAM("A: \n" << A << "\n vs B: \n" << B);

	return 0;
}

