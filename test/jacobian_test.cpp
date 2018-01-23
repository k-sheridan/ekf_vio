/*
 * jacobian_test.cpp
 *
 *  Created on: Jan 1, 2018
 *      Author: kevin
 */


#include <ros/ros.h>

#include "../include/invio/VIO.h"
#include "Params.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "jacobian_test"); // initializes ros


	ros::NodeHandle nh;


	ros::param::param<double>("~default_point_depth", DEFAULT_POINT_DEPTH, D_DEFAULT_POINT_DEPTH);


	TightlyCoupledEKF tc_ekf = TightlyCoupledEKF();

	std::vector<Eigen::Vector2f> features;
	features.push_back(Eigen::Vector2f(0.1, 0.1));
	features.push_back(Eigen::Vector2f(-0.1, -0.1));
	features.push_back(Eigen::Vector2f(0.1, -0.1));

	tc_ekf.addNewFeatures(features);

	ROS_INFO_STREAM("F for starting state: " << tc_ekf.numericallyLinearizeProcess(tc_ekf.base_mu, tc_ekf.features, 0.1));

	return 0;
}

