/*
 * jacobian_test.cpp
 *
 *  Created on: Jan 1, 2018
 *      Author: kevin
 */


#include <ros/ros.h>

#include "../include/ekf_vio/EKFVIO.h"
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

	ROS_INFO_STREAM("F , with dt = 0: " << tc_ekf.numericallyLinearizeProcess(tc_ekf.base_mu, tc_ekf.features, 0.0));

	// try with x omega of pi
	tc_ekf.base_mu(10) = 3.1415;

	ROS_INFO_STREAM("F , omega_x=pi: " << tc_ekf.numericallyLinearizeProcess(tc_ekf.base_mu, tc_ekf.features, 0.1));

	tc_ekf.base_mu(7) = 1;

	ROS_INFO_STREAM("F , omega_x=pi, b_dx=1: " << tc_ekf.numericallyLinearizeProcess(tc_ekf.base_mu, tc_ekf.features, 0.1));

	ROS_INFO_STREAM("F , omega_x=pi, b_dx=1 with dt = 0: " << tc_ekf.numericallyLinearizeProcess(tc_ekf.base_mu, tc_ekf.features, 0.0));


	ros::Time start = ros::Time::now();
	tc_ekf.numericallyLinearizeProcess(tc_ekf.base_mu, tc_ekf.features, 0.1);
	ROS_INFO_STREAM("computed F for 3 feats in: " << (ros::Time::now() - start).toSec() * 1000);

	// add 96 more features
	for(int i = 0; i < 32; i++)
	{
		tc_ekf.addNewFeatures(features);
	}

	start = ros::Time::now();
	tc_ekf.numericallyLinearizeProcess(tc_ekf.base_mu, tc_ekf.features, 0.1);
	ROS_INFO_STREAM("computed F for 100 feats in: " << (ros::Time::now() - start).toSec() * 1000);

	// add 400 more features
	for(int i = 0; i < 134; i++)
	{
		tc_ekf.addNewFeatures(features);
	}

	start = ros::Time::now();
	tc_ekf.numericallyLinearizeProcess(tc_ekf.base_mu, tc_ekf.features, 0.1);
	ROS_INFO_STREAM("computed F for 500 feats in: " << (ros::Time::now() - start).toSec() * 1000);

	return 0;
}

