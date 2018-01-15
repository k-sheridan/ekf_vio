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

	TightlyCoupledEKF tc_ekf;

	std::vector<Eigen::Vector2f> features;
	features.push_back(Eigen::Vector2f(1, 1));
	features.push_back(Eigen::Vector2f(-1, -1));
	features.push_back(Eigen::Vector2f(1, -1));

	tc_ekf.addNewFeatures(features);

	std::vector<bool> measured;
	measured.push_back(true);
	measured.push_back(false);
	measured.push_back(true);

	ROS_INFO_STREAM("feature H: " << tc_ekf.formFeatureMeasurementMap(measured));
	Eigen::SparseMatrix<float> H_answer(4, 22 + 9);
	H_answer.insert(0, 22)=1.0;
	H_answer.insert(1, 23)=1.0;
	H_answer.insert(2, 28)=1.0;
	H_answer.insert(3, 29)=1.0;

	ROS_ASSERT(tc_ekf.formFeatureMeasurementMap(measured).toDense() == H_answer.toDense());

	return 0;
}

