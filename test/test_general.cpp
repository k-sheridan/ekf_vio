/*
 * test_general.cpp
 *
 *  Created on: Jan 3, 2018
 *      Author: kevin
 */


#include <ros/ros.h>

#include "vioParams.h"
#include "../include/invio/VIO.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "general_test"); // initializes ros

	parseROSParams();

	Eigen::MatrixXf A;

	A = Eigen::MatrixXf::Identity(2, 2);
	A(0, 1) = 2;
	A(1, 0) = 3;

	Eigen::MatrixXf B = A;

	B.conservativeResize(4, 4);

	ROS_ASSERT(A == (B.block<2, 2>(0, 0)));
	ROS_INFO_STREAM("A: \n" << A << "\n vs B: \n" << B);

	// test basic ekf functionality

	TightlyCoupledEKF tc_ekf;

	std::vector<Eigen::Vector2f> features;
	features.push_back(Eigen::Vector2f(0.1, 0.1));
	features.push_back(Eigen::Vector2f(-0.1, -0.1));
	features.push_back(Eigen::Vector2f(0.1, -0.1));

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


	//test update method
	std::vector<Eigen::Matrix2f> covs;
	Eigen::Matrix2f cov;
	cov << 0.001, 0, 0, 0.001;
	covs.push_back(cov);
	covs.push_back(cov);
	covs.push_back(cov);


	// perform update
	ROS_INFO_STREAM("base_mu before: " << tc_ekf.base_mu);
	ROS_INFO_STREAM("cov before: " << tc_ekf.Sigma);

	tc_ekf.updateWithFeaturePositions(features, covs, measured);

	ROS_INFO_STREAM("base_mu after: " << tc_ekf.base_mu);
	ROS_INFO_STREAM("cov after: " << tc_ekf.Sigma);



	return 0;
}

