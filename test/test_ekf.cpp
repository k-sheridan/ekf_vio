/*
 * test_general.cpp
 *
 *  Created on: Jan 3, 2018
 *      Author: kevin
 */


#include <ros/ros.h>

#include "Params.h"
#include "../include/ekf_vio/EKFVIO.h"


int main(int argc, char **argv)
{
	ros::init(argc, argv, "general_test"); // initializes ros

	ros::NodeHandle nh;


	ros::param::param<double>("~default_point_depth", DEFAULT_POINT_DEPTH, D_DEFAULT_POINT_DEPTH);
	ros::param::param<double>("~default_point_depth_variance", DEFAULT_POINT_DEPTH_VARIANCE, D_DEFAULT_POINT_DEPTH_VARIANCE);
		ros::param::param<double>("~default_point_homogenous_variance", DEFAULT_POINT_HOMOGENOUS_VARIANCE, D_DEFAULT_POINT_HOMOGENOUS_VARIANCE);
	//parseROSParams();

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


	//ROS_INFO_STREAM("test: " << tc_ekf.Sigma * tc_ekf.Sigma.selfadjointView<Eigen::Upper>() * tc_ekf.Sigma);


	ROS_INFO("test update function timing");
	ros::Time t_start;

	t_start = ros::Time::now();
	ROS_INFO("small start");
	tc_ekf.updateWithFeaturePositions(features, covs, measured);
	ROS_INFO("small stop");
	ROS_INFO_STREAM("dt: " << (ros::Time::now() - t_start).toSec() * 1000);

	tc_ekf = TightlyCoupledEKF();

	for(int i = 0; i < 100; i++){
		features.push_back(Eigen::Vector2f(0.1, 0.1));
		covs.push_back(cov);
		measured.push_back(true);
	}

	tc_ekf.addNewFeatures(features);

	t_start = ros::Time::now();
	ROS_INFO("medium start");
	tc_ekf.updateWithFeaturePositions(features, covs, measured);
	ROS_INFO("medium stop");
	ROS_INFO_STREAM("dt: " << (ros::Time::now() - t_start).toSec() * 1000);

	tc_ekf = TightlyCoupledEKF();

	for(int i = 0; i < 400; i++){
		features.push_back(Eigen::Vector2f(0.1, 0.1));
		covs.push_back(cov);
		measured.push_back(false);
	}

	tc_ekf.addNewFeatures(features);

	t_start = ros::Time::now();
	ROS_INFO("large with false start");
	tc_ekf.updateWithFeaturePositions(features, covs, measured);
	ROS_INFO("large with false stop");
	ROS_INFO_STREAM("dt: " << (ros::Time::now() - t_start).toSec() * 1000);

	tc_ekf = TightlyCoupledEKF();

	for(size_t i = 0; i < measured.size(); i++){
		measured.at(i) = true;
	}

	tc_ekf.addNewFeatures(features);

	t_start = ros::Time::now();
	ROS_INFO("large full start");
	tc_ekf.updateWithFeaturePositions(features, covs, measured);
	ROS_INFO("large full stop");
	ROS_INFO_STREAM("dt: " << (ros::Time::now() - t_start).toSec() * 1000);



	Eigen::MatrixXf test_mat(1500, 1500);
	test_mat.setIdentity();
	t_start = ros::Time::now();
	ROS_DEBUG("test sparsification large ones");
	Eigen::SparseMatrix<float> sparse_test = test_mat.sparseView();
	ROS_DEBUG("done sparsification");
	ROS_INFO_STREAM("dt: " << (ros::Time::now() - t_start).toSec() * 1000);


	//test process

	tc_ekf = TightlyCoupledEKF();

	features.clear();
	features.push_back(Eigen::Vector2f(0.1, 0.1));
	features.push_back(Eigen::Vector2f(-0.1, -0.1));
	features.push_back(Eigen::Vector2f(0.1, -0.1));

	tc_ekf.addNewFeatures(features);

	ROS_INFO_STREAM("base mu before: " << tc_ekf.base_mu);
	ROS_INFO_STREAM("feature 1 before: " << tc_ekf.features.front().getMu());

	ROS_INFO_STREAM("base_mu after: " << tc_ekf.convolveBaseState(tc_ekf.base_mu, 0.1) << "\nfeature 1 after: " << tc_ekf.convolveFeature(tc_ekf.base_mu, tc_ekf.features.front().getMu(), 0.1));

	tc_ekf.base_mu(9) = 1;

	ROS_INFO_STREAM("base mu before: " << tc_ekf.base_mu);
	ROS_INFO_STREAM("feature 1 before: " << tc_ekf.features.front().getMu());

	ROS_INFO_STREAM("base_mu after: " << tc_ekf.convolveBaseState(tc_ekf.base_mu, 0.1) << "\nfeature 1 after: " << tc_ekf.convolveFeature(tc_ekf.base_mu, tc_ekf.features.front().getMu(), 0.1));


	tc_ekf.base_mu(10) = 3.14;

	ROS_INFO_STREAM("base mu before: " << tc_ekf.base_mu);
	ROS_INFO_STREAM("feature 1 before: " << tc_ekf.features.front().getMu());

	ROS_INFO_STREAM("base_mu after: " << tc_ekf.convolveBaseState(tc_ekf.base_mu, 0.1) << "\nfeature 1 after: " << tc_ekf.convolveFeature(tc_ekf.base_mu, tc_ekf.features.front().getMu(), 0.1));

	tc_ekf.base_mu(10) = 0;
	tc_ekf.base_mu(12) = 3.14;

	ROS_INFO_STREAM("base mu before: " << tc_ekf.base_mu);
	ROS_INFO_STREAM("feature 1 before: " << tc_ekf.features.front().getMu());

	ROS_INFO_STREAM("base_mu after: " << tc_ekf.convolveBaseState(tc_ekf.base_mu, 0.1) << "\nfeature 1 after: " << tc_ekf.convolveFeature(tc_ekf.base_mu, tc_ekf.features.front().getMu(), 0.1));


	tc_ekf.base_mu(12) = 0;
	tc_ekf.base_mu(11) = -3.1415;
	tc_ekf.base_mu(9) = 1;
	tc_ekf.base_mu(7) = 1;
	tc_ekf.features.front().getMu()(0) = 0;
	tc_ekf.features.front().getMu()(1) = 0;

	ROS_INFO_STREAM("base mu before: " << tc_ekf.base_mu);
	ROS_INFO_STREAM("feature 1 before: " << tc_ekf.features.front().getMu());

	ROS_INFO_STREAM("base_mu after: " << tc_ekf.convolveBaseState(tc_ekf.base_mu, 0.5) << "\nfeature 1 after: " << tc_ekf.convolveFeature(tc_ekf.base_mu, tc_ekf.features.front().getMu(), 0.5));


	ROS_INFO_STREAM("process noise dt=0.1: " << tc_ekf.generateProcessNoise(0.1));

	

	return 0;
}

