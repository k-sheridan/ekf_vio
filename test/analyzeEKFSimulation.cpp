#include <ros/ros.h>

#include "Params.h"
#include "../include/invio/VIO.h"

void simulateAndVisualizeEKF(int feature_count, float depth_sigma, float depth_mu, Eigen::Vector3f vel, Eigen::Vector3f accel, Eigen::Vector3f omega){

}

void generateFakeMeasurementsAndUpdateEKF(TightlyCoupledEKF& tc_ekf, std::vector<Eigen::Vector3f> point_pos, float dt, Eigen::Vector3f pos, Eigen::Quaternionf quat){
	
	std::vector<Eigen::Vector2f> feature_pos_vec;
	std::vector<Eigen::Matrix2f> covs;
	std::vector<bool> measured;

	Eigen::Matrix2f cov;
	cov << 0.00001, 0, 0, 0.00001;

	for(auto e : point_pos){
		Eigen::Vector3f fp;

		fp = quat.inverse()*e - quat.inverse()*pos;

		// add a 2d observation
		feature_pos_vec.push_back(Eigen::Vector2f(fp.x() / fp.z(), fp.y() / fp.z());
		covs.push_back(cov);
		measured.push_back(true);
	}

	tc_ekf.updateWithFeaturePositions(feature_pos_vec, covs, measured); // perform update 

}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "general_test"); // initializes ros

	ros::NodeHandle nh;


	ros::param::param<double>("~default_point_depth", DEFAULT_POINT_DEPTH, D_DEFAULT_POINT_DEPTH);
	//parseROSParams();


	//simulate a moving camera and make the points converge (ideal scenario)
	simulateAndVisualizeEKF(30, 0.000001, 0.5, Eigen::Vector3f(0.5, 0, 0), Eigen::Vector3f(0, 0, 0), Eigen::Vector3f(0, 0, 0));

	return 0;
}