/*
 * TightlyCoupledEKF.cpp
 *
 *  Created on: Nov 25, 2017
 *      Author: kevin
 */

#include <TightlyCoupledEKF.h>

TightlyCoupledEKF::TightlyCoupledEKF() {
	// TODO Auto-generated constructor stub

}

TightlyCoupledEKF::~TightlyCoupledEKF() {
	// TODO Auto-generated destructor stub
}

void TightlyCoupledEKF::addNewFeatures(std::vector<Eigen::Vector2f> new_homogenous_features){
	if(!new_homogenous_features.size()){return;}

	// add all new features to the state and adjust the current Sigma
}

std::vector<Eigen::Vector2f> TightlyCoupledEKF::previousFeaturePositionVector(){
	std::vector<Eigen::Vector2f> output;
	output.reserve(this->features.size());
	for(auto e : this->features){
		output.push_back(e.getLastResultFromKLTTracker());
	}
}

void TightlyCoupledEKF::updateWithFeaturePositions(std::vector<Eigen::Vector2f> measured_positions, std::vector<Eigen::Matrix2f> estimated_covariance, std::vector<bool> pass)
{
	ROS_ASSERT(measured_positions.size() == estimated_covariance.size() == pass.size() == this->features.size()); // make sure that there are enough features

	//TODO actually update the state


}
