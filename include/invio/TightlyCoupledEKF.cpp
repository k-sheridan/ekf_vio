/*
 * TightlyCoupledEKF.cpp
 *
 *  Created on: Nov 25, 2017
 *      Author: kevin
 */

#include <TightlyCoupledEKF.h>

TightlyCoupledEKF::TightlyCoupledEKF() {

	this->Sigma.resize(BASE_STATE_SIZE, BASE_STATE_SIZE);

	// TODO setup initial variances and values of the base state

}

TightlyCoupledEKF::~TightlyCoupledEKF() {
	// TODO Auto-generated destructor stub
}

void TightlyCoupledEKF::addNewFeatures(std::vector<Eigen::Vector2f> new_homogenous_features){
	if(!new_homogenous_features.size()){return;}

	//resize the covariance matrix without changing other values
	int new_size = BASE_STATE_SIZE + this->features.size() * 3 + new_homogenous_features.size() * 3; // each feature has 3 degrees of freedom
	this->Sigma.conservativeResize(new_size, new_size);

	//TODO compute the average depth in the scene
	float average_scene_depth = DEFAULT_POINT_DEPTH;

	// add all new features to the state and adjust the current Sigma
	int starting_index = BASE_STATE_SIZE + this->features.size() * 3;
	for(auto e : new_homogenous_features){
		this->features.push_back(Feature(e, average_scene_depth)); // add a point with an estimated depth

		//initialize this point's uncertainties
		this->Sigma(starting_index, starting_index) = DEFAULT_POINT_HOMOGENOUS_VARIANCE;
		starting_index++;
		this->Sigma(starting_index, starting_index) = DEFAULT_POINT_HOMOGENOUS_VARIANCE;
		starting_index++;
		this->Sigma(starting_index, starting_index) = DEFAULT_POINT_DEPTH_VARIANCE; // high uncertainty for the depth of the feature
		starting_index++;
	}
}

std::vector<Eigen::Vector2f> TightlyCoupledEKF::previousFeaturePositionVector(){
	std::vector<Eigen::Vector2f> output;
	//output.reserve(this->features.size());
	for(auto e : this->features){
		output.push_back(e.getLastResultFromKLTTracker());
	}

	return output;
}

void TightlyCoupledEKF::updateWithFeaturePositions(std::vector<Eigen::Vector2f> measured_positions, std::vector<Eigen::Matrix2f> estimated_covariance, std::vector<bool> pass)
{
	ROS_DEBUG_STREAM(measured_positions.size() <<" , "<< estimated_covariance.size() <<" , "<< pass.size() <<" , "<< this->features.size());
	ROS_ASSERT(measured_positions.size() == estimated_covariance.size() && pass.size() == this->features.size() && estimated_covariance.size() == pass.size()); // make sure that there are enough features

	//ROS_ASSERT(this->Sigma.rows() == this->Sigma.cols() && this->Sigma.cols() == BASE_STATE_SIZE + this->features.size());

	//TODO actually update the whole state
	int i = 0; // track the index
	for(auto& e : this->features){

		if(pass.at(i))
		{
			if(!e.flaggedForDeletion())
			{
				//ROS_DEBUG_STREAM(measured_positions.at(i));
				e.setLastResultFromKLTTracker(measured_positions.at(i)); // fix
				e.setNormalizedPixel(measured_positions.at(i));
			}
		}
		else
		{
			ROS_DEBUG("fail");
			e.setDeleteFlag(true);
		}

		//increment
		i++;
	}


}

/*
 * form the mapping between our measurement of feature positions (homogenous) and the state
 * the measured vector is used to tell this function which features were not observed in this measurement
 */
Eigen::SparseMatrix<float> TightlyCoupledEKF::formFeatureMeasurementMap(std::vector<bool> measured){

	ROS_ASSERT(measured.size() == this->features.size()); // sanity check

}

Eigen::Matrix2f TightlyCoupledEKF::getFeatureHomogenousCovariance(int index){
	int start = BASE_STATE_SIZE + index * 3;
	return this->Sigma.block<2, 2>(start, start);
}

float TightlyCoupledEKF::getFeatureDepthVariance(int index){
	int start = BASE_STATE_SIZE + index * 3 + 2;
	return this->Sigma(start, start);
}

Eigen::Matrix2f TightlyCoupledEKF::getMetric2PixelMap(Eigen::Matrix3f& K){
	Eigen::Matrix2f J;
	J << K(0, 0), 0, 0, K(1, 1);
	return J;
}

