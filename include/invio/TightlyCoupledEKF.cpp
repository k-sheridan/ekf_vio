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

void TightlyCoupledEKF::addNewFeatures(std::vector<Eigen::Vector2d> new_homogenous_features){
	if(!new_homogenous_features.size()){return;}

	// add all new features to the state and adjust the current Sigma
}
