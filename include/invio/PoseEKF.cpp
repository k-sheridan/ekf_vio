/*
 * PoseEKF.cpp
 *
 *  Created on: Oct 31, 2017
 *      Author: kevin
 */

#include <PoseEKF.h>

PoseEKF::PoseEKF() {
	// TODO Auto-generated constructor stub
	this->resetState();
}

PoseEKF::PoseEKF(ros::Time start)
{
	this->t = start;
	this->resetState();
}

PoseEKF::~PoseEKF() {
	// TODO Auto-generated destructor stub
}

void PoseEKF::resetState()
{

}

