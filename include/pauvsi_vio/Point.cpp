/*
 * Point.cpp
 *
 *  Created on: Jan 30, 2017
 *      Author: pauvsi
 */


#include "Point.h"

Point::Point()
{
	status = TRACKING_GOOD;
	sigma = 1000; // starting depth certainty
}

Point::Point(Feature* ft){
	status = TRACKING_GOOD;
	observations.push_front(ft); // add this observation to the deque
	sigma = 1000; // starting depth certainty
}

void Point::addObservation(Feature* ft)
{
	/*
	if(observations.size() >= 1)
	{
		ROS_DEBUG_STREAM("this feature frame " << ft->frame << " last frame: " << observations.at(0)->frame);
	}*/

	status = TRACKING_GOOD;
	observations.push_front(ft);

	/*
	if(observations.size() > 1)
	{
		ROS_DEBUG_STREAM("*most recent obs frame: " << observations.front()->frame << " last obs frame: " << observations.back()->frame);
	}*/
}


void Point::update(Eigen::Vector3d z, double variance)
{
	ROS_ASSERT(this->sigma > 0 || variance > 0);
	double K = this->sigma / (this->sigma + variance);

	this->pos(0) = this->pos(0) + K * (z(0) - this->pos(0));
	this->pos(1) = this->pos(1) + K * (z(1) - this->pos(1));
	this->pos(2) = this->pos(2) + K * (z(2) - this->pos(2));

	this->sigma = (1-K) * this->sigma;
}

Eigen::Vector3d Point::getWorldCoordinate()
{
	return this->pos;
}

/*
 * please give the transform from camera coord to world coord
 */
void Point::initializePoint(Eigen::Isometry3d transform, Feature* ft, double start_depth, double start_sigma)
{
	this->pos = transform * (start_depth * ft->getDirectionVector());
	this->sigma = start_sigma;
}
