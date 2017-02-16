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
	this->theMap == NULL;
}

Point::Point(Feature* ft){
	status = TRACKING_GOOD;
	observations.push_front(ft); // add this observation to the deque
	sigma = 1000; // starting depth certainty
	this->theMap == NULL;
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
void Point::initializePoint(tf::Transform transform, Feature* ft, double start_depth, double start_sigma)
{
	Eigen::Vector3d eig_dir = (start_depth * ft->getDirectionVector());
	tf::Vector3 dir_vec = tf::Vector3(eig_dir(0), eig_dir(1), eig_dir(2));
	tf::Vector3 world_point = transform * dir_vec;

	this->pos(0) = world_point.getX();
	this->pos(1) = world_point.getY();
	this->pos(2) = world_point.getZ();

	this->sigma = start_sigma;
}

void Point::safelyDelete(){
	//first nullify all references to me
	ROS_ASSERT(this->thisPoint->pos == this->pos);


	ROS_DEBUG_STREAM("point deleting itself");

	//peace out delete my self
	// we had a good run
	this->theMap->erase(this->thisPoint);
	ROS_DEBUG_STREAM("I deleted myself");
}
