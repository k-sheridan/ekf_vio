/*
 * Point.cpp
 *
 *  Created on: Jan 30, 2017
 *      Author: pauvsi
 */


#include "Point.h"

Point::Point()
{
	deleted = false;
	this->sigma = DEFAULT_STARTING_SIGMA;
}

Point::Point(Feature* ft){
	_observations.push_front(ft); // add this observation to the deque
	deleted = false;
	this->sigma = DEFAULT_STARTING_SIGMA;
}

void Point::addObservation(Feature* ft)
{
	/*
	if(observations.size() >= 1)
	{
		ROS_DEBUG_STREAM("this feature frame " << ft->frame << " last frame: " << observations.at(0)->frame);
	}*/

	_observations.push_front(ft);

	/*
	if(observations.size() > 1)
	{
		ROS_DEBUG_STREAM("*most recent obs frame: " << observations.front()->frame << " last obs frame: " << observations.back()->frame);
	}*/
}


void Point::safelyDeletePoint()
{
	ROS_DEBUG_STREAM("deleting point with " << this->observations().size() << " obs");


	/*for(auto e : this->observations())
	{
		ROS_ASSERT(e != NULL);
		ROS_ASSERT(e->frame != NULL);
		if(e != NULL && e->frame->isKeyframe)
		{
			ROS_ASSERT(e->frame->linkedKeyframe != NULL);
			e->frame->linkedKeyframe->numFeatures--; // decrement the feature counter in the keyframe to say that this frame has one less linked feature
		}
	}*/

	ROS_DEBUG_STREAM("point deleting itself");

	//peace out delete my self
	// we had a good run
	this->theMap->erase(this->thisPoint);

	deleted = true;

	ROS_DEBUG("point deleted");
}

/*
 * please give the transform from camera coord to world coord
 */
void Point::initializePoint(tf::Transform transform, Feature* ft, double start_depth)
{
	Eigen::Vector3d eig_dir = (start_depth * ft->getDirectionVector());
	tf::Vector3 dir_vec = tf::Vector3(eig_dir(0), eig_dir(1), eig_dir(2));
	tf::Vector3 world_point = transform * dir_vec;

	this->pos(0) = world_point.getX();
	this->pos(1) = world_point.getY();
	this->pos(2) = world_point.getZ();

	//this->sigma = start_sigma;

	//this->_initialized = true;
}
