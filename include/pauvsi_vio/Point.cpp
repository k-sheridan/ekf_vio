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
}

Point::Point(Feature* ft){
	_observations.push_front(ft); // add this observation to the deque
	deleted = false;
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

	deleted = true;

	/*
	for(auto e : this->observations)
	{
		ROS_ASSERT(e != NULL);
		e->point = NULL;
	}*/

	ROS_DEBUG_STREAM("point deleting itself");

	//peace out delete my self
	// we had a good run
	this->theMap->erase(this->thisPoint);

	ROS_DEBUG("point deleted");
}
