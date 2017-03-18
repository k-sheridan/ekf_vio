/*
 * Point.cpp
 *
 *  Created on: Jan 30, 2017
 *      Author: pauvsi
 */


#include "Point.h"

Point::Point()
{

}

Point::Point(Feature* ft){
	observations.push_front(ft); // add this observation to the deque
}

void Point::addObservation(Feature* ft)
{
	/*
	if(observations.size() >= 1)
	{
		ROS_DEBUG_STREAM("this feature frame " << ft->frame << " last frame: " << observations.at(0)->frame);
	}*/

	observations.push_front(ft);

	/*
	if(observations.size() > 1)
	{
		ROS_DEBUG_STREAM("*most recent obs frame: " << observations.front()->frame << " last obs frame: " << observations.back()->frame);
	}*/
}


void Point::safelyDeletePoint()
{

}
