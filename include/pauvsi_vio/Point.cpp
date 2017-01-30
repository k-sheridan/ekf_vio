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
}

Point::Point(Feature* ft){
	status = TRACKING_GOOD;
	observations.push_front(ft); // add this observation to the deque
}

void Point::addObservation(Feature* ft)
{
	//status = TRACKING_GOOD;
	observations.push_front(ft);
}
