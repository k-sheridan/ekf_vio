/*
 * Point.h
 *
 *  Created on: Jan 29, 2017
 *      Author: pauvsi
 */

#ifndef PAUVSI_VIO_INCLUDE_PAUVSI_VIO_POINT_H_
#define PAUVSI_VIO_INCLUDE_PAUVSI_VIO_POINT_H_

#include <eigen3/Eigen/Geometry>

#include "Feature.h"


class Point{

public:

	enum PointStatus{
		TRACKING_GOOD,
		TRACKING_LOST
	};

	PointStatus status;

	std::deque<Feature*> observations; // this is  a list of observations of this 3d point from different frames

	Point()
	{
		status = TRACKING_GOOD;
	}

	Point(Feature* ft){
		status = TRACKING_GOOD;
		observations.push_front(ft); // add this observation to the deque
	}

	void addObservation(Feature* ft)
	{
		//status = TRACKING_GOOD;
		observations.push_front(ft);
	}
};




#endif /* PAUVSI_VIO_INCLUDE_PAUVSI_VIO_POINT_H_ */
