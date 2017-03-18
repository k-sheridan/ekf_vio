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

class Feature;

class Point{

public:

	std::deque<Feature*> observations; // this is  a list of observations of this 3d point from different frames

	Point();

	Point(Feature* ft);

	void addObservation(Feature* ft);

	void safelyDeletePoint();
};




#endif /* PAUVSI_VIO_INCLUDE_PAUVSI_VIO_POINT_H_ */
