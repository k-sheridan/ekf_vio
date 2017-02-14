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

	enum PointStatus{
		TRACKING_GOOD,
		TRACKING_LOST
	};

	PointStatus status;

	std::deque<Feature*> observations; // this is  a list of observations of this 3d point from different frames

	Eigen::Vector3d pos; // this is the world coordinate of the point
	double sigma; // the variance of the point's depth

	Point();

	Point(Feature* ft);

	void addObservation(Feature* ft);

	void update(Eigen::Vector3d z, double variance);

	Eigen::Vector3d getWorldCoordinate();

	void initializePoint(Eigen::Isometry3d transform, Feature* ft, double start_depth, double start_sigma);
};




#endif /* PAUVSI_VIO_INCLUDE_PAUVSI_VIO_POINT_H_ */
