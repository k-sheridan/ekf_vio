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
#include "KeyFrame.h"

#define DEFAULT_STARTING_SIGMA 1000

class Feature;

class Point{


private:

	bool deleted;

	double sigma; // the certainty of this point's depth
	Eigen::Vector3d pos; // this is the world coordinate of the point

	std::deque<Feature*> _observations; // this is  a list of observations of this 3d point from different frames


public:

	std::list<Point>::iterator thisPoint; // the iterator of this point in the map
	std::list<Point>* theMap; // a pointer to the map which this point is stored in

	Point();

	Point(Feature* ft);

	void addObservation(Feature* ft);

	std::deque<Feature*>& observations(){
		ROS_ASSERT(!deleted);
		return this->_observations;
	}

	Eigen::Vector3d getWorldCoordinate()
	{
		ROS_ASSERT(!deleted);
		return this->pos;
	}

	bool isDeleted()
	{
		return deleted;
	}

	void safelyDeletePoint();

	void initializePoint(tf::Transform transform, Feature* ft, double start_depth);
};




#endif /* PAUVSI_VIO_INCLUDE_PAUVSI_VIO_POINT_H_ */
