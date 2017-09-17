/*
 * Feature.cpp
 *
 *  Created on: Aug 1, 2017
 *      Author: kevin
 */

#include "../invio/Feature.h"

Feature::Feature() {
	parentFrame = NULL;
	point = NULL;
	obsolete = false;
}

Feature::~Feature() {
	// TODO Auto-generated destructor stub
}

Eigen::Vector3d Feature::getHomogenousCoord()
{
	return Eigen::Vector3d((px.x - getParentFrame()->K(2)) / getParentFrame()->K(0),
			(px.y - getParentFrame()->K(5)) / getParentFrame()->K(4), 1.0);
}

Eigen::Vector2d Feature::getMetricPixel() {
	ROS_ASSERT(!obsolete);
	return Eigen::Vector2d(
			(px.x - getParentFrame()->K(2)) / getParentFrame()->K(0),
			(px.y - getParentFrame()->K(5)) / getParentFrame()->K(4));
}


