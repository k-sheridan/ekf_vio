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

void Feature::computeBorderWeight()
{
	ROS_ASSERT(this->parentFrame != NULL);
	this->border_weight = -std::pow(std::max(std::fabs((this->px.x - this->parentFrame->undistorted_width / 2.0) / this->parentFrame->undistorted_width),
			std::fabs((this->px.y - this->parentFrame->undistorted_height / 2.0) / this->parentFrame->undistorted_height)), BORDER_WEIGHT_EXPONENT) + 1.0;
}

