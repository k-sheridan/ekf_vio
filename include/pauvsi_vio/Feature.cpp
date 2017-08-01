/*
 * Feature.cpp
 *
 *  Created on: Aug 1, 2017
 *      Author: kevin
 */

#include "Feature.h"

Feature::Feature() {
	parentFrame = NULL;
	immature = true;
}

Feature::~Feature() {
	// TODO Auto-generated destructor stub
}


bool Feature::computeObjectPositionWithPlanarApproximation(tf::Transform w2c, cv::Mat_<float> K)
{

	tf::Vector3 pixel = tf::Vector3((px.x - K(2)) / K(0), (px.y - K(5)) / K(4), 1.0);

	tf::Vector3 dir = w2c * pixel - w2c.getOrigin();

	double dt = (-w2c.getOrigin().z() / dir.z());

	if(dt <= 0)
	{
		ROS_DEBUG("removing NEW feature from planar odom because it is not on the xy plane");
		return false;
	}
	else
	{
		obj = w2c.getOrigin() + dir * dt;
		return true;
	}
}
