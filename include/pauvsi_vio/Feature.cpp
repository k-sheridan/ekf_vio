/*
 * Feature.cpp
 *
 *  Created on: Aug 1, 2017
 *      Author: kevin
 */

#include "Feature.h"

Feature::Feature() {
	parentFrame = NULL;
	point = NULL;
	immature = true;
}

Feature::~Feature() {
	// TODO Auto-generated destructor stub
}


void Feature::computeObjectPositionWithAverageSceneDepth()
{
	//TODO check if bug
	tf::Vector3 normalized = tf::Vector3((this->px.x - this->getParentFrame()->K(2)) / this->getParentFrame()->K(0), (this->px.y - this->getParentFrame()->K(5)) / this->getParentFrame()->K(4), 1.0);

	this->obj = this->getParentFrame()->getPose() * (this->getParentFrame()->getAverageFeatureDepth() * normalized); // computes the object pose at the average scene depth

}

bool Feature::computeObjectPositionWithPlanarApproximation(tf::Transform w2c, cv::Mat_<float> K)
{

	tf::Vector3 pixel = tf::Vector3((px.x - K(2)) / K(0), (px.y - K(5)) / K(4), 1.0);

	tf::Vector3 dir = w2c * pixel - w2c.getOrigin();

	double dt = (-w2c.getOrigin().z() / dir.z());

	if(dt <= 0)
	{
		ROS_FATAL("removing NEW feature from planar odom because it is not on the xy plane");
		return false;
	}
	else
	{
		tf::Vector3 obj = w2c.getOrigin() + dir * dt;


		return true;
	}
}
