/*
 * VIOFeature3D.hpp
 *
 *  Created on: Sep 28, 2016
 *      Author: kevinsheridan
 */

#ifndef PAUVSI_M7_PAUVSI_VIO_INCLUDE_PAUVSI_VIO_VIOFEATURE3D_HPP_
#define PAUVSI_M7_PAUVSI_VIO_INCLUDE_PAUVSI_VIO_VIOFEATURE3D_HPP_

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/video.hpp"
#include <vector>
#include <string>
#include <ros/ros.h>
#include <eigen3/Eigen/Geometry>

//#include "VIOLine.hpp"

#define PROCESS_ERROR 0.0

/*
 * subfeature is a 3d point and certainty associated with a 3d feature in space
 * a new subfeature is added each time a 3d point is triangulated from a delta r in the state.
 */
struct SubFeature{
	Eigen::Vector3d position;
	double certianty;
};

class VIOFeature3D
{

public:
	double variance; // this is how certain we are of this 3d points location
	Eigen::Vector3d position;

	//int stamina; // if the feature has lasted for a long time it gets stamina
	//int armor; // how resistant the point is to taking health damage
	//int agility; // how well the feature can move
	//int charisma; // how influential the feature is on the pose
	//int health; // a quantified representation of how good this feature is.
	//int strength; // how much force the feature can put on the camera
	//int intelligence; // how man skills the feature has
	//int morale; // if the morale is low the health will not increase at a normal levels

	bool colorSet;
	cv::Scalar color; // the color of this point for visualization

	int current2DFeatureMatchIndex;
	int current2DFeatureMatchID;

	VIOFeature3D(){

	}

	/*
	 * NOTE: this should start with a really high variance
	 */
	VIOFeature3D(int matchedFeatureIndex, int matchedFeatureID, cv::Scalar color, double variance, Eigen::Vector3d position){
		this->position = position;
		this->variance = variance;
		this->color = color;
		this->colorSet = true;
		this->current2DFeatureMatchID = matchedFeatureID;
		this->current2DFeatureMatchIndex = matchedFeatureIndex;
	}

	/*
	 * run a very simple 1d kalman filter to update the point's 3d position
	 * variance: 0 is perfect. infinity is worst.
	 */
	void update(Eigen::Vector3d measurement, double variance)
	{
		//this->process(); // process the previous state
		ROS_ASSERT(this->variance > 0 || variance > 0);
		double K = this->variance / (this->variance + variance);

		this->position(0) = this->position(0) + K * (measurement(0) - this->position(0));
		this->position(1) = this->position(1) + K * (measurement(1) - this->position(1));
		this->position(2) = this->position(2) + K * (measurement(2) - this->position(2));

		this->variance = (1 - K) * this->variance;
	}



	void process()
	{
		this->variance += PROCESS_ERROR;
	}

};



#endif /* PAUVSI_M7_PAUVSI_VIO_INCLUDE_PAUVSI_VIO_VIOFEATURE3D_HPP_ */
