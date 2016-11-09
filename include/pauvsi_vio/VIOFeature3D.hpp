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

#include "VIOLine.hpp"

#define RANSAC_ITERATIONS 100

class VIOFeature3D
{
private:
	cv::Mat feature_description;
	std::vector<double> position;

	//int stamina; // if the feature has lasted for a long time it gets stamina
	//int armor; // how resistant the point is to taking health damage
	//int agility; // how well the feature can move
	//int charisma; // how influential the feature is on the pose
	int health; // a quantified representation of how good this feature is.
	//int strength; // how much force the feature can put on the camera
	//int intelligence; // how man skills the feature has
	//int morale; // if the morale is low the health will not increase at a normal levels

	cv::Scalar color; // the color of this point for visualization

	int current2DFeatureMatchIndex;
	int current2DFeatureMatchID;

public:

	VIOFeature3D(){

	}

};



#endif /* PAUVSI_M7_PAUVSI_VIO_INCLUDE_PAUVSI_VIO_VIOFEATURE3D_HPP_ */
