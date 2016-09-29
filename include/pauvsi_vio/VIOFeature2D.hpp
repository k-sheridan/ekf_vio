/*
 * Feature2D.h
 *
 *  Created on: Sep 28, 2016
 *      Author: kevinsheridan
 */

#ifndef PAUVSI_M7_PAUVSI_VIO_INCLUDE_PAUVSI_VIO_VIOFEATURE2D_HPP_
#define PAUVSI_M7_PAUVSI_VIO_INCLUDE_PAUVSI_VIO_VIOFEATURE2D_HPP_

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

#define DEAFAULT_FEATURE_SIZE 1.0

class VIOFeature2D
{

private:
	int id;
	cv::KeyPoint feature;
	cv::Mat description;
	bool described;
	bool matched;
	int matchedFeatureFromLastFrameIndex;
	int matchedFeatureFromLastFrameID;

public:

	/*
	 * create a feature with a position and match
	 */
	VIOFeature2D(cv::Point2f pt, int matchedID, int matchedIndex, int _id){
		this->setFeaturePosition(pt);
		id = _id;
		described = false; // the feature has not been described with this constructor
		matched = true;
		matchedFeatureFromLastFrameID = matchedID;
		matchedFeatureFromLastFrameIndex = matchedIndex;
	}
	/*
	 * create a feature with a position, description and match
	 */
	VIOFeature2D(cv::Point2f pt, int matchedID, int matchedIndex, cv::Mat desc, int _id){
		this->setFeaturePosition(pt);
		id = _id;
		described = true; // the feature has been described with this constructor
		description = desc;
		matched = true;
		matchedFeatureFromLastFrameID = matchedID;
		matchedFeatureFromLastFrameIndex = matchedIndex;
	}
	/*
	 * creates a feature
	 * without description
	 * not matched
	 */
	VIOFeature2D(cv::KeyPoint _corner, int _id){
		feature = _corner;
		id = _id;
		described = false; // the feature has not been described with this constructor
		matched = false;
		matchedFeatureFromLastFrameID = -1;
		matchedFeatureFromLastFrameIndex = -1;
	}

	/*
	 * creates a feature with a description
	 * not matched
	 */
	VIOFeature2D(cv::KeyPoint _corner, cv::Mat _description, int _id){
		feature = _corner;
		id = _id;
		description = _description;
		described = true; // the feature has not been described with this constructor
		matched = false;
		matchedFeatureFromLastFrameID = -1;
		matchedFeatureFromLastFrameIndex = -1;
	}

	cv::KeyPoint getFeature(){
		return this->feature;
	}

	cv::Point2f getFeaturePosition(){
		return this->feature.pt;
	}

	cv::Mat getFeatureDescription(){
		return description;
	}

	bool isFeatureDescribed(){
		return described;
	}

	int getFeatureID(){
		return id;
	}

	/*
	 * sets the corner and position of feature
	 */
	void setFeature(cv::KeyPoint kp){
		feature = kp;
	}

	void setFeaturePosition(cv::Point2f pt){
		feature = cv::KeyPoint(pt, DEAFAULT_FEATURE_SIZE);
	}

	/*
	 * sets the feature by creating it with a szie and pos
	 */
	void setFeature(cv::Point2f pt, float size){
		feature = cv::KeyPoint(pt, size);
	}

	/*
	 * sets the description and sets the feature to described
	 */
	void setFeatureDescription(cv::Mat desc){
		description = desc;
		described = true;
	}

	void setFeatureID(int _id){
		id = _id;
	}

};



#endif /* PAUVSI_M7_PAUVSI_VIO_INCLUDE_PAUVSI_VIO_VIOFEATURE2D_HPP_ */
