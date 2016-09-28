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

class VIOFeature2D
{

private:
	unsigned int id;
	cv::KeyPoint fast_corner;
	cv::Point2f position;
	cv::Mat description;
	bool described;

public:
	/*
	 * creates a feature
	 * without description
	 */
	VIOFeature2D(cv::KeyPoint corner, int _id){
		fast_corner = corner;
		position = corner.pt;
		id = _id;
		described = false; // the feature has not been described with this constructor
	}

	/*
	 * creates a feature with a description
	 */
	VIOFeature2D(cv::KeyPoint corner, cv::Mat _description, int _id){
		fast_corner = corner;
		position = corner.pt;
		id = _id;
		description = _description;
		described = true; // the feature has not been described with this constructor
	}

	cv::KeyPoint getFASTCorner(){
		return fast_corner;
	}

	cv::Point2f getFeaturePosition(){
		return position;
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
	void setFASTCorner(cv::KeyPoint kp){
		fast_corner = kp;
		position = kp.pt;
	}

	void setFeaturePosition(cv::Point2f pt){
		position = pt;
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
