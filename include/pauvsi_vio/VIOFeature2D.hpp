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


};



#endif /* PAUVSI_M7_PAUVSI_VIO_INCLUDE_PAUVSI_VIO_VIOFEATURE2D_HPP_ */
