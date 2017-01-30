/*
 * Feature.h
 *
 *  Created on: Jan 29, 2017
 *      Author: pauvsi
 */

#ifndef PAUVSI_VIO_INCLUDE_PAUVSI_VIO_FEATURE_H_
#define PAUVSI_VIO_INCLUDE_PAUVSI_VIO_FEATURE_H_

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/video.hpp"
#include <vector>
#include <deque>
#include <string>
#include <ros/ros.h>
#include <eigen3/Eigen/Geometry>

#include "Frame.h"

class Point;

class Frame;

class Feature{

public:

	Frame* frame; // this points to the frame from which this feature was extracted

	cv::KeyPoint feature; // response = quality

	int id;

	cv::Point2f undistort_pxl; // this is the undistorted normal pixel location of this feature
	cv::Point2f original_pxl; // the original pixel location

	//temporary variables
	float radius; // the radius of te feature from the center of the image
	float quality; // this quality is for the ranking process

	Point* point;

	cv::Mat description;

	//flags
	bool described; // is this feature described
	bool undistorted; // is this feature undistorted
	bool set; // says whether the feature was set


	Feature();

	Feature(Frame* _frame, cv::Point2f px, Point* pt, int id = -1);

	void undistort(cv::Mat K, cv::Mat D);

	Eigen::Vector3d getDirectionVector();


};


#endif /* PAUVSI_VIO_INCLUDE_PAUVSI_VIO_FEATURE_H_ */
