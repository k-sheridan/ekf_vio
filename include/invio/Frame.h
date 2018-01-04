/*
 * Frame.h
 *
 *  Created on: Aug 1, 2017
 *      Author: kevin
 */

#ifndef PAUVSI_VIO_INCLUDE_PAUVSI_VIO_FRAME_H_
#define PAUVSI_VIO_INCLUDE_PAUVSI_VIO_FRAME_H_

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/video.hpp"
#include "sensor_msgs/CameraInfo.h"

#include <Eigen/Core>

#include <vioParams.h>
#include <ros/ros.h>

class Frame {

public:

	Eigen::Matrix<float, 3, 3> K;

	// for now the image must be undistorted for simplicity
	Eigen::Matrix<float, 1, 5> D; // opencv distortion coefficients

	cv::Mat img;
	ros::Time t;

	Frame();
	Frame(int inv_scale, cv::Mat _img, boost::array<double, 9> k, std::vector<double> d , ros::Time _t);

	virtual ~Frame();

	bool isPixelInBox(cv::Point2f px);

};

#endif /* PAUVSI_VIO_INCLUDE_PAUVSI_VIO_FRAME_H_ */
