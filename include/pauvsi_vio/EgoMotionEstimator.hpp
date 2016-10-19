/*
 * EgoMotionEstimator.h
 *
 *  Created on: Oct 18, 2016
 *      Author: kevinsheridan
 */

#ifndef PAUVSI_M7_PAUVSI_VIO_INCLUDE_PAUVSI_VIO_EGOMOTIONESTIMATOR_HPP_
#define PAUVSI_M7_PAUVSI_VIO_INCLUDE_PAUVSI_VIO_EGOMOTIONESTIMATOR_HPP_

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
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_broadcaster.h>

#include "Frame.hpp"
#include "VIOFeature3D.hpp"
#include "VIOFeature2D.hpp"

class EgoMotionEstimator {
public:
	EgoMotionEstimator(){

	}

	bool estimateEgoMotion(Frame frame1, Frame frame2, tf::Vector3 angleChangePrediction)
	{

	}

	double computeError(cv::InputArray _m1, cv::InputArray _m2, cv::InputArray _model, cv::OutputArray _err)
	{
		double error = 0;

		cv::Mat X1 = _m1.getMat(), X2 = _m2.getMat(), model = _model.getMat();
		const cv::Point2d* x1ptr = X1.ptr<cv::Point2d>();
		const cv::Point2d* x2ptr = X2.ptr<cv::Point2d>();
		int n = X1.checkVector(2);
		cv::Matx33d E(model.ptr<double>());

		_err.create(n, 1, CV_32F);
		cv::Mat err = _err.getMat();

		for (int i = 0; i < n; i++)
		{
			cv::Vec3d x1(x1ptr[i].x, x1ptr[i].y, 1.);
			cv::Vec3d x2(x2ptr[i].x, x2ptr[i].y, 1.);
			cv::Vec3d Ex1 = E * x1;
			cv::Vec3d Etx2 = E.t() * x2;
			double x2tEx1 = x2.dot(Ex1);

			double a = Ex1[0] * Ex1[0];
			double b = Ex1[1] * Ex1[1];
			double c = Etx2[0] * Etx2[0];
			double d = Etx2[1] * Etx2[1];

			err.at<float>(i) = (float)(x2tEx1 * x2tEx1 / (a + b + c + d));
			error += (double)err.at<float>(i);
		}

		//set the computed values for later use
		this->error = error / (double)n;

		return this->error;
	}

protected:

	cv::Mat E;

	cv::Mat points1;
	cv::Mat points2;

	cv::Mat mask;

	double error;

};


#endif /* PAUVSI_M7_PAUVSI_VIO_INCLUDE_PAUVSI_VIO_EGOMOTIONESTIMATOR_HPP_ */
