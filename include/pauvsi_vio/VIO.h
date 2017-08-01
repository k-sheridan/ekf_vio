/*
 * FeatureTracker.h
 *
 *  Created on: Jul 8, 2017
 *      Author: kevin
 */

#ifndef PAUVSI_VIO_INCLUDE_PAUVSI_VIO_VIO_H_
#define PAUVSI_VIO_INCLUDE_PAUVSI_VIO_VIO_H_

#include <ros/ros.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/video.hpp"
#include <vector>
#include <string>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <tf/tfMessage.h>

#include <vioParams.h>
#include <Feature.h>
#include <Frame.h>

class VIO {
public:

	std::deque<Frame> frame_buffer; // stores all frames and pose estimates at this frames

	tf::TransformListener tf_listener;

	VIO();
	virtual ~VIO();

	std::vector<cv::Point2d> getPixelsInOrder(Frame& f);

	std::vector<cv::Point2f> getPixels2fInOrder(Frame& f);

	std::vector<cv::Point3d> getObjectsInOrder(Frame& f);

	void addFrame(cv::Mat img, cv::Mat_<float> k, ros::Time t);

	void updateFeatures(Frame& last_f, Frame& new_f);

	bool computePose(double& perPixelError);

	void updatePose(tf::Transform w2c, ros::Time t);

	void replenishFeatures(Frame& f);

	void tf2rvecAndtvec(tf::Transform tf, cv::Mat& tvec, cv::Mat& rvec);

	tf::Transform rvecAndtvec2tf(cv::Mat tvec, cv::Mat rvec);

	cv::Mat draw(cv::Mat in)
	{
		for(auto e : frame_buffer.front().features)
		{
			cv::drawMarker(in, e.px, cv::Scalar(255, 0, 0));
		}
		return in;
	}
};

#endif /* PAUVSI_VIO_INCLUDE_PAUVSI_VIO_VIO_H_ */
