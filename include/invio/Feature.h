/*
 * Feature.h
 *
 *  Created on: Aug 1, 2017
 *      Author: kevin
 */

#ifndef PAUVSI_VIO_INCLUDE_PAUVSI_VIO_FEATURE_H_
#define PAUVSI_VIO_INCLUDE_PAUVSI_VIO_FEATURE_H_

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

#include <Frame.h>
#include <vioParams.h>

#include <Eigen/Core>

class Frame; // need to tell the feature that there is something called frame

class Feature {
private:

	Eigen::Vector3f mu; // [u, v, d] (u and v are in homogenous coord)

public:

	Feature();
	virtual ~Feature();

	Eigen::Vector2f getNormalizedPixel();

	float getDepth();

	cv::Point2f getPixel(Frame& f);

	static inline Eigen::Vector2f pixel2Metric(Frame& f, cv::Point2f px){
		return Eigen::Vector2f((px.x - f.K(2)) / f.K(0), (px.y - f.K(5)) / f.K(4));
	}

};

#endif /* PAUVSI_VIO_INCLUDE_PAUVSI_VIO_FEATURE_H_ */
