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

#include <vioParams.h>
#include "Frame.h"
#include "Point.h"

class Point; // tell it that point is a class

class Frame; // need to tell the feature that there is something called frame

class Feature {
private:
	Frame* parentFrame;

	Point* point;

public:

	bool obsolete;

	cv::Point2f px;

	Feature();
	Feature(Frame* parent, cv::Point2f px);
	virtual ~Feature();

	bool computeObjectPositionWithPlanarApproximation(tf::Transform w2c, cv::Mat_<float> K);

	void computeObjectPositionWithAverageSceneDepth();

	Frame* getParentFrame(){ROS_ASSERT(parentFrame != NULL); return parentFrame;}

	Point* getPoint(){ ROS_ASSERT(point != NULL); return point;}

	void setParentFrame(Frame* f)
	{
		ROS_ASSERT(f != NULL);
		parentFrame = f;
	}

	void setPoint(Point* pt)
	{
		point = pt;
	}

	Eigen::Vector2d getMetricPixel();

	double getAverageFeatureDepth();

};

#endif /* PAUVSI_VIO_INCLUDE_PAUVSI_VIO_FEATURE_H_ */
