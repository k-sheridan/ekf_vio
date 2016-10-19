/*
 * FeatureTracker.h
 *
 *  Created on: Oct 18, 2016
 *      Author: kevinsheridan
 */

#ifndef PAUVSI_M7_PAUVSI_VIO_INCLUDE_PAUVSI_VIO_FEATURETRACKER_H_
#define PAUVSI_M7_PAUVSI_VIO_INCLUDE_PAUVSI_VIO_FEATURETRACKER_H_

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

class FeatureTracker {

public:

	FeatureTracker();
	FeatureTracker(int fst, float mev, bool kbd, int nf, int mnfd);

	std::vector<cv::DMatch> matchFeaturesWithFlann(cv::Mat queryDescriptors, cv::Mat trainDescriptors);

	bool flowFeaturesToNewFrame(Frame& oldFrame, Frame& newFrame);

	void getCorrespondingPointsFromFrames(Frame lastFrame, Frame currentFrame, std::vector<cv::Point2f>& lastPoints, std::vector<cv::Point2f>& currentPoints);

	void checkFeatureConsistency(Frame& checkFrame, int killThreshold );

	double averageFeatureChange(std::vector<cv::Point2f> points1, std::vector<cv::Point2f> points2);

protected:

	int FEATURE_SIMILARITY_THRESHOLD;
	float MIN_EIGEN_VALUE;
	bool KILL_BY_DISSIMILARITY;
	int NUM_FEATURES;
	int MIN_NEW_FEATURE_DISTANCE;

};

#endif /* PAUVSI_M7_PAUVSI_VIO_INCLUDE_PAUVSI_VIO_FEATURETRACKER_H_ */
