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
#include <Params.h>

#include <Eigen/Core>

class Frame; // need to tell the feature that there is something called frame

class Feature {
private:

	Eigen::Vector3f mu; // [u, v, d^-1] (u and v are in homogenous coord), depth is stored as inverse to prvent issues with far objects and prevent features from converging behind the camera

	Eigen::Vector2f last_result_from_klt_tracker; // used to store the previous reference feature position
	// it is important that this estimate is as close to the actual feature position in the frame as possible. otherwise drift will be significant

	bool delete_flag;

public:

	Feature();
	Feature(Eigen::Vector2f homogenous, float depth);
	virtual ~Feature();

	Eigen::Vector2f getNormalizedPixel();

	float getDepth();

	cv::Point2f getPixel(const Frame& f);

	static inline Eigen::Vector2f pixel2Metric(const Frame& f, const cv::Point2f px){
		return Eigen::Vector2f((px.x - f.K(2)) / f.K(0), (px.y - f.K(5)) / f.K(4));
	}

	static inline cv::Point2f metric2Pixel(const Frame& f, const Eigen::Vector2f pos){
		return cv::Point2f(pos.x()*f.K(0) + f.K(2), pos.y()*f.K(4) + f.K(5));
	}

	Eigen::Vector2f getLastResultFromKLTTracker(){
		return this->last_result_from_klt_tracker;
	}

	/*
	 * this should be given in meters
	 */
	void setLastResultFromKLTTracker(Eigen::Vector2f in){
		this->last_result_from_klt_tracker = in;
	}

	bool flaggedForDeletion(){return this->delete_flag;}
	void setDeleteFlag(bool in){this->delete_flag = in;}

	void setNormalizedPixel(Eigen::Vector2f in){
		this->mu(0) = in(0);
		this->mu(1) = in(1);
	}

	void setDepth(float in){
		this->mu(2) = in;
	}

	void setMu(Eigen::Vector3f in){this->mu = in;}

	Eigen::Vector3f& getMu(){return this->mu;}

};

#endif /* PAUVSI_VIO_INCLUDE_PAUVSI_VIO_FEATURE_H_ */
