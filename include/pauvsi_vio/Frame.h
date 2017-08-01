/*
 * Frame.h
 *
 *  Created on: Aug 1, 2017
 *      Author: kevin
 */

#ifndef PAUVSI_VIO_INCLUDE_PAUVSI_VIO_FRAME_H_
#define PAUVSI_VIO_INCLUDE_PAUVSI_VIO_FRAME_H_

#include <Feature.h>
#include <vioParams.h>

class Frame {
private:

	bool keyframe;

	double avgFeatureDepth;
	bool avgFeatureDepthSet;

	tf::Transform poseEstimate; // this is a world to camera pose estimate
	tf::Transform poseEstimate_inv;

public:

	std::deque<Feature> features;

	cv::Mat_<float> K;
	cv::Mat img;
	ros::Time t;

	Frame();
	Frame(cv::Mat _img, cv::Mat_<float> _k, ros::Time _t);

	virtual ~Frame();

	bool isKeyframe(){return this->isKeyframe;}

	tf::Transform getPose(){return poseEstimate;}
	tf::Transform getPose_inv(){return poseEstimate_inv;}

	void setPose(tf::Transform tf);
	void setPose_inv(tf::Transform tf);

	double getAverageFeatureDepth();
};

#endif /* PAUVSI_VIO_INCLUDE_PAUVSI_VIO_FRAME_H_ */
