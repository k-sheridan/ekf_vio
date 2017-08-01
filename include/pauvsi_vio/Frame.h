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

public:

	std::deque<Feature> features;

	tf::Transform poseEstimate; // this is a world to camera pose estimate

	cv::Mat_<float> K;
	cv::Mat img;
	ros::Time t;

	Frame();
	Frame(cv::Mat _img, cv::Mat_<float> _k, ros::Time _t);

	virtual ~Frame();

	bool isKeyframe(){return this->isKeyframe;}
};

#endif /* PAUVSI_VIO_INCLUDE_PAUVSI_VIO_FRAME_H_ */
