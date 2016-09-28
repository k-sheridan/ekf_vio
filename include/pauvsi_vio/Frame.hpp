/*
 * frame.h
 *
 *  Created on: Sep 20, 2016
 *      Author: kevinsheridan
 */

#ifndef PAUVSI_M7_INCLUDE_PAUVSI_VO_FRAME_H_
#define PAUVSI_M7_INCLUDE_PAUVSI_VO_FRAME_H_

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

#include "VIOFeature2D.hpp"

class Frame
{

private:

public:
	ros::Time timeCreated;
	cv::Mat image;
	std::vector<VIOFeature2D> features; //the feature vector for this frame

	Frame(cv::Mat img, ros::Time t)
	{
		this->image = img;
		this->timeCreated = t;
	}

	Frame()
	{

	}

	/*
	 * gets the time since this frame was create
	 */
	double getAgeSeconds()
	{
		return ros::Time::now().toSec() - this->timeCreated.toSec();
	}

};



#endif /* PAUVSI_M7_INCLUDE_PAUVSI_VO_FRAME_H_ */
