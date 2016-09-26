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
#include <vector>
#include <string>
#include <ros/ros.h>

class Frame
{

private:

public:
	ros::Time timeCreated;
	cv::Mat image;
	std::vector<cv::KeyPoint> corners;

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

	void setCorners(std::vector<cv::KeyPoint> _corners){
		this->corners = _corners;
	}

};



#endif /* PAUVSI_M7_INCLUDE_PAUVSI_VO_FRAME_H_ */
