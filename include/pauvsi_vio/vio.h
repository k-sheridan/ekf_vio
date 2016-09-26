/*
 * vio.h
 *
 *  Created on: Sep 19, 2016
 *      Author: kevinsheridan
 */

#ifndef PAUVSI_VIO_INCLUDE_VIO_H_
#define PAUVSI_VIO_INCLUDE_VIO_H_

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <vector>
#include <string>
#include <ros/ros.h>

#include "Frame.h"


#define DEFAULT_NUM_KEYFRAMES 1
#define DEFAULT_CAMERA_TOPIC "/camera/image"
#define DEFAULT_IMU_TOPIC "/IMU_Full"

class VIO
{

private:

	Frame currentFrame;
	std::vector<Frame> keyFrames; //the keyFrame vector
	int numKeyFrames; //number of keyframes to keep track of

	std::vector<double> position; // the current position
	std::vector<double> orientation; // the current orientation

public:

	//global vars initialized with default values
	std::string cameraTopic;
	std::string imuTopic;

	VIO();

	void correctPosition(std::vector<double> pos);

	void readROSParameters();

	void setCurrentFrame(cv::Mat frame, ros::Time t);

};


#endif /* PAUVSI_VIO_INCLUDE_VIO_LIB_H_ */
