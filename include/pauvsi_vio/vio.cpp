/*
 * vio.cpp
 *
 *  Created on: Sep 19, 2016
 *      Author: kevinsheridan
 */

#include "vio.h"

VIO::VIO()
{
	this->readROSParameters(); //import the parameters from server
}

/*
 * corrects the drift in position
 */
void VIO::correctPosition(std::vector<double> pos)
{
	position = pos;
}

/*
 * sets the current frame and starts keyframe update
 */
void VIO::setCurrentFrame(cv::Mat frame, ros::Time t)
{
	currentFrame = Frame(frame, t);
}

/*
 * gets parameters from ROS param server
 */
void VIO::readROSParameters()
{
	//CAMERA TOPIC
	ROS_WARN_COND(!ros::param::has("~cameraTopic"), "Parameter for 'cameraTopic' has not been set");
	ros::param::param<std::string>("~cameraTopic", cameraTopic, DEFAULT_CAMERA_TOPIC);
	ROS_DEBUG_STREAM("camera topic is: " << cameraTopic);

	//IMU TOPIC
	ROS_WARN_COND(!ros::param::has("~imuTopic"), "Parameter for 'imuTopic' has not been set");
	ros::param::param<std::string>("~imuTopic", imuTopic, DEFAULT_IMU_TOPIC);
	ROS_DEBUG_STREAM("IMU topic is: " << imuTopic);
}


