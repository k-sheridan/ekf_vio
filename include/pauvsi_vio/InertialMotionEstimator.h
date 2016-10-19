/*
 * InertialMotionEstimator.h
 *
 *  Created on: Oct 18, 2016
 *      Author: kevinsheridan
 */

#ifndef PAUVSI_M7_PAUVSI_VIO_INCLUDE_PAUVSI_VIO_INERTIALMOTIONESTIMATOR_H_
#define PAUVSI_M7_PAUVSI_VIO_INCLUDE_PAUVSI_VIO_INERTIALMOTIONESTIMATOR_H_

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


class InertialMotionEstimator{
public:

	InertialMotionEstimator();

	void setGravityMagnitude(double g)
	{
		GRAVITY_MAG = g;
	}

	void setTransforms(tf::StampedTransform c2o, tf::StampedTransform i2c, tf::StampedTransform c2i, tf::StampedTransform i2o)
	{
		cam2odom = c2o;
		imu2cam = i2c;
		com2imu = c2i;
		imu2odom = i2o;
	}

	int getInertialMotionEstimate(ros::Time fromTime, ros::Time toTime, tf::Vector3 fromVelocity,
			tf::Vector3 fromAngularVelocity, tf::Vector3& angleChange,
			tf::Vector3& positionChange, tf::Vector3& velocityChange, tf::Vector3& lastOmega);

	void addIMUMessage(sensor_msgs::Imu msg)
	{
		this->imuMessageBuffer.push_back(msg);
	}

protected:

	/*
	 * this buffer stores IMU messages until they are needed
	 * for integration
	 * The reason we don't integrate them right away is because there is a
	 * slight gap it the time that the image is captured and when it is processed
	 */
	std::vector<sensor_msgs::Imu> imuMessageBuffer;

	tf::StampedTransform cam2odom, imu2cam, com2imu, imu2odom;

	double GRAVITY_MAG;
};

#endif /* PAUVSI_M7_PAUVSI_VIO_INCLUDE_PAUVSI_VIO_INERTIALMOTIONESTIMATOR_H_ */
