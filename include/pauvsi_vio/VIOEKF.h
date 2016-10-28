/*
 * VIOEKF.h
 *
 *  Created on: Oct 28, 2016
 *      Author: kevin
 */

#ifndef PAUVSI_M7_PAUVSI_VIO_INCLUDE_PAUVSI_VIO_VIOEKF_H_
#define PAUVSI_M7_PAUVSI_VIO_INCLUDE_PAUVSI_VIO_VIOEKF_H_

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

class VIOEKF {
public:
	VIOEKF();
	virtual ~VIOEKF();

	double gyroBiasX;
	double gyroBiasY;
	double gyroBiasZ;
	double scaleAccelerometer;
	sensor_msgs::Imu lastMessageUsed;


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

	sensor_msgs::Imu getMostRecentImu()
	{
		if(imuMessageBuffer.empty())
			return lastMessageUsed;

		return imuMessageBuffer.back();
	}

	tf::Quaternion getDifferenceQuaternion(tf::Vector3 v1, tf::Vector3 v2)
	{
		tf::Quaternion q;
		tf::Vector3 a = v1.cross(v2);
		q.setX(a.getX());
		q.setY(a.getY());
		q.setZ(a.getZ());
		q.setW(sqrt(v1.length()*v1.length()*v2.length()*v2.length()) + v1.dot(v2));
		q.normalize();

		return q;
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

#endif /* PAUVSI_M7_PAUVSI_VIO_INCLUDE_PAUVSI_VIO_VIOEKF_H_ */
