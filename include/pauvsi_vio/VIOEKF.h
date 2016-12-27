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
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include "tf/tfMessage.h"
#include "message_filters/subscriber.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include "pauvsi_vio/VIOState.hpp"
#include <eigen3/Eigen/Geometry>
#include "pauvsi_vio/VisualMeasurement.hpp"

#define PI 3.14156
#define CONVERT_2_RAD_DEFAULT false

class VIOEKF {
public:
	VIOEKF();

	virtual ~VIOEKF();

	double gyroBiasX;
	double gyroBiasY;
	double gyroBiasZ;
	double scaleAccelerometer;
	sensor_msgs::Imu lastMessageUsed;

	//frames
	std::string imu_frame;
	std::string camera_frame;
	std::string odom_frame;
	std::string CoM_frame;
	std::string world_frame;

	tf::TransformListener tf_listener;

	VIOState predict(VIOState lastState, ros::Time predictionTime);

	VIOState update(VIOState lastState, VisualMeasurement z);

	//STATE x STATE
	Eigen::Matrix<double, 16, 16> stateJacobian(VIOState x, double dt);

	Eigen::Matrix<double, 16, 16> computePredictionError(double dt);

	VIOState transitionState(VIOState x, double dt);

	VisualMeasurement predictMeasurement();

	void setGravityMagnitude(double g)
	{
		GRAVITY_MAG = g;
	}

	void addIMUMessage(sensor_msgs::Imu msg)
	{
		if(convert2rad)
		{
			msg.angular_velocity.x = PI / 180 * msg.angular_velocity.x;
			msg.angular_velocity.y = PI / 180 * msg.angular_velocity.y;
			msg.angular_velocity.z = PI / 180 * msg.angular_velocity.z;
		}
		this->imuMessageBuffer.push_back(msg);
	}

	int getMessagesBetweenTimes(ros::Time t0, ros::Time t1, std::vector<sensor_msgs::Imu>& returnBuffer);

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

	bool convert2rad;

	double GRAVITY_MAG;
};

#endif /* PAUVSI_M7_PAUVSI_VIO_INCLUDE_PAUVSI_VIO_VIOEKF_H_ */
