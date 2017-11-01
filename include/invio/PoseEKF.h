/*
 * PoseEKF.h
 *
 *  Created on: Oct 31, 2017
 *      Author: kevin
 *
 */
#ifndef INVIO_INCLUDE_INVIO_POSEEKF_H_
#define INVIO_INCLUDE_INVIO_POSEEKF_H_

#include <ros/ros.h>
#include <sophus/se3.hpp>
#include "../invio/vioParams.h"

// x = x, y, z, qw, qx, qy, qz, lambda, b_dx, b_dy, b_dz, b_wx, b_wy, b_wz, b_ax, b_ay, b_az, gx, gy, gz, bias_accel_x, bias_accel_y, bias_accel_z, bias_gyro_x, bias_gyro_y, bias_gyro_z;
#define STATE_SIZE 26

class PoseEKF {

	//this EKF allows invio to fuse in an imu to the motion estimate

public:
	PoseEKF();
	virtual ~PoseEKF();

	Eigen::Matrix<double, STATE_SIZE, 1> x; // the state
	ros::Time t; // current time of state estimate
	Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> Sigma; // covariance

	void predict(ros::Time to_time); // predict the state to this time

	void updateWithVOPose(Sophus::SE3d pose, Eigen::Matrix<double, 6, 6> cov); // update with a pose estimate

};

#endif /* INVIO_INCLUDE_INVIO_POSEEKF_H_ */
