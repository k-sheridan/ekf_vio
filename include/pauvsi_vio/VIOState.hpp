/*
 * VIOState.hpp
 *
 *  Created on: Oct 28, 2016
 *      Author: kevin
 */

#ifndef PAUVSI_M7_PAUVSI_VIO_INCLUDE_PAUVSI_VIO_VIOSTATE_HPP_
#define PAUVSI_M7_PAUVSI_VIO_INCLUDE_PAUVSI_VIO_VIOSTATE_HPP_

#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <sensor_msgs/Imu.h>
#include <opencv2/core/eigen.hpp>


class VIOState
{
public:

	Eigen::Matrix<double, 16, 1> vector; //x, y, z, dx, dy, dz, q0, q1, q2, q3, bgx, bgy, bgz, bax, bay, baz
	Eigen::Matrix<double, 16, 16> covariance; // the covariance matrix for this state
	bool timeSet;

	VIOState(){
		vector << 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0; // initialize the state vector
		covariance = 0.001 * Eigen::MatrixXd::Identity(16, 16);
		omega << 0, 0, 0;
		alpha << 0, 0, 0;
		timeSet = false;
		//t = ros::Time::now();
	}

	double x(){
		return vector(0, 0);
	}

	double y(){
		return vector(1, 0);
	}

	double z(){
		return vector(2, 0);
	}

	double dx(){
		return vector(3, 0);
	}

	double dy(){
		return vector(4, 0);
	}

	double dz(){
		return vector(5, 0);
	}

	double q0(){
		return vector(6, 0);
	}

	double q1(){
		return vector(7, 0);
	}

	double q2(){
		return vector(8, 0);
	}

	double q3(){
		return vector(9, 0);
	}

	void setOmega(Eigen::Vector3d omega)
	{
		this->omega = omega;
	}

	void setOmega(double x, double y, double z)
	{
		this->setOmega(Eigen::Vector3d(x, y, z));
	}

	void setAlpha(Eigen::Vector3d alpha)
	{
		this->alpha = alpha;
	}

	void setAlpha(double x, double y, double z)
	{
		this->setAlpha(Eigen::Vector3d(x, y, z));
	}

	void setIMU(sensor_msgs::Imu msg)
	{
		this->setAlpha(msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z);
		this->setOmega(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z);
	}

	void setTime(ros::Time t)
	{
		timeSet = true;
		this->t = t;
	}

	ros::Time getTime()
	{
		return this->t;
	}

	Eigen::Vector3d getOmega()
	{
		return this->omega;
	}

	Eigen::Vector3d getAlpha()
	{
		return this->alpha;
	}

	void setQuaternion(tf::Quaternion q)
	{
		vector(6, 0) = q.getW();
		vector(7, 0) = q.getX();
		vector(8, 0) = q.getY();
		vector(9, 0) = q.getZ();
	}

	tf::Quaternion getTFQuaternion()
	{
		//ROS_DEBUG_STREAM("values " << q1() << ", " << q3() << ", " << q2() << ", " << q0());
		return tf::Quaternion(q1(), q2(), q3(), q0());
	}

	void setVelocity(Eigen::Vector3d vel)
	{
		vector(3, 0) = vel[0];
		vector(4, 0) = vel[1];
		vector(5, 0) = vel[2];
	}

	Eigen::Vector3d getVelocity()
	{
		Eigen::Vector3d v;
		v << dx(), dy(), dz();
		return v;
	}

	/*
	 * creates the extrinsic matrix for this state
	 * takes the transform from base link to camera
	 */
	cv::Matx34d getRTMatrix(tf::StampedTransform base2cam)
	{
		tf::StampedTransform world2odom;
		world2odom.setOrigin(tf::Vector3(this->x(), this->y(), this->z()));
		world2odom.setRotation(this->getTFQuaternion());

		tf::Transform world2cam = world2odom * base2cam;

		tf::Quaternion tf_q = world2cam.getRotation();
		Eigen::Quaternion<double> q = Eigen::Quaternion<double>(tf_q.w(), tf_q.x(), tf_q.y(), tf_q.z());

		Eigen::Matrix<double,3,3> Rc = q.matrix();

		ROS_DEBUG_STREAM(" R: " << Rc);

		Eigen::Vector3cd C;
		C << world2cam.getOrigin().getX(), world2cam.getOrigin().getY(), world2cam.getOrigin().getZ();

		Eigen::Vector3cd t = -Rc.transpose() * C;

		ROS_DEBUG_STREAM("t: " << t);

		Eigen::Matrix<double,3,4> RT;
		RT.block(0, 0, 3, 3) = Rc;
		RT(0, 3) = t(0).real();
		RT(1, 3) = t(1).real();
		RT(2, 3) = t(2).real();

		cv::Matx34d cv_RT;
		cv::eigen2cv(RT, cv_RT);

		ROS_DEBUG_STREAM("RT: " << cv_RT);

		return cv_RT;
	}

private:

	Eigen::Vector3d omega;
	Eigen::Vector3d alpha;
	ros::Time t;
};



#endif /* PAUVSI_M7_PAUVSI_VIO_INCLUDE_PAUVSI_VIO_VIOSTATE_HPP_ */
