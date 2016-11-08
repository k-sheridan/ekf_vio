/*
 * VIOEKF.cpp
 *
 *  Created on: Oct 28, 2016
 *      Author: kevin
 */

#include <VIOEKF.h>

VIOEKF::VIOEKF() {
	this->gyroBiasX = 0;
	this->gyroBiasY = 0;
	this->gyroBiasZ = 0;
	this->scaleAccelerometer = 1.0;
	this->GRAVITY_MAG = 9.8;

}

VIOEKF::~VIOEKF() {
	// TODO Auto-generated destructor stub
}

VIOState VIOEKF::transitionState(VIOState x, sensor_msgs::Imu imu, double dt)
{
	// get the imu 2 com transform
	tf::StampedTransform imu2odom;
	try{
		tf_listener.lookupTransform(this->odom_frame, this->imu_frame, ros::Time(0), imu2odom);
	}
	catch(tf::TransformException e){
		ROS_WARN_STREAM(e.what());
	}

	//convert the imu readings to tf::Vectors and remove their biases
	tf::Vector3 alpha_tf(imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z);
	alpha_tf = this->scaleAccelerometer * alpha_tf;
	tf::Vector3 omega_tf(imu.angular_velocity.x - this->gyroBiasX, imu.angular_velocity.y - this->gyroBiasY, imu.angular_velocity.z - this->gyroBiasZ);

	//transform the imu readings into the center of mass frame
	alpha_tf = imu2odom * alpha_tf - imu2odom * tf::Vector3(0.0, 0.0, 0.0);
	omega_tf = imu2odom * omega_tf - imu2odom * tf::Vector3(0.0, 0.0, 0.0);

	Eigen::Vector3d alpha, omega;
	alpha << alpha_tf.getX(), alpha_tf.getY(), alpha_tf.getZ();
	omega << omega_tf.getX(), omega_tf.getY(), omega_tf.getZ();

	VIOState xNew;
	double ax = (alpha(0) * (1 - 2*x.q2()*x.q2() - 2*x.q3()*x.q3()) +
			alpha(1) * 2 * (x.q1()*x.q2() + x.q0()*x.q3()) + alpha(2) * 2 * (x.q1()*x.q3() - x.q0()*x.q2()));
	double ay = (alpha(0)*2*(x.q1()*x.q2() - x.q0()*x.q3()) + alpha(1)*(1-2*x.q1()*x.q1() - 2*x.q3()*x.q3())
			+ alpha(2)*2*(x.q2()*x.q3() + x.q0()*x.q1()));


	//transition state
	xNew.vector(0, 0) = x.x() + x.dx()*dt + 0.5 * ax * dt*dt;

	xNew.vector(1, 0) = x.y() + x.dy()*dt + 0.5 * ay * dt*dt;
}


