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

VIOState predict(VIOState lastState, ros::Time predictionTime)
{

}

/*
 * this function will use the state's current imu measurment to predict dt seconds into the future
 * and return the updated state.
 * NOTE: this function will keep the same imu reading as the last state x
 */
VIOState VIOEKF::transitionState(VIOState x, double dt)
{
	ROS_DEBUG_STREAM("state before: " << x.vector);
	// get the imu 2 com transform
	tf::StampedTransform imu2odom;
	try{
		tf_listener.lookupTransform(this->odom_frame, this->imu_frame, ros::Time(0), imu2odom);
	}
	catch(tf::TransformException e){
		ROS_WARN_STREAM(e.what());
	}

	//convert the imu readings to tf::Vectors and remove their biases
	tf::Vector3 alpha_tf(x.getAlpha()(0), x.getAlpha()(1), x.getAlpha()(2));
	alpha_tf = this->scaleAccelerometer * alpha_tf;
	tf::Vector3 omega_tf(x.getOmega()(0) - this->gyroBiasX, x.getOmega()(0)- this->gyroBiasY, x.getOmega()(0) - this->gyroBiasZ);

	//transform the imu readings into the center of mass frame
	alpha_tf = imu2odom * alpha_tf - imu2odom * tf::Vector3(0.0, 0.0, 0.0);
	omega_tf = imu2odom * omega_tf - imu2odom * tf::Vector3(0.0, 0.0, 0.0);

	Eigen::Vector3d alpha, omega;
	alpha << alpha_tf.getX(), alpha_tf.getY(), alpha_tf.getZ();
	omega << omega_tf.getX(), omega_tf.getY(), omega_tf.getZ();

	VIOState xNew;

	// these equations are from matlab's quatrotate function
	double ax = (alpha(0) * (1 - 2*x.q2()*x.q2() - 2*x.q3()*x.q3()) +
			alpha(1) * 2 * (x.q1()*x.q2() + x.q0()*x.q3()) + alpha(2) * 2 * (x.q1()*x.q3() - x.q0()*x.q2()));
	double ay = (alpha(0)*2*(x.q1()*x.q2() - x.q0()*x.q3()) + alpha(1)*(1-2*x.q1()*x.q1() - 2*x.q3()*x.q3())
			+ alpha(2)*2*(x.q2()*x.q3() + x.q0()*x.q1()));
	double az = (alpha(0) * 2 * (x.q1()*x.q3() + x.q0()*x.q2()) + alpha(1) * 2 * (x.q2()*x.q3() - x.q0()*x.q1()) +
			alpha(2) * (1 - 2 * x.q1()*x.q1() - 2 * x.q2()*x.q2()));

	// compute the delta quaternion
	double w_mag = sqrt(omega(0)*omega(0) + omega(1)*omega(1) + omega(2)*omega(2));

	double dq0 = 1;
	double dq1 = 0;
	double dq2 = 0;
	double dq3 = 0;
	
	if(w_mag != 0)
	{
		dq0 = cos(0.5 * w_mag * dt);
		dq1 = (2 * omega(0) / w_mag) * sin(0.5 * w_mag * dt);
		dq2 = (2 * omega(1) / w_mag) * sin(0.5 * w_mag * dt);
		dq3 = (2 * omega(2) / w_mag) * sin(0.5 * w_mag * dt);
	}

	Eigen::Quaterniond dq(dq0, dq1, dq2, dq3); // the delta quaternion
	dq.normalize();
	Eigen::Quaterniond q(x.q0(), x.q1(), x.q2(), x.q3());

	Eigen::Quaterniond newQ = dq * q; // rotate the quaternions


	//transition state
	xNew.vector(0, 0) = x.x() + x.dx()*dt + 0.5 * ax * dt*dt;

	xNew.vector(1, 0) = x.y() + x.dy()*dt + 0.5 * ay * dt*dt;

	xNew.vector(2, 0) = x.z() + x.dz()*dt + 0.5 * az * dt*dt;

	xNew.vector(3, 0) = x.dx() + ax * dt;

	xNew.vector(4, 0) = x.dy() + ay * dt;

	xNew.vector(5, 0) = x.dz() + az * dt;

	xNew.vector(6, 0) = newQ.w();

	xNew.vector(7, 0) = newQ.x();

	xNew.vector(8, 0) = newQ.y();

	xNew.vector(9, 0) = newQ.z();

	ROS_DEBUG_STREAM("state after: " << xNew.vector);

	//set the same imu reading
	xNew.setAlpha(x.getAlpha());
	xNew.setOmega(x.getOmega());

	//update the new state time
	xNew.setTime(ros::Time(x.getTime().toSec() + dt));

	return xNew;
}


