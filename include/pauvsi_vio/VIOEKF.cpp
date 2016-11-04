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
	tf::Vector3 alpha(imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z);
	alpha = this->scaleAccelerometer * alpha;
	tf::Vector3 omega(imu.angular_velocity.x - this->gyroBiasX, imu.angular_velocity.y - this->gyroBiasY, imu.angular_velocity.z - this->gyroBiasZ);

	//transform the imu readings into the center of mass frame
	alpha = imu2odom * alpha - imu2odom * tf::Vector3(0.0, 0.0, 0.0);
	alpha = imu2odom * omega - imu2odom * tf::Vector3(0.0, 0.0, 0.0);
}


