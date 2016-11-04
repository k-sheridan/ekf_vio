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

	tf2_ros::TransformListener tf_listener(tfBuffer);

}

VIOEKF::~VIOEKF() {
	// TODO Auto-generated destructor stub
}

