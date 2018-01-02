/*
 * Feature.cpp
 *
 *  Created on: Aug 1, 2017
 *      Author: kevin
 */

#include "../invio/Feature.h"

Feature::Feature() {

}

Feature::~Feature() {
	// TODO Auto-generated destructor stub
}

Eigen::Vector2d Feature::getMetricPixel(){
	return Eigen::Vector2d(this->mu(0), this->mu(1));
}

double Feature::getDepth(){
	return mu(2);
}
