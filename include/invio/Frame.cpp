/*
 * Frame.cpp
 *
 *  Created on: Aug 1, 2017
 *      Author: kevin
 */

#include "../invio/Frame.h"

Frame::Frame() {


}

Frame::Frame(cv::Mat _img, cv::Mat_<float> _k, ros::Time _t)
{

	this->img = _img;

	this->t = _t;

	this->K << _k(0), _k(1), _k(2), _k(3), _k(4), _k(5), _k(6), _k(7), _k(8);

}

Frame::~Frame() {
	// TODO Auto-generated destructor stub
}

