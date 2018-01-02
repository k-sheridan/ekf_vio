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
	this->K = _k;
	this->t = _t;


	//TODO make work for different rectification cases
	this->undistorted_width = this->img.cols;
	this->undistorted_height = this->img.rows;

}

Frame::~Frame() {
	// TODO Auto-generated destructor stub
}

