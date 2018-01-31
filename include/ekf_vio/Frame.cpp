/*
 * Frame.cpp
 *
 *  Created on: Aug 1, 2017
 *      Author: kevin
 */

#include "../ekf_vio/Frame.h"

Frame::Frame() {


}

Frame::Frame(int inv_scale, cv::Mat _img, boost::array<double, 9> k, std::vector<double> d , ros::Time _t)
{

	cv::Mat scaled_img;
	cv::resize(_img, scaled_img, cv::Size(_img.cols / inv_scale, _img.rows / inv_scale));
	this->img = scaled_img;

	this->t = _t;

	this->K.setZero();

	this->K(0, 0) = k.at(0)/inv_scale;
	this->K(0, 2) = k.at(2)/inv_scale;
	this->K(1, 1) = k.at(4)/inv_scale;
	this->K(1, 2) = k.at(5)/inv_scale;
	this->K(2, 2) = 1.0;

	ROS_DEBUG_STREAM("K: " << this->K);

	//this->D.setZero();
	this->D(0) = d.at(0);
	this->D(1) = d.at(1);
	this->D(2) = d.at(2);
	this->D(3) = d.at(3);
	this->D(4) = d.at(4);

}


bool Frame::isPixelInBox(cv::Point2f px)
{
	if(px.x < KILL_PAD || px.y < KILL_PAD || this->img.cols - px.x < KILL_PAD || this->img.rows - px.y < KILL_PAD)
	{
		ROS_DEBUG_STREAM("pixel is outside of kill box");
		return false;
	}
	else
	{
		return true;
	}
}
