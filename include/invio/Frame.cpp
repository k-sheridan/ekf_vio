/*
 * Frame.cpp
 *
 *  Created on: Aug 1, 2017
 *      Author: kevin
 */

#include "../invio/Frame.h"

Frame::Frame() {


}

Frame::Frame(int inv_scale, cv::Mat _img, boost::array<double, 9> k, std::vector<double> d , ros::Time _t)
{

	cv::Mat scaled_img;
	cv::resize(_img, scaled_img, cv::Size(_img.cols / inv_scale, _img.rows / inv_scale));
	this->img = scaled_img;

	this->t = _t;

	this->K << k.at(0), k.at(1), k.at(2), k.at(3), k.at(4), k.at(5), k.at(6), k.at(7), k.at(8);
	this->K = (double)(1/inv_scale) * this->K;

	this->D << d.at(0), d.at(1), d.at(2), d.at(3), d.at(4);

}

Frame::~Frame() {
	// TODO Auto-generated destructor stub
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
