/*
 * klt_test.cpp
 *
 *  Created on: Jan 4, 2018
 *      Author: kevin
 */



#include <ros/ros.h>
#include <ros/package.h>

#include "../include/ekf_vio/EKFVIO.h"
#include <KLTTracker.h>


int main(int argc, char **argv)
{
	ros::init(argc, argv, "klt_test"); // initializes ros

	cv::Mat raw_img1;
	std::stringstream ss;
	ss << ros::package::getPath("invio") << "/images/640_480_test.png";
	raw_img1 = cv::imread(ss.str(), CV_LOAD_IMAGE_COLOR);
	cv::Mat img1;
	cvtColor( raw_img1, img1, cv::COLOR_BGR2GRAY );

	cv::imshow("test",img1);
	cv::waitKey(1000);

	// test the blur functionality
	KLTTracker::Pyramid pyr = KLTTracker::Pyramid(2, img1);





	return 0;
}
