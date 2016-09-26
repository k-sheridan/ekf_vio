/*
 * vio.h
 *
 *  Created on: Sep 19, 2016
 *      Author: kevinsheridan
 */

#ifndef PAUVSI_VIO_INCLUDE_VIO_H_
#define PAUVSI_VIO_INCLUDE_VIO_H_

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include <vector>
#include <string>
#include <ros/ros.h>

#include "Frame.h"


#define DEFAULT_CAMERA_TOPIC "/camera/image"
#define DEFAULT_IMU_TOPIC "/IMU_Full"
#define DEFAULT_FAST_THRESHOLD 100

class VIO
{

private:

	//global vars initialized with default values
	std::string cameraTopic;
	std::string imuTopic;

	Frame currentFrame;
	bool currentFrameSet;
	Frame lastFrame; //the last frame
	bool lastFrameSet;

	std::vector<cv::DMatch> matchesFromLastToCurrentFrame;

	std::vector<double> position; // the current position
	std::vector<double> orientation; // the current orientation

	int fastThreshold;

public:

	VIO();

	void correctPosition(std::vector<double> pos);

	void readROSParameters();

	void setCurrentFrame(cv::Mat frame, ros::Time t);

	/*
	 * gets the most recently added frame
	 */
	Frame getCurrentFrame(){
		return currentFrame;
	}

	/*
	 * checks if the last frame has been set
	 */
	bool isLastFrameSet(){
		return this->lastFrameSet;
	}

	/*
	 * gets the last frame
	 */
	Frame getLastFrame(){
		return lastFrame;
	}

	/*
	 * returns the camera topic used by this node
	 */
	std::string getCameraTopic(){
		return cameraTopic;
	}

	void viewImage(cv::Mat img);
	void viewImage(cv::Mat img, std::vector<cv::KeyPoint> keypoints);
	void viewImage(Frame frame1, Frame frame2, std::vector<cv::DMatch> matches);

	std::vector<cv::KeyPoint> computeFASTFeatures(cv::Mat, int);

	cv::Mat extractFREAKDescriptors(cv::Mat, std::vector<cv::KeyPoint>);
	cv::Mat extractBRIEFDescriptors(cv::Mat, std::vector<cv::KeyPoint>);

	std::vector<cv::DMatch> matchFeaturesWithFlann(cv::Mat queryDescriptors, cv::Mat trainDescriptors);


};


#endif /* PAUVSI_VIO_INCLUDE_VIO_LIB_H_ */
