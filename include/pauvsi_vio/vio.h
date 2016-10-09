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
#include "opencv2/video.hpp"
#include <vector>
#include <string>
#include <ros/ros.h>

#include "Frame.hpp"
#include "VIOFeature3D.hpp"
#include "VIOFeature2D.hpp"


#define DEFAULT_CAMERA_TOPIC "/camera/image"
#define DEFAULT_IMU_TOPIC "/IMU_Full"
#define DEFAULT_FAST_THRESHOLD 50
#define DEFAULT_2D_KILL_RADIUS 210
#define DEFAULT_FEATURE_SIMILARITY_THRESHOLD 10
#define DEFAULT_MIN_EIGEN_VALUE 1e-4

class VIO
{

private:

	//global vars initialized with default values
	std::string cameraTopic;
	std::string imuTopic;

	Frame currentFrame; // the current frame
	Frame lastFrame; //the last frame

	std::vector<double> position; // the current position
	std::vector<double> orientation; // the current orientation

	std::vector<VIOFeature3D> active3DFeatures;
	std::vector<VIOFeature3D> inactive3DFeatures;

public:

	int FAST_THRESHOLD;
	float KILL_RADIUS;
	int FEATURE_SIMILARITY_THRESHOLD;
	float MIN_EIGEN_VALUE;
	bool KILL_BY_DISSIMILARITY;

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
	void viewImage(Frame frame);

	std::vector<cv::DMatch> matchFeaturesWithFlann(cv::Mat queryDescriptors, cv::Mat trainDescriptors);

	bool flowFeaturesToNewFrame(Frame& oldFrame, Frame& newFrame);

	void getCorrespondingPointsFromFrames(Frame lastFrame, Frame currentFrame, std::vector<cv::Point2f>& lastPoints, std::vector<cv::Point2f>& currentPoints);

	int estimateMotion(Frame frame1, Frame frame2);

	void checkFeatureConsistency(Frame& checkFrame, int killThreshold );
};


#endif /* PAUVSI_VIO_INCLUDE_VIO_LIB_H_ */
