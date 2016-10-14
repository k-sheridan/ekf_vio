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
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include "Frame.hpp"
#include "VIOFeature3D.hpp"
#include "VIOFeature2D.hpp"


#define DEFAULT_CAMERA_TOPIC "/camera/image"
#define DEFAULT_IMU_TOPIC "/IMU_Full"
#define DEFAULT_FAST_THRESHOLD 50
#define DEFAULT_2D_KILL_RADIUS 210
#define DEFAULT_FEATURE_SIMILARITY_THRESHOLD 10
#define DEFAULT_MIN_EIGEN_VALUE 1e-4
#define DEFAULT_NUM_FEATURES 50
#define DEFAULT_MIN_NEW_FEATURE_DIST 10

class VIO
{

public:

	int FAST_THRESHOLD;
	float KILL_RADIUS;
	int FEATURE_SIMILARITY_THRESHOLD;
	float MIN_EIGEN_VALUE;
	bool KILL_BY_DISSIMILARITY;
	int NUM_FEATURES;
	int MIN_NEW_FEATURE_DISTANCE;

	VIO();
	~VIO();

	//callbacks
	void imuCallback(const sensor_msgs::ImuConstPtr& msg);
	void cameraCallback(const sensor_msgs::ImageConstPtr& img, const sensor_msgs::CameraInfoConstPtr& cam);

	cv::Mat get3x3FromVector(boost::array<double, 9> vec);

	void correctOrientation(std::vector<double> orientation, double certainty);

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
	std::string getIMUTopic(){
		return imuTopic;
	}

	void setK(cv::Mat _K){
		K = _K;
	}

	void setD(cv::Mat _D){
		D = _D;
	}

	void viewImage(cv::Mat img, bool rect);
	void viewImage(Frame frame);

	std::vector<cv::DMatch> matchFeaturesWithFlann(cv::Mat queryDescriptors, cv::Mat trainDescriptors);

	bool flowFeaturesToNewFrame(Frame& oldFrame, Frame& newFrame);

	void getCorrespondingPointsFromFrames(Frame lastFrame, Frame currentFrame, std::vector<cv::Point2f>& lastPoints, std::vector<cv::Point2f>& currentPoints);

	bool visualMotionInference(Frame frame1, Frame frame2, std::vector<double> angleChangePrediction, std::vector<double>& rotationInference, std::vector<double>& unitVelocityInference, double& averageMovement);

	double estimateMotion();

	void checkFeatureConsistency(Frame& checkFrame, int killThreshold );

	void run();

	double averageFeatureChange(std::vector<cv::Point2f> points1, std::vector<cv::Point2f> points2);

	void addIMUReading(sensor_msgs::Imu msg){
		this->imuMessageBuffer.push_back(msg);
	}

protected:
	ros::NodeHandle nh;

	image_transport::CameraSubscriber cameraSub;

	tf::TransformListener listener; // starts a thread which keeps track of transforms in the system

	//initialized with default values
	std::string cameraTopic;
	std::string imuTopic;

	Frame currentFrame; // the current frame
	Frame lastFrame; //the last frame

	std::vector<double> position; // the current position
	std::vector<double> velocity; // the current velocity
	std::vector<double> orientation; // the current orientation in rpy

	/*
	 * this buffer stores IMU messages until they are needed
	 * for integration
	 * The reason we don't integrate them right away is because there is a
	 * slight gap it the time that the image is captured and when it is processed
	 */
	std::vector<sensor_msgs::Imu> imuMessageBuffer;

	std::vector<VIOFeature3D> active3DFeatures;
	std::vector<VIOFeature3D> inactive3DFeatures;

	cv::Mat K;
	cv::Mat D;

};

#endif /* PAUVSI_VIO_INCLUDE_VIO_LIB_H_ */
