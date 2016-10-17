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
#include <tf/transform_broadcaster.h>

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
#define DEFAULT_IMU_FRAME_NAME "imu_frame"
#define DEFAULT_ODOM_FRAME_NAME "odom"
#define DEFAULT_CAMERA_FRAME_NAME "camera_frame"
#define DEFAULT_COM_FRAME_NAME "base_link"
#define DEFAULT_WORLD_FRAME_NAME "world"
#define DEFAULT_GRAVITY_MAGNITUDE 10.027876884


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
	double GRAVITY_MAG;

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

	void broadcastWorldToOdomTF();

	ros::Time broadcastOdomToTempIMUTF(double roll, double pitch, double yaw, double x, double y, double z);

	std::vector<cv::DMatch> matchFeaturesWithFlann(cv::Mat queryDescriptors, cv::Mat trainDescriptors);

	bool flowFeaturesToNewFrame(Frame& oldFrame, Frame& newFrame);

	void getCorrespondingPointsFromFrames(Frame lastFrame, Frame currentFrame, std::vector<cv::Point2f>& lastPoints, std::vector<cv::Point2f>& currentPoints);

	double estimateMotion();

	void checkFeatureConsistency(Frame& checkFrame, int killThreshold );

	void run();

	double averageFeatureChange(std::vector<cv::Point2f> points1, std::vector<cv::Point2f> points2);

	std::vector<double> placeFeatureInSpace(cv::Point2f point1,cv::Point2f point2,cv::Mat rotation, cv::Mat translation );

	void addIMUReading(sensor_msgs::Imu msg){
		this->imuMessageBuffer.push_back(msg);
	}

	bool visualMotionInference(Frame frame1, Frame frame2, tf::Vector3 angleChangePrediction, tf::Vector3& rotationInference,
			tf::Vector3& unitVelocityInference, double& averageMovement);

	int getInertialMotionEstimate(ros::Time fromTime, ros::Time toTime, tf::Vector3 fromVelocity,
			tf::Vector3 fromAngularVelocity, tf::Vector3& angleChange,
			tf::Vector3& positionChange, tf::Vector3& velocityChange);

	void assembleStateVectors(tf::Vector3 finalPositionChange, tf::Vector3 finalAngleChange, tf::Vector3 finalVelocityChange, tf::Vector3 angular_velocity);



protected:
	ros::NodeHandle nh;

	image_transport::CameraSubscriber cameraSub;
	ros::Subscriber imuSub;

	tf::TransformListener tf_listener; // starts a thread which keeps track of transforms in the system

	//initialized with default values
	std::string cameraTopic;
	std::string imuTopic;

	//frames
	std::string imu_frame;
	std::string camera_frame;
	std::string odom_frame;
	std::string CoM_frame;
	std::string world_frame;

	Frame currentFrame; // the current frame
	Frame lastFrame; //the last frame

	geometry_msgs::PoseStamped pose;
	geometry_msgs::Vector3Stamped velocity;
	geometry_msgs::Vector3Stamped angular_velocity;

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
