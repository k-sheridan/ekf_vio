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

#include "EgoMotionEstimator.hpp"
#include "Frame.hpp"
#include "VIOFeature3D.hpp"
#include "VIOFeature2D.hpp"
#include "FeatureTracker.h"
#include "InertialMotionEstimator.h"


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
#define DEFAULT_GRAVITY_MAGNITUDE 9.8065
#define PI_OVER_180 0.01745329251
#define DEFAULT_RECALIBRATION_THRESHOLD 0.02
#define DEFAULT_QUEUE_SIZE 10

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
	double RECALIBRATION_THRESHOLD;

	tf::TransformListener tf_listener; // starts a thread which keeps track of transforms in the system

	//frames
	std::string imu_frame;
	std::string camera_frame;
	std::string odom_frame;
	std::string CoM_frame;
	std::string world_frame;

	VIO();
	~VIO();

	//callbacks
	void imuCallback(const sensor_msgs::ImuConstPtr& msg);
	void cameraCallback(const sensor_msgs::ImageConstPtr& img, const sensor_msgs::CameraInfoConstPtr& cam);

	cv::Mat get3x3FromVector(boost::array<double, 9> vec);

	void correctOrientation(tf::Quaternion q, double certainty);

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

	void viewImage(cv::Mat img);
	void viewImage(Frame frame);

	void broadcastWorldToOdomTF();

	ros::Time broadcastOdomToTempIMUTF(double roll, double pitch, double yaw, double x, double y, double z);

	double estimateMotion();

	void run();

	bool visualMotionInference(Frame frame1, Frame frame2, tf::Vector3 angleChangePrediction, tf::Vector3& rotationInference,
			tf::Vector3& unitVelocityInference, double& averageMovement);


	void recalibrateState(double avgPixelChange, double threshold, bool consecutive);

protected:
	ros::NodeHandle nh;

	image_transport::CameraSubscriber cameraSub;
	ros::Subscriber imuSub;

	//initialized with default values
	std::string cameraTopic;
	std::string imuTopic;

	Frame currentFrame; // the current frame
	Frame lastFrame; //the last frame

	FeatureTracker feature_tracker;

	InertialMotionEstimator inertial_motion_estimator;

	tf::Vector3 position;
	tf::Vector3 velocity;
	tf::Vector3 angular_velocity;
	tf::Quaternion orientation;
	struct gyroNode
	{
		tf::Vector3 gyroBias;
		double certainty;
	};
	struct accelNode
	{
		double accelScale;
		double certainty;
	};
	std::vector<gyroNode> gyroQueue;
	std::vector<accelNode> accelQueue;

	std::vector<VIOFeature3D> active3DFeatures;
	std::vector<VIOFeature3D> inactive3DFeatures;

	cv::Mat K;
	cv::Mat D;

};

#endif /* PAUVSI_VIO_INCLUDE_VIO_LIB_H_ */
