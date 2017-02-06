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
#include <sensor_msgs/PointCloud.h>
#include "message_filters/subscriber.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <Frame.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>

#include "Point.h"
#include "Feature.h"
#include "FeatureTracker.h"
#include "VIOEKF.h"
#include "VIOState.hpp"
#include "KeyFrame.h"


#define SUPER_DEBUG true


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
#define DEFAULT_RECALIBRATION_THRESHOLD 0.02
#define DEFAULT_QUEUE_SIZE 10
#define DEFAULT_ACTIVE_FEATURES_TOPIC "/pauvsi_vio/activefeatures"
#define DEFAULT_PUBLISH_ACTIVE_FEATURES true
#define DEFAULT_MIN_TRIANGUALTION_DIST 0.1
#define DEFAULT_INIT_PXL_DELTA 1
#define DEFAULT_FRAME_BUFFER_LENGTH 20
#define DEFAULT_MAX_TRIAG_ERROR 2000
#define DEFAULT_MIN_TRIAG_Z 0.02
#define DEFAULT_MIN_TRIAG_FEATURES 40
#define DEFAULT_IDEAL_FUNDAMENTAL_PXL_DELTA 0.3
#define DEFAULT_MIN_FUNDAMENTAL_PXL_DELTA 0.3
#define DEFAULT_MAX_FUNDAMENTAL_ERROR 5e-7
#define DEFAULT_MAX_GN_ITERS 10

#define PI_OVER_180 0.01745329251

#define MAXIMUM_KEYFRAMES 1
#define DEFAULT_MINIMUM_KEYFRAME_FEATURE_RATIO 0.6



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
	bool PUBLISH_ACTIVE_FEATURES;
	double MIN_TRIANGUALTION_DIST;
	double INIT_PXL_DELTA;
	int FRAME_BUFFER_LENGTH;
	double MAX_TRIAG_ERROR;
	double MIN_TRIAG_Z;

	int MAX_GN_ITERS;
	int MIN_TRIAG_FEATURES;
	double IDEAL_FUNDAMENTAL_PXL_DELTA;
	double MIN_FUNDAMENTAL_PXL_DELTA;
	double MAXIMUM_FUNDAMENTAL_ERROR;

	bool initialized;

	std::string ACTIVE_FEATURES_TOPIC;

	typedef Eigen::Matrix<double, 3, 4> Matrix3x4d;

	//frames
	std::string imu_frame;
	std::string camera_frame;
	std::string odom_frame;
	std::string CoM_frame;
	std::string world_frame;

	VIO();
	~VIO();

	//VIO FUNCTIONS
	void imuCallback(const sensor_msgs::ImuConstPtr& msg);

	void cameraCallback(const sensor_msgs::ImageConstPtr& img, const sensor_msgs::CameraInfoConstPtr& cam);

	cv::Mat get3x3FromVector(boost::array<double, 9> vec);

	void correctOrientation(tf::Quaternion q, double certainty);

	void readROSParameters();

	void setCurrentFrame(cv::Mat frame, ros::Time t);

	Frame& currentFrame(){
		return this->frameBuffer.at(0);
	}

	Frame& lastFrame(){
		return this->frameBuffer.at(1);
	}

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

	void broadcastWorldToOdomTF();

	ros::Time broadcastOdomToTempIMUTF(double roll, double pitch, double yaw, double x, double y, double z);

	void recalibrateState(double avgPixelChange, double threshold, bool consecutive);

	void run();





	//DRAWING
	template <typename T>
	float distancePointLine(const cv::Point_<T> point, const cv::Vec<T,3>& line);

	template <typename T1, typename T2>
	void drawEpipolarLines(const std::string& title, const cv::Matx<T1,3,3> F,
			const cv::Mat& img1, const cv::Mat& img2,
			const std::vector<cv::Point_<T2> > points1,
			const std::vector<cv::Point_<T2> > points2,
			const float inlierDistance = -1);

	void viewImage(cv::Mat img);

	void viewImage(Frame frame);

	void viewMatches(std::vector<Feature> ft1, std::vector<Feature> ft2, Frame f1, Frame f2, std::vector<cv::Point2f> pt1_new, std::vector<cv::Point2f> pt2_new);

	//void viewMatches(std::vector<VIOFeature2D> ft1, std::vector<VIOFeature2D> ft2, Frame f1, Frame f2, std::vector<cv::Point2f> pt1_new, std::vector<cv::Point2f> pt2_new, cv::Matx33f F);

	void drawKeyFrames();





	//MOTION ESTIMATION
	VIOState estimateMotion(VIOState x, Frame& frame1, Frame& frame2);

	void updateKeyFrameInfo();

	tf::Transform cameraTransformFromState(VIOState x, tf::Transform b2c);

	void pose_gauss_newton(const std::vector< cv::Point3d > &wX,
	                       const std::vector< cv::Point2d > &x,
	                       cv::Mat &ctw, cv::Mat &cRw);

	float manhattan(cv::Point2f p1, cv::Point2f p2){
		return abs(p2.x - p1.x) + abs(p2.y - p1.y);
	}



	//TRIANGULATION
	bool Triangulate(const Matrix3x4d& pose1, const Matrix3x4d& pose2,
			const Eigen::Vector2d& point1, const Eigen::Vector2d& point2,
			Eigen::Vector4d* triangulated_point, Eigen::Matrix3d fmatrix);

	void FundamentalMatrixFromProjectionMatrices(const double pmatrix1[3 * 4],
			const double pmatrix2[3 * 4], double fmatrix[3 * 3]);

	bool TriangulateDLT(const Matrix3x4d& pose1, const Matrix3x4d& pose2,
			const Eigen::Vector2d& point1, const Eigen::Vector2d& point2,
			Eigen::Vector4d* triangulated_point);

	void FindOptimalImagePoints(const Eigen::Matrix3d& ematrix,
			const Eigen::Vector2d& point1, const Eigen::Vector2d& point2,
			Eigen::Vector2d* corrected_point1, Eigen::Vector2d* corrected_point2);

	void decomposeEssentialMatrix(cv::Matx33f E, cv::Matx34d& Rt);

	double ReprojectionError(const Matrix3x4d& pose, const Eigen::Vector4d& world_point, const Eigen::Vector2d& image_point);

protected:
	ros::NodeHandle nh;

	//tf2_ros::Buffer tfBuffer;

	image_transport::CameraSubscriber cameraSub;
	ros::Subscriber imuSub;

	ros::Publisher featurePub;

	//initialized with default values
	std::string cameraTopic;
	std::string imuTopic;

	//Frame currentFrame; // the current frame
	//Frame lastFrame; //the last frame

	std::deque<Frame> frameBuffer; // holds frames
	std::deque<KeyFrame> keyFrames; // holds information about the key frames

	FeatureTracker feature_tracker;

	VIOState lastState;
	VIOState state;
	VIOEKF ekf;

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

	cv::Mat K;
	cv::Mat D;

};


#endif /* PAUVSI_VIO_INCLUDE_VIO_LIB_H_ */
