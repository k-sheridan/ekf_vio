/*
 * FeatureTracker.h
 *
 *  Created on: Jul 8, 2017
 *      Author: kevin
 */

#ifndef PAUVSI_VIO_INCLUDE_PAUVSI_VIO_VIO_H_
#define PAUVSI_VIO_INCLUDE_PAUVSI_VIO_VIO_H_

#include <ros/ros.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/video.hpp"
#include <vector>
#include <string>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <tf/tfMessage.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <nav_msgs/Odometry.h>

#include <sophus/se3.hpp>

#include "../invio/Feature.h"
#include "../invio/Frame.h"
#include <sophus/types.hpp>
#include "../invio/vioParams.h"

class VIO {
public:

	//ROS PARAMS
	bool PUBLISH_INSIGHT;
	std::string INSIGHT_TOPIC;

	//VISUAL ODOM
	//fast corner detector for planar odometry
	int FAST_THRESHOLD;
	// the amount to blur the image before feature extraction
	double FAST_BLUR_SIGMA;

	double INVERSE_IMAGE_SCALE;

	// use previous odometry for prior
	bool USE_ODOM_PRIOR;

	//analyze the function times
	bool ANALYZE_RUNTIME;

	// the radius to remove features at in pixels
	int KILL_BOX_WIDTH;
	int KILL_BOX_HEIGHT;

	// the minimum feature eigen val where it is determined as lost
	double KLT_MIN_EIGEN;

	// the minimum pixel distance a feature must have between a previous feature
	double MIN_NEW_FEATURE_DIST;

	// the desired number of features more = robust...er (and slower)
	int NUM_FEATURES;

	// the amount of points needed to start pauvsi vio odometry
	int START_FEATURE_COUNT;

	// the minimum amount of mature features which is deemed dangerous
	int DANGEROUS_MATURE_FEATURE_COUNT_LEVEL;

	//minimum detected features before vo has failed
	int MINIMUM_TRACKABLE_FEATURES;

	// the amount of frames to store for feature optimization
	int FRAME_BUFFER_SIZE;

	// the number of times a feature must be observed before allowed to be optimized
	int KEYFRAME_COUNT_FOR_OPTIMIZATION;

	// the minimum ratio of translation to avg scene depth
	double T2ASD;

	// the maximum error a feature can have after an optim
	double MAXIMUM_FEATURE_DEPTH_ERROR;

	// default point depth used for initialization in meters
	double DEFAULT_POINT_DEPTH;
	double DEFAULT_POINT_STARTING_ERROR;

	// epsilon for convergence in structure bundle adjustment and motion
	double EPS_SBA;
	double EPS_MOBA;

	//max iterations for gausss newton
	int MOBA_MAX_ITERATIONS;
	int SBA_MAX_ITERATIONS;

	//min and maximum point depths in meters
	double MAX_POINT_Z;
	double MIN_POINT_Z;

	//END VISUAL ODOM

	std::string ODOM_TOPIC;
	std::string POINTS_TOPIC;

	//double CAMERA_TOPIC "/guidance/left/image_rect"
	//double CAMERA_FRAME "guidance"
	std::string CAMERA_TOPIC;

	std::string BASE_FRAME;

	std::string WORLD_FRAME;
	//END ROS PARAMS

	bool initialized; // has the program been initialized
	bool tracking_lost; // do we still have tracking

	ros::Time timer_start;

	std::list<Frame> frame_buffer; // stores all frames and pose estimates at this frames

	std::list<Point> map; // this is a list of all active 3d points

	tf::TransformListener tf_listener;

	tf::Transform b2c, c2b, c2imu, c2stereo;

	ros::Publisher insight_pub, odom_pub, points_pub;

	bool velocity_set;
	Eigen::Vector3d omega, velocity; // meant to store the last angular and linear velocity

	typedef Eigen::Matrix<double,2,6> Matrix26d;

	VIO();
	virtual ~VIO();

	void camera_callback(const sensor_msgs::ImageConstPtr& img, const sensor_msgs::CameraInfoConstPtr& cam);

	//std::vector<cv::Point2d> getPixelsInOrder(Frame& f);

	std::vector<cv::Point2f> getPixels2fInOrder(Frame& f);

	//std::vector<cv::Point3d> getObjectsInOrder(Frame& f);

	void addFrame(cv::Mat img, cv::Mat_<float> k, ros::Time t);

	void predictPose(Frame& new_frame, Frame& old_frame);

	void updateFeatures(Frame& last_f, Frame& new_f);

	bool MOBA(Frame& f, double& perPixelError, bool useImmature);

	bool optimizePose(Frame& f, double& ppe);

	void optimizePoints(Frame& f);

	void keyFrameUpdate();

	void replenishFeatures(Frame& f);

	void tf2rvecAndtvec(tf::Transform tf, cv::Mat& tvec, cv::Mat& rvec);

	tf::Transform rvecAndtvec2tf(cv::Mat tvec, cv::Mat rvec);

	void publishInsight(Frame& f);

	void publishOdometry(Frame& last_f, Frame& new_f);

	void publishPoints(Frame& f);

	void correctPointers(bool allFrames = false);

	void startTimer(){this->timer_start = ros::Time::now();}
	void stopTimer(std::string prefix){ROS_INFO_STREAM(prefix <<": "<<(ros::Time::now() - this->timer_start).toSec() * 1000 << "ms");}
};

#endif /* PAUVSI_VIO_INCLUDE_PAUVSI_VIO_VIO_H_ */
