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

	bool initialized; // has the program been initialized
	bool tracking_lost; // do we still have tracking

	std::list<Frame> frame_buffer; // stores all frames and pose estimates at this frames

	std::list<Point> map; // this is a list of all active 3d points

	tf::TransformListener tf_listener;

	tf::Transform b2c, c2b, c2imu, c2stereo;

	ros::Publisher insight_pub, odom_pub, points_pub;

	typedef Eigen::Matrix<double,2,6> Matrix26d;

	VIO();
	virtual ~VIO();

	void camera_callback(const sensor_msgs::ImageConstPtr& img, const sensor_msgs::CameraInfoConstPtr& cam);

	//std::vector<cv::Point2d> getPixelsInOrder(Frame& f);

	std::vector<cv::Point2f> getPixels2fInOrder(Frame& f);

	//std::vector<cv::Point3d> getObjectsInOrder(Frame& f);

	void addFrame(cv::Mat img, cv::Mat_<float> k, ros::Time t);

	void updateFeatures(Frame& last_f, Frame& new_f);

	bool MOBA(Frame& f, double& perPixelError, bool useImmature);

	bool optimizePose(Frame& f, double& ppe);

	void replenishFeatures(Frame& f);

	void tf2rvecAndtvec(tf::Transform tf, cv::Mat& tvec, cv::Mat& rvec);

	tf::Transform rvecAndtvec2tf(cv::Mat tvec, cv::Mat rvec);

	void publishInsight(Frame& f);

	void publishOdometry(Frame& last_f, Frame& new_f);

	void correctPointers(bool allFrames = false);
};

#endif /* PAUVSI_VIO_INCLUDE_PAUVSI_VIO_VIO_H_ */
