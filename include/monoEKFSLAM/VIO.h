/*
 * FeatureTracker.h
 *
 *  Created on: Jul 8, 2017
 *      Author: kevin
 */

#ifndef PAUVSI_VIO_INCLUDE_PAUVSI_VIO_VIO_H_
#define PAUVSI_VIO_INCLUDE_PAUVSI_VIO_VIO_H_

#include <ros/ros.h>

#include <cmath>

#include <opencv2/highgui.hpp>
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
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <tf/tfMessage.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

#include <sophus/se3.hpp>

#include <Feature.h>
#include <Frame.h>
#include <Params.h>
#include <TightlyCoupledEKF.h>
#include <KLTTracker.h>

#include <Eigen/Eigenvalues>

class VIO {
public:


	bool initialized; // has the program been initialized
	bool tracking_lost; // do we still have tracking

	std::deque<Frame> frame_buffer; // stores all frames and pose estimates at this frames

	struct UpdatedState{
		TightlyCoupledEKF state;
		sensor_msgs::Imu msg;
	};

	std::deque<UpdatedState> imu_update_buffer; // store states updated with IMU readings to reduce latency

	//std::deque<sensor_msgs::Imu> imu_buffer; // store imu messages 

	tf::TransformListener tf_listener;

	TightlyCoupledEKF tc_ekf;

	KLTTracker tracker;

	tf::Transform b2c, c2b, c2imu, c2stereo;

	ros::Subscriber imu_sub;

	ros::Publisher insight_pub, insight_cinfo_pub, odom_pub, points_pub;

	VIO();

	void imu_callback(const sensor_msgs::ImuConstPtr& msg);

	void camera_callback(const sensor_msgs::ImageConstPtr& img, const sensor_msgs::CameraInfoConstPtr& cam);

	void addFrame(Frame f);

	void removeExcessFrames(std::deque<Frame>& buffer);

	void replenishFeatures(Frame& f);

	void updateStateWithNewImage(Frame& lf, Frame& cf);

	void publishInsight(Frame& f);

	void publishPoints(Frame& f);

	void publishOdometry(Frame& cf);

	cv::RotatedRect getErrorEllipse(double chisquare_val, cv::Point2f mean, Eigen::Matrix2f covmat);
};

#endif /* PAUVSI_VIO_INCLUDE_PAUVSI_VIO_VIO_H_ */
