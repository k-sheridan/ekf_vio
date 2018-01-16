/*
 * vioParams.h
 *
 *  Created on: Jul 31, 2017
 *      Author: kevin
 */

#ifndef PAUVSI_VIO_INCLUDE_PAUVSI_VIO_VIOPARAMS_H_
#define PAUVSI_VIO_INCLUDE_PAUVSI_VIO_VIOPARAMS_H_

//#define ROSCONSOLE_MIN_SEVERITY ROSCONSOLE_SEVERITY_DEBUG

#include <ros/ros.h>
#include <string.h>

#define SUPER_DEBUG false

#define D_PUBLISH_INSIGHT true
#define D_INSIGHT_TOPIC "invio/insight"
#define D_INSIGHT_CINFO_TOPIC "invio/camera_info"

//VISUAL ODOM
//fast corner detector for planar odometry
#define D_FAST_THRESHOLD 50
// the amount to blur the image before feature extraction
#define D_FAST_BLUR_SIGMA 0.0

#define D_INVERSE_IMAGE_SCALE 4

//analyze the function times
#define D_ANALYZE_RUNTIME true

#define D_KILL_PAD 11

// the minimum feature eigen val where it is determined as lost
#define D_KLT_MIN_EIGEN 1e-4

#define D_HUBER_WIDTH 1e-5

#define D_BORDER_WEIGHT_EXPONENT 10

// the minimum pixel distance a feature must have between a previous feature
#define D_MIN_NEW_FEATURE_DIST 30

// the desired number of features more = robust...er (and slower)
#define D_NUM_FEATURES 100

// the amount of points needed to start pauvsi vio odometry
#define D_START_FEATURE_COUNT 20

// the minimum amount of mature features which is deemed dangerous
#define D_DANGEROUS_MATURE_FEATURE_COUNT_LEVEL 10

//minimum detected features before vo has failed
#define D_MINIMUM_TRACKABLE_FEATURES 4

// the amount of frames to store for feature optimization
#define D_FRAME_BUFFER_SIZE 2

// the number of times a feature must be observed before allowed to be optimized
#define D_MINIMUM_KEYFRAME_COUNT_FOR_OPTIMIZATION 4
#define D_MAXIMUM_KEYFRAME_COUNT_FOR_OPTIMIZATION 7

// the minimum ratio of translation to avg scene depth
#define D_MIN_T2D 0.1

// the maximum number of features whose depth will be updated each frame
#define D_MAX_DEPTH_UPDATES_PER_FRAME 10

// the variance when a point becomes a motion estimation candidate
#define D_MOBA_CANDIDATE_VARIANCE 0.2


//OUTLIER DETECTION

// the maximum error a feature can have after moba
#define D_MAXIMUM_REPROJECTION_ERROR 0.0005

// the maximum reprojection error a candidate can have before it is allowed into the moba problem
#define D_MAXIMUM_CANDIDATE_REPROJECTION_ERROR 0.00005

// default point depth used for initialization in meters
#define D_DEFAULT_POINT_DEPTH 0.5
#define D_DEFAULT_POINT_DEPTH_VARIANCE 100

#define D_DEFAULT_POINT_HOMOGENOUS_VARIANCE 0.0005 // in meters!

// epsilon for convergence in structure bundle adjustment and motion
#define D_EPS_SBA 0.0000000001
#define D_EPS_MOBA 0.000001

#define D_MINIMUM_DEPTH_DETERMINANT 0.001

//max iterations for gausss newton
#define D_MOBA_MAX_ITERATIONS 10
#define D_SBA_MAX_ITERATIONS 10

//min and maximum point depths in meters
#define D_MAX_POINT_Z 10
#define D_MIN_POINT_Z 0.02

//KLT
#define D_MAX_PYRAMID_LEVEL 3
#define D_WINDOW_SIZE 21

//END VISUAL ODOM

#define D_MAX_VARIANCE_SIZE 40
#define D_MIN_VARIANCE_SIZE 5

#define D_ODOM_TOPIC "invio/odom"
#define D_POINTS_TOPIC "invio/points"
#define D_ODOM_FRAME "invio_odom"

//#define CAMERA_TOPIC "/guidance/left/image_rect"
//#define CAMERA_FRAME "guidance"
#define D_CAMERA_TOPIC "/camera/image_rect"
#define D_CAMERA_FRAME "camera"

#define D_BASE_FRAME "base_link"

#define D_WORLD_FRAME "world"

#define D_USE_IMU true
#define D_IMU_TOPIC "imu/measurement"
#define D_IMU_FRAME "imu"

//ROS PARAMS
bool PUBLISH_INSIGHT;
std::string INSIGHT_TOPIC;
std::string INSIGHT_CINFO_TOPIC;

int MAX_VARIANCE_SIZE, MIN_VARIANCE_SIZE;

//VISUAL ODOM
//fast corner detector for planar odometry
int FAST_THRESHOLD;
// the amount to blur the image before feature extraction
double FAST_BLUR_SIGMA;

double INVERSE_IMAGE_SCALE;

//analyze the function times
bool ANALYZE_RUNTIME;

// the distance from the image border where a feature is marginalized
int KILL_PAD;

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
int MINIMUM_KEYFRAME_COUNT_FOR_OPTIMIZATION;
int MAXIMUM_KEYFRAME_COUNT_FOR_OPTIMIZATION;

// the minimum ratio of translation to avg scene depth
double MIN_T2D;

// the maximum number of features whose depth will be updated each frame
double MAX_DEPTH_UPDATES_PER_FRAME;

// the maximum error a feature can have after an optim
// if the error for a point is greater than this after an optimization it is deleted (to remove possible outliers)
double MAXIMUM_REPROJECTION_ERROR;

// the maximum reprojection error a candidate can have before it is allowed into the moba problem
double MAXIMUM_CANDIDATE_REPROJECTION_ERROR;


// the variance when a point becomes a motion estimation candidate
double MOBA_CANDIDATE_VARIANCE;

// default point depth used for initialization in meters
double DEFAULT_POINT_DEPTH;
double DEFAULT_POINT_DEPTH_VARIANCE;

// initially this is a fairly certain value, but not infinitely certain. we allow the homogeneous part to drift slightly with more measurements
double DEFAULT_POINT_HOMOGENOUS_VARIANCE;

// epsilon for convergence in structure bundle adjustment and motion
double EPS_SBA;
double EPS_MOBA;

// the huber width for robust moba
double HUBER_WIDTH;

// the exponent for the border weight function
double BORDER_WEIGHT_EXPONENT;

// the minimum determinant of the linear systems used to determine the depth of the first observation
double MINIMUM_DEPTH_DETERMINANT;

//max iterations for gausss newton
int MOBA_MAX_ITERATIONS;
int SBA_MAX_ITERATIONS;

//min and maximum point depths in meters
double MAX_POINT_Z;
double MIN_POINT_Z;

double MAX_RANGE_PER_DEPTH;

int WINDOW_SIZE;
int MAX_PYRAMID_LEVEL;

//END VISUAL ODOM

std::string ODOM_TOPIC;
std::string POINTS_TOPIC;
std::string ODOM_FRAME;

//double CAMERA_TOPIC "/guidance/left/image_rect"
std::string CAMERA_FRAME;
std::string CAMERA_TOPIC;

std::string BASE_FRAME;

std::string WORLD_FRAME;

bool USE_IMU;
std::string IMU_TOPIC;
std::string IMU_FRAME;
//END ROS PARAMS

void parseROSParams()
{
	ros::param::param<bool>("~publish_insight", PUBLISH_INSIGHT, D_PUBLISH_INSIGHT);
	ros::param::param<std::string>("~insight_topic", INSIGHT_TOPIC, D_INSIGHT_TOPIC);
	ros::param::param<std::string>("~insight_camera_info_topic", INSIGHT_CINFO_TOPIC, D_INSIGHT_CINFO_TOPIC);
	ros::param::param<int>("~fast_threshold", FAST_THRESHOLD, D_FAST_THRESHOLD);
	ros::param::param<double>("~fast_blur_sigma", FAST_BLUR_SIGMA, D_FAST_BLUR_SIGMA);
	ros::param::param<double>("~inverse_image_scale", INVERSE_IMAGE_SCALE, D_INVERSE_IMAGE_SCALE);
	ros::param::param<bool>("~analyze_runtime", ANALYZE_RUNTIME, D_ANALYZE_RUNTIME);
	ros::param::param<int>("~kill_pad", KILL_PAD, D_KILL_PAD);
	ros::param::param<double>("~border_weight_exponent", BORDER_WEIGHT_EXPONENT, D_BORDER_WEIGHT_EXPONENT);
	ros::param::param<double>("~min_klt_eigen_val", KLT_MIN_EIGEN, D_KLT_MIN_EIGEN);
	ros::param::param<double>("~min_new_feature_dist", MIN_NEW_FEATURE_DIST, D_MIN_NEW_FEATURE_DIST);
	ros::param::param<int>("~num_features", NUM_FEATURES, D_NUM_FEATURES);
	ros::param::param<int>("~start_feature_count", START_FEATURE_COUNT, D_START_FEATURE_COUNT);
	ros::param::param<int>("~dangerous_mature_feature_count", DANGEROUS_MATURE_FEATURE_COUNT_LEVEL, D_DANGEROUS_MATURE_FEATURE_COUNT_LEVEL);
	ros::param::param<int>("~minimum_trackable_features", MINIMUM_TRACKABLE_FEATURES, D_MINIMUM_TRACKABLE_FEATURES);
	ros::param::param<int>("~frame_buffer_size", FRAME_BUFFER_SIZE, D_FRAME_BUFFER_SIZE);
	ros::param::param<int>("~minimum_keyframe_count_for_optimization", MINIMUM_KEYFRAME_COUNT_FOR_OPTIMIZATION, D_MINIMUM_KEYFRAME_COUNT_FOR_OPTIMIZATION);
	ros::param::param<int>("~maximum_keyframe_count_for_optimization", MAXIMUM_KEYFRAME_COUNT_FOR_OPTIMIZATION, D_MAXIMUM_KEYFRAME_COUNT_FOR_OPTIMIZATION);
	ros::param::param<double>("~depth_translation_ratio", MIN_T2D, D_MIN_T2D);
	ros::param::param<double>("~max_depth_updates_per_frame", MAX_DEPTH_UPDATES_PER_FRAME, D_MAX_DEPTH_UPDATES_PER_FRAME);
	ros::param::param<double>("~maximum_reprojection_error", MAXIMUM_REPROJECTION_ERROR, D_MAXIMUM_REPROJECTION_ERROR);
	ros::param::param<double>("~moba_candidate_variance", MOBA_CANDIDATE_VARIANCE, D_MOBA_CANDIDATE_VARIANCE);
	ros::param::param<double>("~maximum_candidate_reprojection_error", MAXIMUM_CANDIDATE_REPROJECTION_ERROR, D_MAXIMUM_CANDIDATE_REPROJECTION_ERROR);
	ros::param::param<double>("~default_point_depth", DEFAULT_POINT_DEPTH, D_DEFAULT_POINT_DEPTH);
	ros::param::param<double>("~default_point_depth_variance", DEFAULT_POINT_DEPTH_VARIANCE, D_DEFAULT_POINT_DEPTH_VARIANCE);
	ros::param::param<double>("~default_point_homogenous_variance", DEFAULT_POINT_HOMOGENOUS_VARIANCE, D_DEFAULT_POINT_HOMOGENOUS_VARIANCE);
	ros::param::param<double>("~eps_moba", EPS_MOBA, D_EPS_MOBA);
	ros::param::param<double>("~eps_sba", EPS_SBA, D_EPS_SBA);
	ros::param::param<double>("~huber_width", HUBER_WIDTH, D_HUBER_WIDTH);
	ros::param::param<double>("~minumum_depth_determinant", MINIMUM_DEPTH_DETERMINANT, D_MINIMUM_DEPTH_DETERMINANT);
	ros::param::param<int>("~moba_max_iterations", MOBA_MAX_ITERATIONS, D_MOBA_MAX_ITERATIONS);
	ros::param::param<int>("~sba_max_iterations", SBA_MAX_ITERATIONS, D_SBA_MAX_ITERATIONS);
	ros::param::param<double>("~max_point_z", MAX_POINT_Z, D_MAX_POINT_Z);
	ros::param::param<double>("~min_point_z", MIN_POINT_Z, D_MIN_POINT_Z);
	ros::param::param<std::string>("~odom_topic", ODOM_TOPIC, D_ODOM_TOPIC);
	ros::param::param<std::string>("~odom_frame", ODOM_FRAME, D_ODOM_FRAME);
	ros::param::param<std::string>("~point_topic", POINTS_TOPIC, D_POINTS_TOPIC);
	ros::param::param<std::string>("~camera_topic", CAMERA_TOPIC, D_CAMERA_TOPIC);
	ros::param::param<std::string>("~base_frame", BASE_FRAME, D_BASE_FRAME);
	ros::param::param<std::string>("~world_frame", WORLD_FRAME, D_WORLD_FRAME);
	ros::param::param<std::string>("~camera_frame", CAMERA_FRAME, D_CAMERA_FRAME);
	ros::param::param<std::string>("~imu_topic", IMU_TOPIC, D_IMU_TOPIC);
	ros::param::param<std::string>("~imu_frame", IMU_FRAME, D_IMU_FRAME);
	ros::param::param<bool>("~use_imu", USE_IMU, D_USE_IMU);
	ros::param::param<int>("~min_variance_box_size", MIN_VARIANCE_SIZE, D_MIN_VARIANCE_SIZE);
	ros::param::param<int>("~max_variance_box_size", MAX_VARIANCE_SIZE, D_MAX_VARIANCE_SIZE);
	ros::param::param<int>("~max_pyramids", MAX_PYRAMID_LEVEL, D_MAX_PYRAMID_LEVEL);
	ros::param::param<int>("~klt_window_size", WINDOW_SIZE, D_WINDOW_SIZE);


}

#endif /* PAUVSI_VIO_INCLUDE_PAUVSI_VIO_VIOPARAMS_H_ */
