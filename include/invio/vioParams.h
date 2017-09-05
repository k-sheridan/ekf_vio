/*
 * vioParams.h
 *
 *  Created on: Jul 31, 2017
 *      Author: kevin
 */

#ifndef PAUVSI_VIO_INCLUDE_PAUVSI_VIO_VIOPARAMS_H_
#define PAUVSI_VIO_INCLUDE_PAUVSI_VIO_VIOPARAMS_H_

//#define ROSCONSOLE_MIN_SEVERITY ROSCONSOLE_SEVERITY_DEBUG

#define SUPER_DEBUG false

#define D_PUBLISH_INSIGHT true
#define D_INSIGHT_TOPIC "invio/insight"

//VISUAL ODOM
//fast corner detector for planar odometry
#define D_FAST_THRESHOLD 100
// the amount to blur the image before feature extraction
#define D_FAST_BLUR_SIGMA 0.0

#define D_INVERSE_IMAGE_SCALE 4

// use previous odometry for prior
#define D_USE_ODOM_PRIOR true

//analyze the function times
#define D_ANALYZE_RUNTIME true

// the radius to remove features at in pixels
#define D_KILL_BOX_WIDTH 110
#define D_KILL_BOX_HEIGHT 110

// the minimum feature eigen val where it is determined as lost
#define D_KLT_MIN_EIGEN 1e-4

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
#define D_FRAME_BUFFER_SIZE 200

// the number of times a feature must be observed before allowed to be optimized
#define D_KEYFRAME_COUNT_FOR_OPTIMIZATION 5

// the minimum ratio of translation to avg scene depth
#define D_T2ASD 0.05

// the maximum error a feature can have after an optim
#define D_MAXIMUM_FEATURE_DEPTH_ERROR 0.01

// default point depth used for initialization in meters
#define D_DEFAULT_POINT_DEPTH 1.0
#define D_DEFAULT_POINT_STARTING_ERROR 10000

// epsilon for convergence in structure bundle adjustment and motion
#define D_EPS_SBA 0.0001
#define D_EPS_MOBA 0.0001

//max iterations for gausss newton
#define D_MOBA_MAX_ITERATIONS 10
#define D_SBA_MAX_ITERATIONS 10

//min and maximum point depths in meters
#define D_MAX_POINT_Z 10
#define D_MIN_POINT_Z 0.02

//OUTLIER DETECTION

//if the ppe of our planar odometry exceeds this value we have lost odometry
#define D_MAXIMUM_VO_PPE 7.0

//END VISUAL ODOM

#define D_ODOM_TOPIC "invio/odom"
#define D_POINTS_TOPIC "invio/points"

// this topic will serve as a last resort for realignment
#define D_POINTCLOUD_TOPIC "guidance/points2"
#define D_POINTCLOUD_FRAME "guidance"

//#define CAMERA_TOPIC "/guidance/left/image_rect"
//#define CAMERA_FRAME "guidance"
#define D_CAMERA_TOPIC "/bottom_camera/image_rect"
#define D_CAMERA_FRAME "bottom_camera"

#define D_BASE_FRAME "base_link"

#define D_WORLD_FRAME "world"


#endif /* PAUVSI_VIO_INCLUDE_PAUVSI_VIO_VIOPARAMS_H_ */
