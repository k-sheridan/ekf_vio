/*
 * Point.h
 *
 *  Created on: Jan 29, 2017
 *      Author: pauvsi
 */

#ifndef PAUVSI_VIO_INCLUDE_PAUVSI_VIO_POINT_H_
#define PAUVSI_VIO_INCLUDE_PAUVSI_VIO_POINT_H_

#include <eigen3/Eigen/Geometry>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <tf/tfMessage.h>


#include "../invio/Feature.h"
#include "../invio/Frame.h"
#include <sophus/se3.hpp>
#include "../invio/vioParams.h"

class Feature;

typedef Eigen::Matrix<double, 2, 3> Matrix23d;

class Point{


private:

	bool deleted;
	bool immature;

	Eigen::Matrix3d Sigma; // sigma of the mu

	Eigen::Vector3d mu; // u (homogeneous), v (homogeneous), depth

	Sophus::SE3d initial_camera_pose; // the w2c pose when the feature was first observed
	Sophus::SE3d initial_camera_pose_inv; // the c2w pose when the feature was first observed

	double max_depth, min_depth; // the maximum and minumum observed depths. used for outlier detection



	std::deque<Feature*> _observations; // this is  a list of observations of this 3d point from different frames


	std::list<Point>::iterator thisPoint; // the iterator of this point in the map
	std::list<Point>* theMap; // a pointer to the map which this point is stored in

public:


	int frames_since_depth_update; // keeps track of the frame count since this point has last been updated

	Sophus::SE3d last_update_pose; // the pose last used for a depth update
	double last_update_pose_depth; // the depth of the feature in the last update pose

	bool guessed; // this should be set if a points depth is a complete guess

	bool moba_candidate; // if the point should be evaluated for moba integration

	double temp_depth;


	Point();

	Point(Feature* ft);

	Point(Feature* ft, std::list<Point>::iterator thisPoint, std::list<Point>* map);

	void addObservation(Feature* ft);

	void setupMapAndPointLocation(std::list<Point>::iterator _thisPoint, std::list<Point>* _map){
		this->theMap = _map;
		ROS_ASSERT(_map != NULL);
		this->thisPoint = _thisPoint;
	}

	std::deque<Feature*>& observations(){
		ROS_ASSERT(!deleted);
		return this->_observations;
	}

	void forceObservationPopBack()
	{
		this->_observations.pop_back();
	}

	Eigen::Vector3d getWorldCoordinate()
	{
		ROS_ASSERT(!deleted);
		return this->initial_camera_pose * Eigen::Vector3d(mu(0) * mu(2), mu(1) * mu(2), mu(2));
	}

	std::deque<Feature*>& getObservations(){return _observations;}

	std::list<Point>* getMap(){ROS_ASSERT(theMap != NULL); return theMap;}

	std::list<Point>::iterator getPointIterator(){
		//ROS_ASSERT(thisPoint != 0);
		return thisPoint;}

	bool isImmature(){return immature;}
	void setImmature(bool val){immature = val;}

	double getDepthVariance()
	{
		return this->Sigma(2, 2);
	}

	void setDepthVariance(double var){
		this->Sigma(2, 2) = var;
	}

	double getDepth(){
		return mu(2);
	}

	double getRange(){
		return (this->max_depth - this->min_depth);
	}

	/*
	 * the coord at initial obs
	 */
	Eigen::Vector3d getInitialHomogenousCoordinate(){
		return Eigen::Vector3d(mu(0), mu(1), 1.0);
	}

	/*
	 * the pose at first obs
	 */
	Sophus::SE3d getInitialCameraPose(){
		return initial_camera_pose;
	}

	Sophus::SE3d getInitialCameraPose_inv(){
		return initial_camera_pose_inv;
	}

	void setDepth(double d){
		this->mu(2) = d;
	}

	void update(Eigen::Vector3d measurement, Eigen::Vector3d sigmas);

	bool isDeleted()
	{
		return deleted;
	}

	void safelyDeletePoint();

	/// Jacobian of point projection on unit plane (focal length = 1) in frame (f).
	// from SVO, Forster et al
	inline static void jacobian_xyz2uv(
			const Eigen::Vector3d& p_in_f,
			const Eigen::Matrix3d& R_f_w,
			Matrix23d& point_jac)
	{
		const double z_inv = 1.0/p_in_f[2];
		const double z_inv_sq = z_inv*z_inv;
		point_jac(0, 0) = z_inv;
		point_jac(0, 1) = 0.0;
		point_jac(0, 2) = -p_in_f[0] * z_inv_sq;
		point_jac(1, 0) = 0.0;
		point_jac(1, 1) = z_inv;
		point_jac(1, 2) = -p_in_f[1] * z_inv_sq;
		point_jac = - point_jac * R_f_w;

	};

	Eigen::Vector3d toHomogenous(Eigen::Vector3d in)
	{
		return Eigen::Vector3d(in(0) / in(2), in(1) / in(2), 1.0);
	}

	inline static Eigen::Vector2d toMetricPixel(Eigen::Vector3d in)
	{
		return Eigen::Vector2d(in(0) / in(2), in(1) / in(2));
	}

	//bool SBA(int iterations);
};




#endif /* PAUVSI_VIO_INCLUDE_PAUVSI_VIO_POINT_H_ */
