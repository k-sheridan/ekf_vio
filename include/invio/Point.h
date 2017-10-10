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

	double variance; // the uncertainty of this point's depth

	double depth; // depth of point in first frame in meters

	Eigen::Vector3d initial_homogenous_pixel; // the homogenous pixel where the feature was first observed

	Sophus::SE3d initial_camera_pose; // the w2c pose when the feature was first observed

	double max_depth, min_depth; // the maximum and minumum observed depths. used for outlier detection



	std::deque<Feature*> _observations; // this is  a list of observations of this 3d point from different frames


	std::list<Point>::iterator thisPoint; // the iterator of this point in the map
	std::list<Point>* theMap; // a pointer to the map which this point is stored in

public:


	int frames_since_depth_update; // keeps track of the frame count since this point has last been updated

	double last_update_t2d; // the translation to depth ratio last used for a depth update

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
		return this->initial_camera_pose * (this->depth * this->initial_homogenous_pixel);
	}

	std::deque<Feature*>& getObservations(){return _observations;}

	std::list<Point>* getMap(){ROS_ASSERT(theMap != NULL); return theMap;}

	std::list<Point>::iterator getPointIterator(){
		//ROS_ASSERT(thisPoint != 0);
		return thisPoint;}

	bool isImmature(){return immature;}
	void setImmature(bool val){immature = val;}

	double getVariance()
	{
		return variance;
	}

	double setVariance(double var){
		this->variance = var;
	}

	double getDepth(){
		return depth;
	}

	double getRangePerDepth(){
		ROS_ASSERT(this->depth != 0);
		return (this->max_depth - this->min_depth) / this->depth;
	}

	/*
	 * the coord at initial obs
	 */
	Eigen::Vector3d getInitialHomogenousCoordinate(){
		return initial_homogenous_pixel;
	}

	/*
	 * the pose at first obs
	 */
	Sophus::SE3d getInitialCameraPose(){
		return initial_camera_pose;
	}

	void setDepth(double d){
		this->depth = d;
	}

	void updateDepth(double measurement, double in_variance);

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
