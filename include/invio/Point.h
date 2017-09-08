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

	double sigma; // the certainty of this point's depth
	Eigen::Vector3d pos; // this is the world coordinate of the point

	bool immature;


	std::deque<Feature*> _observations; // this is  a list of observations of this 3d point from different frames


	std::list<Point>::iterator thisPoint; // the iterator of this point in the map
	std::list<Point>* theMap; // a pointer to the map which this point is stored in

public:


	bool guessed;

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
		return this->pos;
	}

	tf::Vector3 getWorldCoordinateTF()
	{
		ROS_ASSERT(!deleted);
		return tf::Vector3(this->pos.x(), this->pos.y(), this->pos.z());
	}

	std::deque<Feature*>& getObservations(){return _observations;}

	/*
	 *
	 * externally setting the position of a point automatically makes is immature
	 * sets the sigma of the point to the default
	 */
	void setPosition(Eigen::Vector3d pos)
	{
		this->immature = true;
		this->sigma = DEFAULT_POINT_STARTING_ERROR;
		this->pos = pos;
	}

	std::list<Point>* getMap(){ROS_ASSERT(theMap != NULL); return theMap;}

	std::list<Point>::iterator getPointIterator(){
		//ROS_ASSERT(thisPoint != 0);
		return thisPoint;}

	bool isImmature(){return immature;}
	void setImmature(bool val){immature = val;}

	double getSigma()
	{
		return sigma;
	}

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

	bool SBA(int iterations);
};




#endif /* PAUVSI_VIO_INCLUDE_PAUVSI_VIO_POINT_H_ */
