/*
 * Point.h
 *
 *  Created on: Jan 29, 2017
 *      Author: pauvsi
 */

#ifndef PAUVSI_VIO_INCLUDE_PAUVSI_VIO_POINT_H_
#define PAUVSI_VIO_INCLUDE_PAUVSI_VIO_POINT_H_

#include <eigen3/Eigen/Geometry>

#include "Feature.h"
#include "KeyFrame.h"

#define DEFAULT_STARTING_SIGMA 1000

#define EPS_SBA 0.0000000001

class Feature;

typedef Eigen::Matrix<double, 2, 3> Matrix23d;

class Point{


private:

	bool deleted;

	double sigma; // the certainty of this point's depth
	Eigen::Vector3d pos; // this is the world coordinate of the point



	std::deque<Feature*> _observations; // this is  a list of observations of this 3d point from different frames


public:

	std::list<Point>::iterator thisPoint; // the iterator of this point in the map
	std::list<Point>* theMap; // a pointer to the map which this point is stored in


	Point();

	Point(Feature* ft);

	void addObservation(Feature* ft);

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

	double getSigma()
	{
		return sigma;
	}

	bool isDeleted()
	{
		return deleted;
	}

	void safelyDeletePoint();

	void initializePoint(tf::Transform transform, Feature* ft, double start_depth);

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

	Eigen::Vector2d toPixel(Eigen::Vector3d in)
	{
		return Eigen::Vector2d(in(0) / in(2), in(1) / in(2));
	}

	void SBA(int iterations);
};




#endif /* PAUVSI_VIO_INCLUDE_PAUVSI_VIO_POINT_H_ */
