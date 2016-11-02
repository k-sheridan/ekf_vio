/*
 * VisualMeasurment.hpp
 *
 *  Created on: Nov 2, 2016
 *      Author: kevin
 */

#ifndef PAUVSI_M7_PAUVSI_VIO_INCLUDE_PAUVSI_VIO_VISUALMEASURMENT_HPP_
#define PAUVSI_M7_PAUVSI_VIO_INCLUDE_PAUVSI_VIO_VISUALMEASURMENT_HPP_


class VisualMeasurment
{
public:
	Eigen::Matrix<double, 7, 1> z; // dx, dy, dz, q0, q1, q2, q3
	Eigen::Matrix<double, 7, 7> covariance;
	ros::Time t;

	VisualMeasurment();
};


#endif /* PAUVSI_M7_PAUVSI_VIO_INCLUDE_PAUVSI_VIO_VISUALMEASURMENT_HPP_ */
