/*
 * TightlyCoupledEKF.h
 *
 *  Created on: Nov 25, 2017
 *      Author: kevin
 */

#ifndef INVIO_INCLUDE_INVIO_TIGHTLYCOUPLEDEKF_H_
#define INVIO_INCLUDE_INVIO_TIGHTLYCOUPLEDEKF_H_

// x, y, z, qw, qx, qy, qz, b_dx, b_dy, b_dz, b_wx, b_wy, b_wz, b_ax, b_ay, b_az, baccx, baccy, baccz, bgyrx, bgyry, bgyrz
#define BASE_STATE_SIZE 22

#include <Feature.h>

class TightlyCoupledEKF {
public:
	TightlyCoupledEKF();
	virtual ~TightlyCoupledEKF();

	Eigen::Matrix<double, BASE_STATE_SIZE, 1> base_mu; // this is the part of the state which always remains. It contains position, vel, orientation etc.

	std::list<Feature> features;

	Eigen::MatrixXd Sigma; // stores the current uncertainty and correlations for the state (including feature positions)
};

#endif /* INVIO_INCLUDE_INVIO_TIGHTLYCOUPLEDEKF_H_ */
