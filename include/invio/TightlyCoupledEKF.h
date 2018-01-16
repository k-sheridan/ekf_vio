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

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/LU>
#include <Eigen/SparseCholesky>
#include <Eigen/Cholesky>

class TightlyCoupledEKF {
public:
	TightlyCoupledEKF();
	virtual ~TightlyCoupledEKF();

	Eigen::Matrix<float, BASE_STATE_SIZE, 1> base_mu; // this is the part of the state which always remains. It contains position, vel, orientation etc.

	std::list<Feature> features; // stores the current best guess about the state of the features

	//must always be kept in sync with the feature vector
	Eigen::MatrixXf Sigma; // stores the current uncertainty and correlations for the state (including feature positions)

	ros::Time t; // store the current time of the state

	void initializeBaseState();

	void addNewFeatures(std::vector<Eigen::Vector2f> new_homogenous_features);

	std::vector<Eigen::Vector2f> previousFeaturePositionVector();

	void process(double dt);

	void updateWithFeaturePositions(std::vector<Eigen::Vector2f> measured_positions, std::vector<Eigen::Matrix2f> estimated_covariance, std::vector<bool> pass);

	Eigen::SparseMatrix<float> formFeatureMeasurementMap(std::vector<bool> measured);

	Eigen::Matrix2f getFeatureHomogenousCovariance(int index);
	float getFeatureDepthVariance(int index);

	void setFeatureHomogenousCovariance(int index, Eigen::Matrix2f cov);

	Eigen::SparseMatrix<float> getMetric2PixelMap(Eigen::Matrix3f& K);
	Eigen::SparseMatrix<float> getPixel2MetricMap(Eigen::Matrix3f& K);

};

#endif /* INVIO_INCLUDE_INVIO_TIGHTLYCOUPLEDEKF_H_ */
