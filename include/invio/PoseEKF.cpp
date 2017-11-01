/*
 * PoseEKF.cpp
 *
 *  Created on: Oct 31, 2017
 *      Author: kevin
 */

#include <PoseEKF.h>

PoseEKF::PoseEKF() {
	// TODO Auto-generated constructor stub
	this->resetState();
}

PoseEKF::PoseEKF(ros::Time start)
{
	this->state.t = start;
	this->resetState();
}

PoseEKF::~PoseEKF() {
	// TODO Auto-generated destructor stub
}

/*
 * set the initial state and initial uncertainties
 */
void PoseEKF::resetState()
{

}

/*
 * this predicts the state forward in time and updates the uncertainties and correlations due to prediction uncertainty
 */
void PoseEKF::process(ros::Time to_time)
{
	double dt = (to_time - this->state.t).toSec();
	ROS_ASSERT(dt >= 0);

	// first compute the linearized transition function at the previous state
	Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> F;
	this->computeStateTransitionJacobian(this->state, dt, F);

	// now we can predict the state forward with the nonlinear update function

	//r = r + R(q)*((1/lambda)(dt*b_r' + 0.5*dt^2*b_r'')) { this is where the scale is estimated }

	//q = q*d_q(b_w*dt)

	//lambda^(-1) = lambda^(-1)

	//b_r' = b_r' + dt*b_dr''

	//b_w = b_w

	//b_r'' = b_r''

	//g = d_q(b_w*dt)*g

	//biases = biases
}

/*
 * need to convert the euler covariance matrix to a quaternion covariance matrix before update can
 * be performed. This is a completely linear process other wise
 */
void PoseEKF::updateWithVOPose(Sophus::SE3d pose, Eigen::Matrix<double, 6, 6> cov, ros::Time t_measured)
{

}

/*
 * due to the scale factor, and quaternion, and body frame velocity... this is not a linear process
 */
void PoseEKF::computeStateTransitionJacobian(State& from_state, double dt, Eigen::Matrix<double, STATE_SIZE, STATE_SIZE>& F)
{

}

