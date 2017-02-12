/*
 * vioMotion.cpp
 *
 *  Created on: Jan 16, 2017
 *      Author: kevin
 */

#include "vio.h"

void VIO::updateKeyFrameInfo() {
	static KeyFrame candidate;

	if (keyFrames.size() < 1) {
		KeyFrame kf;
		kf.frame = &this->frameBuffer.at(frameBuffer.size() - 2);
		keyFrames.push_back(kf);
	} else {
		keyFrames.at(0).frame = &this->frameBuffer.at(frameBuffer.size() - 2);
	}
}

tf::Transform VIO::cameraTransformFromState(VIOState x, tf::Transform b2c) {
	return tf::Transform(x.getTFQuaternion(), tf::Vector3(x.x(), x.y(), x.z()))
			* b2c;
}

VIOState VIO::transformState(VIOState x, tf::Transform trans) {
	tf::Transform tf_base = tf::Transform(
			tf::Quaternion(x.q1(), x.q2(), x.q3(), x.q0()),
			tf::Vector3(x.x(), x.y(), x.z()));
	tf::Transform transformed = tf_base * trans;

	tf::Quaternion newQ = transformed.getRotation();

	x.vector(0, 0) = transformed.getOrigin().x();
	x.vector(1, 0) = transformed.getOrigin().y();
	x.vector(2, 0) = transformed.getOrigin().z();

	x.vector(6, 0) = newQ.w();
	x.vector(7, 0) = newQ.x();
	x.vector(8, 0) = newQ.y();
	x.vector(9, 0) = newQ.z();

	return x;
}

void VIO::structureOnlyBundleAdjustment(Frame& cf, KeyFrame& kf) {
	g2o::SparseOptimizer optimizer; // this is the g2o optimizer which ultimately solves the problem
	optimizer.setVerbose(true); // set the verbosity of the optimizer

	g2o::BlockSolver_6_3::LinearSolverType * linearSolver; //create a linear solver type pointer
	linearSolver = new g2o::LinearSolverCholmod<
			g2o::BlockSolver_6_3::PoseMatrixType>(); // use a cholesky linear solver

	g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver); // finally create the solver

	g2o::OptimizationAlgorithmLevenberg* solver =
			new g2o::OptimizationAlgorithmLevenberg(solver_ptr); // create a LM optimization type using the cholmod solver

	optimizer.setAlgorithm(solver); // add the LM to the optimizer

	// this should contain the rotation and translation from the base of the system to the camera
	tf::StampedTransform b2c;
	try {
		this->ekf.tf_listener.lookupTransform(this->CoM_frame, this->camera_frame,
				ros::Time(0), b2c);
	} catch (tf::TransformException& e) {
		ROS_WARN_STREAM(e.what());
	}

	VIOState x_currentfFrame = transformState(cf.state, b2c);
	VIOState x_keyFrame = transformState(kf.frame->state, b2c);

	double baseline_val = (cf.state.getr() - kf.frame->state.getr()).norm(); // this is the estimated distance between the two frames

	g2o::VertexSCam::setKcam(cf.K.at<double>(0, 0), cf.K.at<double>(1, 1),
			cf.K.at<double>(0, 2), cf.K.at<double>(1, 3), baseline_val); // set up the camera parameters

}

