/*
 * vioMotion.cpp
 *
 *  Created on: Jan 16, 2017
 *      Author: kevin
 */


#include "vio.h"

void VIO::updateKeyFrameInfo()
{
	static KeyFrame candidate;

	if(keyFrames.size() < 1)
	{
		KeyFrame kf;
		kf.frame = &this->frameBuffer.at(frameBuffer.size() - 2);
		keyFrames.push_back(kf);
	}
	else
	{
		keyFrames.at(0).frame = &this->frameBuffer.at(frameBuffer.size() - 2);
	}
}


tf::Transform VIO::cameraTransformFromState(VIOState x, tf::Transform b2c)
{
	return tf::Transform(x.getTFQuaternion(), tf::Vector3(x.x(), x.y(), x.z())) * b2c;
}

void structureOnlyBundleAdjustment(Frame& cf, KeyFrame& kf)
{
	g2o::SparseOptimizer optimizer; // this is the g2o optimizer which ultimately solves the problem
	optimizer.setVerbose(true); // set the verbosity of the optimizer

	g2o::BlockSolver_6_3::LinearSolverType * linearSolver; //create a linear solver type pointer
	linearSolver = new g2o::LinearSolverCholmod<g2o::BlockSolver_6_3::PoseMatrixType>(); // use a cholesky linear solver

	g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver); // finally create the solver

	g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr); // create a LM optimization type using the cholmod solver

	optimizer.setAlgorithm(solver); // add the LM to the optimizer

	double baseline_val = (cf.state.getr() - kf.frame->state.getr()).norm(); // this is the estimated distance between the two frames

	g2o::VertexSCam::setKcam(cf.K.at<double>(0, 0), cf.K.at<double>(1, 1), cf.K.at<double>(0, 2), cf.K.at<double>(1, 3), baseline_val); // set up the camera parameters

}






