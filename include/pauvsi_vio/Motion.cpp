/*
 * vioMotion.cpp
 *
 *  Created on: Jan 16, 2017
 *      Author: kevin
 */


#include "vio.h"

/*
 * this function will add new keyframes to the list and remove obsolete
 * keyframes from the list
 */
void VIO::updateKeyFrameInfo() {


	//clean up old keyframes and excess keyframes
	while(!keyFrames.empty() && (keyFrames.back().frame->finalFrame || keyFrames.size() > MAX_KEYFRAMES))
	{
		ROS_DEBUG("removing an old keyframe");
		keyFrames.pop_back();
	}

	if(keyFrames.empty())
	{
		keyFrames.push_front(KeyFrame(&currentFrame(), currentFrame().features.size())); // in the case f no keyframes use the current frame
	}
	else
	{

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

/*
 * structure only bundle adjustment between two frames
 */
void VIO::structureOnlyBundleAdjustment(Frame& cf, KeyFrame& kf) {
	g2o::SparseOptimizer optimizer; // this is the g2o optimizer which ultimately solves the problem
	optimizer.setVerbose(true); // set the verbosity of the optimizer

	g2o::BlockSolver_6_3::LinearSolverType * linearSolver; //create a linear solver type pointer
	linearSolver = new g2o::LinearSolverCholmod<g2o::BlockSolver_6_3::PoseMatrixType>(); // use a cholesky linear solver

	g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver); // finally create the solver

	g2o::OptimizationAlgorithmLevenberg* solver =
			new g2o::OptimizationAlgorithmLevenberg(solver_ptr); // create a LM optimization type using the cholmod solver

	solver->setMaxTrialsAfterFailure(5);
	optimizer.setAlgorithm(solver); // add the LM to the optimizer

	//configure the camera parameters
	g2o::CameraParameters * cam_params = new g2o::CameraParameters(1.0, Eigen::Vector2d(0.0, 0.0), 0.0);
	cam_params->setId(0);
	if(!optimizer.addParameter(cam_params)){
		ROS_FATAL("Could not add the camera parameters to g2o");
	}

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

	//ROS_DEBUG_STREAM_ONCE("dx of b2c: " << b2c.getOrigin().getX());
	//double baseline_val = (cf.state.getr() - kf.frame->state.getr()).norm(); // this is the estimated distance between the two frames
	//g2o::VertexSCam::setKcam(1.0, 1.0, 0.0, 0.0, baseline_val); // set up the camera parameters

	// setup vertex 1 aka keyFrame1
	const int KEYFRAME_VERTEX_ID = 0;

	g2o::VertexSE3Expmap * kf_vertex = new g2o::VertexSE3Expmap(); // create a new vertex for this frame;

	kf_vertex->setId(KEYFRAME_VERTEX_ID); // the id of the first keyframe is 0

	kf_vertex->setEstimate(g2o::SE3Quat(x_keyFrame.getQuaternion(), x_keyFrame.getr())); // set the estimate of this frame
	kf_vertex->setFixed(true); // the keyframe's position is fixed

	optimizer.addVertex(kf_vertex); // add this vertex to the optimizer

	//setup vertex 2 aka current Frame
	const int CURRENTFRAME_VERTEX_ID = 1;

	g2o::VertexSE3Expmap * cf_vertex = new g2o::VertexSE3Expmap(); // create a new vertex for this frame;

	cf_vertex->setId(CURRENTFRAME_VERTEX_ID); // the id of the first current frame is 1

	cf_vertex->setEstimate(g2o::SE3Quat(x_currentfFrame.getQuaternion(), x_currentfFrame.getr())); // set the estimate of this frame
	// if this is structure only the vertex is fixed
	// otherwise it is not fixed and can be optimized
	cf_vertex->setFixed(true); // the current frame's position is fixed if structure only

	optimizer.addVertex(cf_vertex); // add the vertex to the problem

	// now the camera vertices are part of the problem
	const int POINTS_STARTING_ID = 2;

	int point_id = POINTS_STARTING_ID; // start the next vertex ids from 2
	int number_of_points = 0;

	ROS_DEBUG("camera vertices have been setup");
	// setup the rest of the graph optimization problem
	ROS_DEBUG_STREAM("features size: " << kf.frame->features.size() << " cf addr: " << &currentFrame());

	for(auto& kf_ft : kf.frame->features)
	{
		//check if the 3d point is still being tracked

		if(kf_ft.point->isDeleted() != true)
		{
			ROS_DEBUG_STREAM("point ID : " << point_id);
			//ROS_DEBUG_STREAM("point address: " << kf_ft.point << " point init: " << kf_ft.point->initialized());
			ROS_DEBUG_STREAM("newest feature's frame index " << kf_ft.point->observations().front()->frame);

			g2o::VertexSBAPointXYZ * v_p = new g2o::VertexSBAPointXYZ(); // create a vertex for this 3d point

			v_p->setId(point_id); // set the vertex's id
			v_p->setMarginalized(true);
			v_p->setEstimate(kf_ft.point->getWorldCoordinate()); // set the initial estimate

			number_of_points++;

			optimizer.addVertex(v_p); // add this point to the problem
			ROS_DEBUG_STREAM("added this 3d point: " << v_p->estimate() << " with dimension " << v_p->Dimension);
			ROS_DEBUG_STREAM("supposed to be " << kf_ft.point->getWorldCoordinate());

			// now we must set up the edges between these three vertices
			g2o::EdgeProjectXYZ2UV * e1 = new g2o::EdgeProjectXYZ2UV(); // here is the first edge

			e1->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(v_p)); // set the 3d point
			e1->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(kf_vertex)); // set the camera
			e1->setMeasurement(kf_ft.getUndistortedMeasurement()); //[u, v]
			e1->setParameterId(0, 0); // set this edge to the camera params

			if(ROBUST_HUBER)
			{
				g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
				//TODO set the huber delta for each edge
				e1->setRobustKernel(rk);
			}

			optimizer.addEdge(e1); // finally add the edge

			g2o::EdgeProjectXYZ2UV * e2 = new g2o::EdgeProjectXYZ2UV(); // here is the first edge

			e2->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(v_p)); // set the 3d point
			e2->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(cf_vertex)); // set the camera
			// get the corresponding 2d feature linked to this 3d point in the current frame
			e2->setMeasurement(kf_ft.point->observations().front()->getUndistortedMeasurement()); //[u, v]
			e2->setParameterId(0, 0); // using the camera params

			if(ROBUST_HUBER)
			{
				g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
				e2->setRobustKernel(rk);
			}

			optimizer.addEdge(e2); // finally add the edge

			ROS_DEBUG("at end of point vertex setup");
		}
		else
		{
			ROS_WARN("there was a null point");
			continue;
		}
		point_id++; // increment the point vertex id's
	}

	ROS_DEBUG("preparing to initilize g2o");
	bool initStatus = optimizer.initializeOptimization(); // set up the problem for optimization
	ROS_WARN_STREAM_COND(!initStatus, "something went wrong when initializing the bundle adjustment problem");
	ROS_DEBUG("initilized g2o");

	//optimizer.setVerbose(true);


}






