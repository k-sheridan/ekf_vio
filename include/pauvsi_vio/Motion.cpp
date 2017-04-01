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
 *
 * it adds a new keyframe is the baseline dist / avg scene depth > Threshold
 * this method was taken from
 * SVO, Forster, Christian and Pizzoli, Matia and Scaramuzza, Davide
 */
void VIO::updateKeyFrameInfo() {


	//clean up old keyframes and excess keyframes
	while(!keyFrames.empty() && (keyFrames.back().frame->finalFrame || keyFrames.size() > MAX_KEYFRAMES))
	{
		ROS_DEBUG("removing an old keyframe");
		ROS_ASSERT(keyFrames.back().frame != NULL);
		keyFrames.back().frame->isKeyframe = false;
		keyFrames.pop_back();
	}

	if(keyFrames.empty())
	{
		keyFrames.push_front(KeyFrame(&currentFrame(), currentFrame().features.size())); // in the case f no keyframes use the current frame
		keyFrames.front().frame->isKeyframe = true;
	}
	else
	{
		ROS_ASSERT(keyFrames.front().frame != NULL);
		double keyFrameRatio = (currentFrame().state.getr() - keyFrames.front().frame->state.getr()).norm() / ((currentFrame().avgSceneDepth + keyFrames.front().frame->avgSceneDepth) / 2);

		if(keyFrameRatio >= NEW_KEYFRAME_RATIO)
		{
			keyFrames.push_front(KeyFrame(&currentFrame(), currentFrame().features.size())); // in the case f no keyframes use the current frame
			keyFrames.front().frame->isKeyframe = true;

			// optimize the position of each point in the frame currently
			ROS_INFO("PERFORMING BA ON ALL POINTS");
			for(auto e : currentFrame().features)
			{
				e.point->SBA(10);
			}
		}

	}
}

tf::Transform VIO::cameraTransformFromState(VIOState x, tf::Transform b2c) {
	return tf::Transform(x.getTFQuaternion(), tf::Vector3(x.x(), x.y(), x.z())) * b2c;
}

VIOState VIO::transformState(VIOState x, tf::Transform trans) {
	tf::Transform transformed = cameraTransformFromState(x, trans);

	ROS_DEBUG_STREAM("cameraTransform from inside " << transformed.getOrigin().x() << ", " << transformed.getOrigin().y() << ", " << transformed.getOrigin().z());

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


void VIO::optimizePose(int iterations, VIOState initialGuess)
{

}

/*
 * motion only bundle adjustment
 * This seems to fail.
 */
void VIO::optimizePoseG2O(int iterations, VIOState initialGuess)
{

	ROS_INFO("SETTING UP MOTION ONLY BA");

	ROS_DEBUG_STREAM("given guess: " << initialGuess.getr());

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

	VIOState x_currentfFrame = transformState(initialGuess, b2c);

	//setup vertex 2 aka current Frame
	const int CURRENTFRAME_VERTEX_ID = 0;

	g2o::VertexSE3Expmap * cf_vertex = new g2o::VertexSE3Expmap(); // create a new vertex for this frame;

	cf_vertex->setId(CURRENTFRAME_VERTEX_ID); // the id of the first current frame is 1

	ROS_DEBUG_STREAM("current pos guess " << x_currentfFrame.getr());

	cf_vertex->setEstimate(g2o::SE3Quat(x_currentfFrame.getQuaternion(), x_currentfFrame.getr())); // set the estimate of this frame
	// if this is structure only the vertex is fixed
	// otherwise it is not fixed and can be optimized
	cf_vertex->setFixed(false); // the current frame's position is not fixed

	optimizer.addVertex(cf_vertex); // add the vertex to the problem

	// now the camera vertices are part of the problem
	const int POINTS_STARTING_ID = 1;

	int point_id = POINTS_STARTING_ID; // start the next vertex ids from 2
	int number_of_points = 0;

	ROS_DEBUG("camera vertex has been setup");
	// setup the rest of the graph optimization problem
	//TODO setup points

	for(auto& ft : currentFrame().features)
	{
		//skip this feature if point is NULL
		if(ft.point == NULL || ft.point->getSigma() > MAX_POINT_SIGMA)
		{
			continue;
		}


		g2o::VertexSBAPointXYZ * v_p = new g2o::VertexSBAPointXYZ(); // create a vertex for this 3d point

		v_p->setId(point_id); // set the vertex's id
		v_p->setMarginalized(true);
		v_p->setFixed(true);
		v_p->setEstimate(ft.point->getWorldCoordinate()); // set the initial estimate

		number_of_points++;

		optimizer.addVertex(v_p); // add this point to the problem
		ROS_DEBUG_STREAM("added this 3d point: " << v_p->estimate());

		// now we must set up the edges between these three vertices
		g2o::EdgeProjectXYZ2UV * e1 = new g2o::EdgeProjectXYZ2UV(); // here is the first edge

		e1->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(v_p)); // set the 3d point
		e1->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(cf_vertex)); // set the camera
		e1->setMeasurement(ft.getUndistortedMeasurement()); //[u, v]
		e1->setParameterId(0, 0); // set this edge to the camera params

		if(ROBUST_HUBER)
		{
			g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
			//TODO set the huber delta for each edge
			ROS_DEBUG_STREAM("huber width: " << rk->delta());
			e1->setRobustKernel(rk);
		}

#if SUPER_DEBUG
		Eigen::Affine3d _tf;
		tf::Transform temp = cameraTransformFromState(initialGuess, b2c);
		currentFrame().tfTransform2EigenAffine(temp, _tf);

		//ROS_DEBUG_STREAM("my chi: " << (ft.point->toPixel(_tf * ft.point->getWorldCoordinate()) - ft.getUndistortedMeasurement()).squaredNorm());
		ROS_DEBUG_STREAM("my chi: " << (ft.point->toPixel(lastFrame().transform_frame_to_world * ft.point->getWorldCoordinate()) - ft.getUndistortedMeasurement()).squaredNorm());
		ROS_DEBUG_STREAM("point's sigma " << ft.point->getSigma());
#endif

		optimizer.addEdge(e1); // finally add the edge

		point_id++;
	}


	if(point_id < 3)
	{
		ROS_DEBUG("too few good 3d points for motion only BA");
		return;
	}

	optimizer.setVerbose(true);
	ROS_DEBUG("preparing to initilize g2o");
	bool initStatus = optimizer.initializeOptimization(); // set up the problem for optimization
	optimizer.computeActiveErrors(); // compute the current errors
	ROS_WARN_STREAM_COND(!initStatus, "something went wrong when initializing the bundle adjustment problem");
	ROS_DEBUG("initilized g2o");


	ROS_DEBUG_STREAM("pos before " << cf_vertex->estimate().translation());
	ROS_DEBUG_STREAM("BEGINNING MOTION ONLY OPTIMIZATION - INITIAL ERROR: " << optimizer.activeChi2());
	optimizer.optimize(iterations);
	ROS_DEBUG_STREAM("ERROR AFTER OPTIMIZATION: " << optimizer.activeChi2());
	ROS_DEBUG_STREAM("pos after " << cf_vertex->estimate().translation());


}






