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

void VIO::twoViewBundleAdjustment(Frame& cf, KeyFrame& kf, bool structureOnly) {
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

	ROS_DEBUG_STREAM_ONCE("dx of b2c: " << b2c.getOrigin().getX());

	double baseline_val = (cf.state.getr() - kf.frame->state.getr()).norm(); // this is the estimated distance between the two frames

	g2o::VertexSCam::setKcam(1.0, 1.0, 0.0, 0.0, baseline_val); // set up the camera parameters

	// setup vertex 1 aka keyFrame1
	const int KEYFRAME_VERTEX_ID = 0;

	Eigen::Isometry3d kf_pose;
	kf_pose = x_keyFrame.getQuaternion(); // set the transform's rotation
	kf_pose.translation() = x_keyFrame.getr(); // set the translation from the world to camera

	g2o::VertexSCam * kf_vertex = new g2o::VertexSCam(); // create a new vertex for this frame;

	kf_vertex->setId(KEYFRAME_VERTEX_ID); // the id of the first keyframe is 0

	kf_vertex->setEstimate(kf_pose); // set the estimate of this frame
	kf_vertex->setFixed(true); // the keyframe's position is fixed
	kf_vertex->setAll();

	optimizer.addVertex(kf_vertex); // add this vertex to the optimizer

	//setup vertex 2 aka current Frame
	const int CURRENTFRAME_VERTEX_ID = 1;

	Eigen::Isometry3d cf_pose;
	cf_pose = x_currentfFrame.getQuaternion();
	cf_pose.translation() = x_currentfFrame.getr();

	g2o::VertexSCam * cf_vertex = new g2o::VertexSCam(); // create a new vertex for this frame;

	cf_vertex->setId(CURRENTFRAME_VERTEX_ID); // the id of the first current frame is 1

	cf_vertex->setEstimate(cf_pose); // set the estimate of this frame
	// if this is structure only the vertex is fixed
	// otherwise it is not fixed and can be optimized
	cf_vertex->setFixed(structureOnly); // the keyframe's position is fixed
	cf_vertex->setAll();

	optimizer.addVertex(cf_vertex); // add the vertex to the problem

	// now the camera vertices are part of the problem
	const int POINTS_STARTING_ID = 2;

	int point_id = POINTS_STARTING_ID; // start the next vertex ids from 2
	int number_of_points = 0;

	// setup the rest of the graph optimization problem

	for(auto& kf_ft : kf.frame->features)
	{
		//check if the 3d point is still being tracked

		if(kf_ft.point->getStatus() == Point::TRACKING_GOOD && kf_ft.point != NULL)
		{
			g2o::VertexSBAPointXYZ * v_p = new g2o::VertexSBAPointXYZ(); // create a vertex for this 3d point

			v_p->setId(point_id); // set the vertex's id
			v_p->setMarginalized(true);
			v_p->setEstimate(kf_ft.point->getWorldCoordinate()); // set the initial estimate

			number_of_points++;

			optimizer.addVertex(v_p); // add this point to the problem

			// now we must set up the edges between these three vertices
			g2o::Edge_XYZ_VSC * e1 = new g2o::Edge_XYZ_VSC(); // here is the first edge

			e1->vertices()[0] = dynamic_cast<g2o::OptimizableGraph::Vertex*>(v_p); // set the 3d point
			e1->vertices()[1] = dynamic_cast<g2o::OptimizableGraph::Vertex*>(kf_vertex); // set the camera
			e1->setMeasurement(kf_ft.getDirectionVector()); //[u, v, 1]

			if(ROBUST_HUBER)
			{
				g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
				e1->setRobustKernel(rk);
			}

			optimizer.addEdge(e1); // finally add the edge

			g2o::Edge_XYZ_VSC * e2 = new g2o::Edge_XYZ_VSC(); // here is the first edge

			e2->vertices()[0] = dynamic_cast<g2o::OptimizableGraph::Vertex*>(v_p); // set the 3d point
			e2->vertices()[1] = dynamic_cast<g2o::OptimizableGraph::Vertex*>(cf_vertex); // set the camera
			// get the corresponding 2d feature linked to this 3d point in the current frame
			e2->setMeasurement(kf_ft.point->observations.front()->getDirectionVector()); //[u, v, 1]

			if(ROBUST_HUBER)
			{
				g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
				e2->setRobustKernel(rk);
			}

			optimizer.addEdge(e2); // finally add the edge

		}
		else
		{
			continue;
		}

		point_id++; // increment the point vertex id's
	}

	bool initStatus = optimizer.initializeOptimization(); // set up the problem for optimization
	ROS_WARN_STREAM_COND(!initStatus, "something went wrong when initializing the bundle adjustment problem");

	//optimizer.setVerbose(true);

	if(structureOnly)
	{
		ROS_INFO("Performing structure-only BA:");
		g2o::StructureOnlySolver<3> structure_only_ba;
		g2o::OptimizableGraph::VertexContainer points;
		for (g2o::OptimizableGraph::VertexIDMap::const_iterator it = optimizer.vertices().begin(); it != optimizer.vertices().end(); ++it) {
			g2o::OptimizableGraph::Vertex* v = static_cast<g2o::OptimizableGraph::Vertex*>(it->second);
			if (v->dimension() == 3)
				points.push_back(v);
		}

		structure_only_ba.calc(points, 10);
	}
}

