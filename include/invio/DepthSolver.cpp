/*
 * DepthSolver.cpp
 *
 *  Created on: Sep 17, 2017
 *      Author: kevin
 */

#include <DepthSolver.h>

DepthSolver::DepthSolver() {
	// TODO Auto-generated constructor stub

}

DepthSolver::~DepthSolver() {
	// TODO Auto-generated destructor stub
}


void DepthSolver::updatePointDepths(Frame& f)
{
	ros::Time start;
	if(ANALYZE_RUNTIME)
	{
		start = ros::Time::now();
	}

	std::vector<Feature*> feature_ptrs;

	feature_ptrs.reserve(f.features.size());

	for(auto& e : f.features)
	{
		if(e.obsolete) // skip if this point has been flagged for deletion
		{
			continue;
		}

		feature_ptrs.push_back(&e); // add the feature

		//increment the frame count for each point
		e.getPoint()->frames_since_depth_update++;
	}

	// sort in order of need for update descending
	this->sortFeaturesByNeedForUpdate(feature_ptrs);

	int updates = 0;

	for(auto& e : feature_ptrs)
	{
		if(updates >= MAX_DEPTH_UPDATES_PER_FRAME) // stop once enough points have been updated
		{
			break;
		}

		//compute the translation/depth ratio
		double t2d = (f.getPose().translation() - e->getPoint()->getInitialCameraPose().translation()).norm() / e->getPoint()->getDepth();

		if(t2d >= MIN_T2D) // if the camera has translated enough perform an update
		{
			ROS_DEBUG_STREAM("updating a feature with a translation ratio of: " << t2d);

			this->solveAndUpdatePointDepth(e->getPoint(), f.getPose_inv() * e->getPoint()->getInitialCameraPose(), e->getHomogenousCoord());

			updates++; // another update has been performed (even if it is bad or failed)

			e->getPoint()->frames_since_depth_update = 0; // reset the frame counter because this feature just had an update
		}
	}

	if(ANALYZE_RUNTIME)
	{
		ROS_INFO_STREAM("depth update runtime: " << (ros::Time::now() - start).toSec() * 1000 << " ms");
	}
}

/*
 * i could completely remove this if I move updated elements to the back of the vector once everything is done
 */
void DepthSolver::sortFeaturesByNeedForUpdate(std::vector<Feature*>& feature_ptrs)
{
	// this sorts from high to low
	struct SortComparison {
	  bool operator() (Feature* i,Feature* j) { return (i->getPoint()->frames_since_depth_update > j->getPoint()->frames_since_depth_update);}
	} sort_key;

	std::sort(feature_ptrs.begin(), feature_ptrs.end(), sort_key);
}

bool DepthSolver::solveAndUpdatePointDepth(Point* pt, Sophus::SE3d cf_2_rf, Eigen::Vector3d curr_ft)
{
	// first orthogonally project the current feature onto the epiline
	Eigen::Vector3d epiline = cf_2_rf.rotationMatrix() * pt->getInitialHomogenousCoordinate();
	//Eigen::Vector3d measured_vector = curr_ft - cf_2_rf.translation();

	//Eigen::Vector3d projected3 = (measured_vector.dot(epiline) / epiline.dot(epiline)) * epiline + cf_2_rf.translation();
	// this is the final projected point
	Eigen::Vector3d projected_ft; 
	
	//projected_ft << (projected3(0) / projected3(2)), (projected3(1) / projected3(2)), 1.0;

	projected_ft = curr_ft;

	//ROS_INFO_STREAM("original: " << curr_ft << " projected: " << projected_ft);

	//solve for the depth
	Eigen::Matrix<double,3,2> A; A << epiline, projected_ft;

	const Eigen::Matrix2d AtA = A.transpose()*A;

	if(AtA.determinant() < MINIMUM_DEPTH_DETERMINANT)
	{
		ROS_DEBUG("determinant too low");
	    return false;
	}

	const Eigen::Vector2d depth2 = - AtA.inverse()*A.transpose()*cf_2_rf.translation();

	double depth = fabs(depth2[0]);

	if(depth < MIN_POINT_Z || depth > MAX_POINT_Z)
		return false;

	//evaluate the reprojection error
	//Eigen::Vector3d projected_ref_ft = (cf_2_rf * (depth * pt->getInitialHomogenousCoordinate()));

	//double chi2 = pow(projected_ref_ft(0)/projected_ref_ft(2) - curr_ft(0), 2) + pow(projected_ref_ft(1)/projected_ref_ft(2) - curr_ft(1), 2);

	Eigen::Vector3d t = cf_2_rf.inverse().translation();
	Eigen::Vector3d d = depth*pt->getInitialHomogenousCoordinate();
	Eigen::Vector3d t2d = d - t;

	double d_norm = d.norm();
	double t2d_norm = t2d.norm();
	//double t_norm = t.norm();

	double sine_theta_d_t = d.cross(t2d).norm() / (d_norm*t2d_norm);

	double variance = 1 / pow(sine_theta_d_t + DBL_MIN, 2);

	ROS_DEBUG_STREAM("updating point with depth: " << depth << " and variance: " << variance << "where the sine is: " << sine_theta_d_t);

	pt->updateDepth(depth, variance);
	
	//check if the feature has converged enough to become a candidate
	if(pt->getVariance() <= MOBA_CANDIDATE_VARIANCE)
	{
		pt->moba_candidate = true; // flag for final step before moba integration or deletion
	}

	return true;

}

