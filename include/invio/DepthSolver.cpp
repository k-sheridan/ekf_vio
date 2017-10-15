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

		//compute the change in translation/depth ratio
		double t2d = ((f.getPose().translation() - e->getPoint()->last_update_pose.translation()).norm() / e->getPoint()->last_update_pose_depth);

		double depth_in_current_frame;

		Sophus::SE3d cf_2_rf;

		if(t2d >= MIN_T2D) // if the camera has translated enough perform an update
		{
			ROS_DEBUG_STREAM("updating a feature with a translation ratio of: " << t2d);

			Sophus::SE3d rf_2_cf = e->getPoint()->getInitialCameraPose_inv() * f.getPose();

			cf_2_rf = rf_2_cf.inverse();

			this->solveAndUpdatePointDepth(e->getPoint(), rf_2_cf, cf_2_rf, e->getHomogenousCoord(), depth_in_current_frame);

			updates++; // another update has been performed (even if it is bad or failed)

			e->getPoint()->frames_since_depth_update = 0; // reset the frame counter because this feature just had an update

			//update the last t2d
			e->getPoint()->last_update_pose = f.getPose();
			e->getPoint()->last_update_pose_depth = (cf_2_rf * (e->getPoint()->getDepth() * e->getPoint()->getInitialHomogenousCoordinate())).z(); // compute the depth in the update pose frame
		}
	}

	if(ANALYZE_RUNTIME)
	{
		ROS_INFO_STREAM("depth update runtime: " << (ros::Time::now() - start).toSec() * 1000 << " ms");
	}
}

/*
 * i could completely remove this if I move updated elements to the back of the vector once everything is done
 *
 * HOWEVER, this runs in about 0.02 ms so I think its good enough
 */
void DepthSolver::sortFeaturesByNeedForUpdate(std::vector<Feature*>& feature_ptrs)
{
	// this sorts from high to low
	struct SortComparison {
	  bool operator() (Feature* i,Feature* j) { return (i->getPoint()->frames_since_depth_update > j->getPoint()->frames_since_depth_update);}
	} sort_key;

	std::sort(feature_ptrs.begin(), feature_ptrs.end(), sort_key);
}

bool DepthSolver::solveAndUpdatePointDepth(Point* pt, Sophus::SE3d rf_2_cf, Sophus::SE3d cf_2_rf, Eigen::Vector3d curr_ft, double& depth_in_current_frame)
{

	// stereo depth estimate of feature in current frame

	Eigen::Vector3d epiline = rf_2_cf.rotationMatrix() * curr_ft;

	//solve for the depth
	Eigen::Matrix<double,3,2> A; A << epiline, pt->getInitialHomogenousCoordinate();

	const Eigen::Matrix2d AtA = A.transpose()*A;

	//double AtA_det = AtA.determinant();

	if(AtA.determinant() < MINIMUM_DEPTH_DETERMINANT)
	{
		ROS_DEBUG("determinant too low");
	    return false;
	}

	const Eigen::Vector2d depth2 = - AtA.inverse()*A.transpose()*rf_2_cf.translation();

	double depth = fabs(depth2[0]);

	if(depth < MIN_POINT_Z || depth > MAX_POINT_Z)
	{
		ROS_DEBUG_STREAM("point at extreme depth");
		return false;
	}

	//evaluate the reprojection error
	//Eigen::Vector3d projected_ref_ft = (cf_2_rf * (depth * pt->getInitialHomogenousCoordinate()));

	//double chi2 = pow(projected_ref_ft(0)/projected_ref_ft(2) - curr_ft(0), 2) + pow(projected_ref_ft(1)/projected_ref_ft(2) - curr_ft(1), 2);

	//Angle variance
	Eigen::Vector3d t = cf_2_rf.translation();
	Eigen::Vector3d d = depth*curr_ft;
	Eigen::Vector3d t2d = d - t;

	double d_norm = d.norm();
	double t2d_norm = t2d.norm();
	//double t_norm = t.norm();

	double sine_theta_d_t = d.cross(t2d).norm() / (d_norm*t2d_norm);

	double variance = 1 / pow(sine_theta_d_t + DBL_MIN, 2);

	ROS_DEBUG_STREAM("updating point with depth: " << depth << " and depth variance: " << variance << "where the sine is: " << sine_theta_d_t);


	//TODO find good homogeneous variance
	Eigen::Vector3d point_in_rf = rf_2_cf * (depth * curr_ft); // transform the measured point into the reference frame

	Eigen::Vector3d sigma = Eigen::Vector3d(2 * DEFAULT_POINT_HOMOGENOUS_VARIANCE, 2 * DEFAULT_POINT_HOMOGENOUS_VARIANCE, variance);

	Eigen::Vector3d z = Eigen::Vector3d(point_in_rf(0) / point_in_rf(2), point_in_rf(1) / point_in_rf(2), point_in_rf(2));

	pt->update(z, sigma);
	
	//check if the feature has converged enough to become a candidate
	if(pt->getDepthVariance() <= MOBA_CANDIDATE_VARIANCE)
	{
		pt->moba_candidate = true; // flag for final step before moba integration or deletion
	}

	return true;

}

