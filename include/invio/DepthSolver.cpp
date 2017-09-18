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
	for(auto e : f.features)
	{
		this->solveAndUpdatePointDepth(e.getPoint(), f.getPose_inv() * e.getPoint()->getInitialCameraPose(), e.getHomogenousCoord());
	}
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
	    return false;

	const Eigen::Vector2d depth2 = - AtA.inverse()*A.transpose()*cf_2_rf.translation();

	double depth = fabs(depth2[0]);

	if(depth < MIN_POINT_Z || depth > MAX_POINT_Z)
		return false;

	//evaluate the reprojection error
	//Eigen::Vector3d projected_ref_ft = (cf_2_rf * (depth * pt->getInitialHomogenousCoordinate()));

	//double chi2 = pow(projected_ref_ft(0)/projected_ref_ft(2) - curr_ft(0), 2) + pow(projected_ref_ft(1)/projected_ref_ft(2) - curr_ft(1), 2);


	double variance = (depth*pt->getInitialHomogenousCoordinate()).squaredNorm() / std::max(cf_2_rf.inverse().translation().cross(depth*pt->getInitialHomogenousCoordinate()).squaredNorm(), 0.00000000001);

	ROS_INFO_STREAM("updating point with depth: " << depth << " and variance: " << variance);

	pt->updateDepth(depth, variance);
	
	pt->setImmature(false);

	return true;

}

