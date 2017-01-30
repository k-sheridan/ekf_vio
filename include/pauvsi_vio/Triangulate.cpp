/*
 * vioTriangulate.cpp
 *
 *  Created on: Jan 16, 2017
 *      Author: kevin
 */

#include "vio.h"


// FROM Theia by Chris Sweeney

// Given either a fundamental or essential matrix and two corresponding images
// points such that ematrix * point2 produces a line in the first image,
// this method finds corrected image points such that
// corrected_point1^t * ematrix * corrected_point2 = 0.
void VIO::FindOptimalImagePoints(const Eigen::Matrix3d& ematrix,
		const Eigen::Vector2d& point1, const Eigen::Vector2d& point2,
		Eigen::Vector2d* corrected_point1, Eigen::Vector2d* corrected_point2) {
	const Eigen::Vector3d point1_homog = point1.homogeneous();
	const Eigen::Vector3d point2_homog = point2.homogeneous();

	// A helper matrix to isolate certain coordinates.
	Eigen::Matrix<double, 2, 3> s_matrix;
	s_matrix << 1, 0, 0, 0, 1, 0;

	const Eigen::Matrix2d e_submatrix = ematrix.topLeftCorner<2, 2>();

	// The epipolar line from one image point in the other image.
	Eigen::Vector2d epipolar_line1 = s_matrix * ematrix * point2_homog;
	Eigen::Vector2d epipolar_line2 = s_matrix * ematrix.transpose()
							* point1_homog;

	const double a = epipolar_line1.transpose() * e_submatrix * epipolar_line2;
	const double b = (epipolar_line1.squaredNorm()
			+ epipolar_line2.squaredNorm()) / 2.0;
	const double c = point1_homog.transpose() * ematrix * point2_homog;

	const double d = sqrt(b * b - a * c);

	double lambda = c / (b + d);
	epipolar_line1 -= e_submatrix * lambda * epipolar_line1;
	epipolar_line2 -= e_submatrix.transpose() * lambda * epipolar_line2;

	lambda *= (2.0 * d)
							/ (epipolar_line1.squaredNorm() + epipolar_line2.squaredNorm());

	*corrected_point1 = (point1_homog
			- s_matrix.transpose() * lambda * epipolar_line1).hnormalized();
	*corrected_point2 = (point2_homog
			- s_matrix.transpose() * lambda * epipolar_line2).hnormalized();
}

// Triangulates 2 posed views
bool VIO::TriangulateDLT(const Matrix3x4d& pose1, const Matrix3x4d& pose2,
		const Eigen::Vector2d& point1, const Eigen::Vector2d& point2,
		Eigen::Vector4d* triangulated_point) {
	Eigen::Matrix4d design_matrix;
	design_matrix.row(0) = point1[0] * pose1.row(2) - pose1.row(0);
	design_matrix.row(1) = point1[1] * pose1.row(2) - pose1.row(1);
	design_matrix.row(2) = point2[0] * pose2.row(2) - pose2.row(0);
	design_matrix.row(3) = point2[1] * pose2.row(2) - pose2.row(1);

	// Extract nullspace.
	*triangulated_point =
			design_matrix.jacobiSvd(Eigen::ComputeFullV).matrixV().rightCols<1>();
	return true;
}


// Ported from Hartley and Zisserman:
// http://www.robots.ox.ac.uk/~vgg/hzbook/code/vgg_multiview/vgg_F_from_P.m
void VIO::FundamentalMatrixFromProjectionMatrices(const double pmatrix1[3 * 4],
		const double pmatrix2[3 * 4], double fmatrix[3 * 3]) {
	Eigen::Map<const Eigen::Matrix<double, 3, 4> > projection1(pmatrix1);
	Eigen::Map<const Eigen::Matrix<double, 3, 4> > projection2(pmatrix2);
	Eigen::Map < Eigen::Matrix3d > fundamental_matrix(fmatrix);

	const int index1[3] = { 1, 2, 0 };
	const int index2[3] = { 2, 0, 1 };
	Eigen::Matrix4d temp_mat;
	for (int r = 0; r < 3; r++) {
		temp_mat.row(2) = projection1.row(index1[r]);
		temp_mat.row(3) = projection1.row(index2[r]);
		for (int c = 0; c < 3; c++) {
			temp_mat.row(0) = projection2.row(index1[c]);
			temp_mat.row(1) = projection2.row(index2[c]);
			fundamental_matrix(r, c) = temp_mat.determinant();
		}
	}
}

// Triangulates 2 posed views
bool VIO::Triangulate(const Matrix3x4d& pose1, const Matrix3x4d& pose2,
		const Eigen::Vector2d& point1, const Eigen::Vector2d& point2,
		Eigen::Vector4d* triangulated_point, Eigen::Matrix3d fmatrix) {

	//FundamentalMatrixFromProjectionMatrices(pose1.data(), pose2.data(), fmatrix.data());

	Eigen::Vector2d corrected_point1, corrected_point2;
	FindOptimalImagePoints(fmatrix, point1, point2, &corrected_point1,
			&corrected_point2);

	//ROS_DEBUG_STREAM("correction delta: " << (point1 - corrected_point1).squaredNorm());

	// Now the two points are guaranteed to intersect. We can use the DLT method
	// since it is easy to construct.
	return TriangulateDLT(pose1, pose2, corrected_point1, corrected_point2,
			triangulated_point);
}


cv::Matx34d tfTransform2RtMatrix(tf::Transform& t)
{
	cv::Matx34d P(t.getBasis()[0][0], t.getBasis()[0][1], t.getBasis()[0][2], t.getOrigin().x(),
			t.getBasis()[1][0], t.getBasis()[1][1], t.getBasis()[1][2], t.getOrigin().y(),
			t.getBasis()[2][0], t.getBasis()[2][1], t.getBasis()[2][2], t.getOrigin().z());
	return P;
}

double VIO::ReprojectionError(const Matrix3x4d& pose, const Eigen::Vector4d& world_point, const Eigen::Vector2d& image_point) {
	const Eigen::Vector3d reprojected_point = pose * world_point;
	const double sq_reproj_error = (reprojected_point.hnormalized() - image_point).squaredNorm();
	return sq_reproj_error;
}

/*
 * cv::Matx61d b;

		cv::Point2f pt2=kf.matchedFeatures.at(i).getUndistorted(), pt1=cf.features.at(kf.currentFrameIndexes.at(i)).getUndistorted(); // get the two points

		b(0) = pt1.x;
		b(1) = pt1.y;
		b(2) = 1.0;
		b(3) = pt2.x;
		b(4) = pt2.y;
		b(5) = 1.0;

		cv::Matx41d X;

		cv::solve(A, b, X, cv::DECOMP_SVD);

		ROS_DEBUG_STREAM("X: " << X(2)/X(3));
 */

void VIO::decomposeEssentialMatrix(cv::Matx33f E, cv::Matx34d& Rt)
{
	cv::SVD svd(E);
	cv::Matx33d W(0,-1,0,   //HZ 9.13
			1,0,0,
			0,0,1);
	cv::Matx33d Winv(0,1,0,
			-1,0,0,
			0,0,1);
	cv::Mat_<double> R = svd.u * cv::Mat(W) * svd.vt; //HZ 9.19
	cv::Mat_<double> t = svd.u.col(2); //u3
	Rt = cv::Matx34d(R(0,0),    R(0,1), R(0,2), t(0),
			R(1,0),    R(1,1), R(1,2), t(1),
			R(2,0),    R(2,1), R(2,2), t(2));
}
