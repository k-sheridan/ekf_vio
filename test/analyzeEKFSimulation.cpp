#include <ros/ros.h>

#include "Params.h"
#include "../include/invio/VIO.h"

void simulateAndVisualizeEKF(int feature_count, float depth_sigma, float depth_mu, Eigen::Vector3f b_vel, Eigen::Vector3f b_accel, Eigen::Vector3f omega, float dt, float tf){
	cv::RNG rng(0); // create a rendom number generator with seed zero for repeatability

	//generate features
	std::vector<Eigen::Vector3f> gt_pos_vec;
	for(int i = 0; i < feature_count; i++){
		Eigen::Vector3f pos;

		pos(2) = depth_mu + rng.gaussian(depth_sigma);
		pos(0) = rng.uniform(-1.5, 1.5) * pos(2);
		pos(1) = rng.uniform(-1.5, 1.5) * pos(2);

		gt_pos_vec.push_back(pos);
	}

	Eigen::Vector3f pos = Eigen::Vector3f(0, 0, 0);
	Eigen::Quaternionf quat = Eigen::Quaternionf(1, 0, 0, 0);

	//simulate and visualize the ekf
	for(float t = 0; t <= tf; t += dt){

	}

}

void generateFakeMeasurementsAndUpdateEKF(TightlyCoupledEKF& tc_ekf, std::vector<Eigen::Vector3f> point_pos, float dt, Eigen::Vector3f pos, Eigen::Quaternionf quat){
	
	std::vector<Eigen::Vector2f> feature_pos_vec;
	std::vector<Eigen::Matrix2f> covs;
	std::vector<bool> measured;

	Eigen::Matrix2f cov;
	cov << 0.00001, 0, 0, 0.00001;

	for(auto e : point_pos){
		Eigen::Vector3f fp;

		fp = quat.inverse()*e - quat.inverse()*pos;

		// add a 2d observation
		feature_pos_vec.push_back(Eigen::Vector2f(fp.x() / fp.z(), fp.y() / fp.z());
		covs.push_back(cov);
		measured.push_back(true);
	}

	tc_ekf.updateWithFeaturePositions(feature_pos_vec, covs, measured); // perform update 

}

void visualizeEKF(TightlyCoupledEKF tc_ekf){
	cv::Mat black = cv::Mat(600, 600, CV_8U);

	int i = 0;
	for(auto& e : tc_ekf.features)
	{
		
		//ROS_DEBUG_STREAM(e.getPixel(f));
		cv::drawMarker(black, e.getPixel(f), cv::Scalar(0, 255, 0), cv::MARKER_SQUARE, 22, 1);

		//ROS_DEBUG_STREAM("plotting covariance in pixels: " << this->tc_ekf.getMetric2PixelMap(f.K)*this->tc_ekf.getFeatureHomogenousCovariance(i)*this->tc_ekf.getMetric2PixelMap(f.K).transpose());
			

		cv::RotatedRect rr = getErrorEllipse(0.99, e.getPixel(f), 300*300*tc_ekf.getFeatureHomogenousCovariance(i));
		//ROS_DEBUG_STREAM(rr.size);
		cv::ellipse(black, rr, cv::Scalar(255, 255, 0), 1);
		
		// next feature
		i++;
	}
}

/*
 * covariance and mean must be in pixels
 */
cv::RotatedRect getErrorEllipse(double chisquare_val, cv::Point2f mean, Eigen::Matrix2f eig_covmat){

	//Get the eigenvalues and eigenvectors
	Eigen::EigenSolver<Eigen::Matrix2f> es;
	es.compute(eig_covmat, true);

	Eigen::EigenSolver<Eigen::Matrix2f>::EigenvalueType eig_vals = es.eigenvalues();

	if(es.info() != Eigen::ComputationInfo::Success)
	{
		ROS_DEBUG_STREAM("eigen vals and or vecs not computed: " << eig_covmat);
		return cv::RotatedRect(mean, cv::Size2f(10, 10), 0);
	}

	Eigen::EigenSolver<Eigen::Matrix2f>::EigenvectorsType eig_vecs = es.eigenvectors();

	double angle;
	double halfmajoraxissize;
	double halfminoraxissize;

	if(eig_vals(0).real() > eig_vals(1).real())
	{
		//Calculate the angle between the largest eigenvector and the x-axis
		angle = atan2(eig_vecs(1,0).real(), eig_vecs(0,0).real());

		//Shift the angle to the [0, 2pi] interval instead of [-pi, pi]
		if(angle < 0)
			angle += 6.28318530718;

		//Conver to degrees instead of radians
		angle = 180*angle/3.14159265359;

		//Calculate the size of the minor and major axes
		halfmajoraxissize=chisquare_val*sqrt(eig_vals(0).real());
		halfminoraxissize=chisquare_val*sqrt(eig_vals(1).real());
	}
	else
	{
		//Calculate the angle between the largest eigenvector and the x-axis
		angle = atan2(eig_vecs(1,1).real(), eig_vecs(0,1).real());

		//Shift the angle to the [0, 2pi] interval instead of [-pi, pi]
		if(angle < 0)
			angle += 6.28318530718;

		//Conver to degrees instead of radians
		angle = 180*angle/3.14159265359;

		//Calculate the size of the minor and major axes
		halfmajoraxissize=chisquare_val*sqrt(eig_vals(1).real());
		halfminoraxissize=chisquare_val*sqrt(eig_vals(0).real());
	}


	halfmajoraxissize = std::max(halfmajoraxissize, 0.1);
	halfminoraxissize = std::max(halfminoraxissize, 0.1);

	//Return the oriented ellipse
	//The -angle is used because OpenCV defines the angle clockwise instead of anti-clockwise
	return cv::RotatedRect(mean, cv::Size2f(halfmajoraxissize, halfminoraxissize), -angle);

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "general_test"); // initializes ros

	ros::NodeHandle nh;


	ros::param::param<double>("~default_point_depth", DEFAULT_POINT_DEPTH, D_DEFAULT_POINT_DEPTH);
	//parseROSParams();


	//simulate a moving camera and make the points converge (ideal scenario)
	simulateAndVisualizeEKF(30, 0.000001, 0.5, Eigen::Vector3f(0.5, 0, 0), Eigen::Vector3f(0, 0, 0), Eigen::Vector3f(0, 0, 0));

	return 0;
}