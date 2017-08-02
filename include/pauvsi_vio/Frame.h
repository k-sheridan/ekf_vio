/*
 * Frame.h
 *
 *  Created on: Aug 1, 2017
 *      Author: kevin
 */

#ifndef PAUVSI_VIO_INCLUDE_PAUVSI_VIO_FRAME_H_
#define PAUVSI_VIO_INCLUDE_PAUVSI_VIO_FRAME_H_

#include "sophus/se3.hpp"

#include <Feature.h>
#include <vioParams.h>

class Frame {
private:

	bool keyframe;

	double avgFeatureDepth;
	bool avgFeatureDepthSet;

	Sophus::SE3 poseEstimate; // this is a world to camera pose estimate
	Sophus::SE3 poseEstimate_inv;

public:

	std::vector<Feature> features;

	cv::Mat_<float> K;
	cv::Mat img;
	ros::Time t;

	Frame();
	Frame(cv::Mat _img, cv::Mat_<float> _k, ros::Time _t);

	virtual ~Frame();

	bool isKeyframe(){return this->isKeyframe;}

	Sophus::SE3 getPose(){return poseEstimate;}
	Sophus::SE3 getPose_inv(){return poseEstimate_inv;}

	void setPose(Sophus::SE3 tf);
	void setPose_inv(Sophus::SE3 tf);

	double getAverageFeatureDepth();

	/// Frame jacobian for projection of 3D point in (f)rame coordinate to
	/// unit plane coordinates uv (focal length = 1).
	// meant for pose optimization
	inline static void jacobian_xyz2uv(
			const Eigen::Vector3d& xyz_in_f,
			Eigen::Matrix<double,2,6>& J)
	{
		const double x = xyz_in_f[0];
		const double y = xyz_in_f[1];
		const double z_inv = 1./xyz_in_f[2];
		const double z_inv_2 = z_inv*z_inv;

		J(0,0) = -z_inv;              // -1/z
		J(0,1) = 0.0;                 // 0
		J(0,2) = x*z_inv_2;           // x/z^2
		J(0,3) = y*J(0,2);            // x*y/z^2
		J(0,4) = -(1.0 + x*J(0,2));   // -(1.0 + x^2/z^2)
		J(0,5) = y*z_inv;             // y/z

		J(1,0) = 0.0;                 // 0
		J(1,1) = -z_inv;              // -1/z
		J(1,2) = y*z_inv_2;           // y/z^2
		J(1,3) = 1.0 + y*J(1,2);      // 1.0 + y^2/z^2
		J(1,4) = -J(0,3);             // -x*y/z^2
		J(1,5) = -x*z_inv;            // x/z
	}

	Sophus::SE3 tf2sophus(tf::Transform tf);
	tf::Transform sophus2tf(Sophus::SE3 se3);
};

#endif /* PAUVSI_VIO_INCLUDE_PAUVSI_VIO_FRAME_H_ */
