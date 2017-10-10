/*
 * Frame.cpp
 *
 *  Created on: Aug 1, 2017
 *      Author: kevin
 */

#include "../invio/Frame.h"

Frame::Frame() {


}

Frame::Frame(cv::Mat _img, cv::Mat_<float> _k, ros::Time _t)
{

	this->img = _img;
	this->K = _k;
	this->t = _t;


}

Frame::~Frame() {
	// TODO Auto-generated destructor stub
}

void Frame::setAllPointsMature(){
	for(auto& e : this->features)
	{
		e.getPoint()->setImmature(false);
		e.getPoint()->guessed = true; // set to guessed
	}
}

double Frame::getAverageFeatureDepth()
{

	Eigen::Matrix3d mat3 = this->getPose_inv().rotationMatrix();
	double a = mat3(2, 0);
	double b = mat3(2, 1);
	double c = mat3(2, 2);
	double d = this->getPose_inv().translation().z();

	// compute the average feature depth of this frame
	int maturePointCount = 0;
	double depth = 0;
	for(auto e : this->features)
	{
		if(!e.obsolete){
			if(!e.getPoint()->isImmature())
			{
				maturePointCount++;

				//make more efficient
				//tf::Vector3 pointInCameraFrame = this->getPose_inv() * e.obj;

				double z_depth = a * e.getPoint()->getWorldCoordinate().x() + b * e.getPoint()->getWorldCoordinate().y() + c * e.getPoint()->getWorldCoordinate().z() + d; // add the z parts together

				e.getPoint()->temp_depth = z_depth;

				if(z_depth > 0)
				{
					depth += z_depth;
				}
				else
				{
					maturePointCount--;
					ROS_WARN("point is behind camera");
				}
			}
		}
	}

	if(maturePointCount > 0)
	{
		ROS_DEBUG_STREAM("average scene depth computed: " << depth / (double)maturePointCount);
		return depth / (double)maturePointCount;
	}
	else
	{
		ROS_WARN("no valid mature points for average feature depth computation");

		return DEFAULT_POINT_DEPTH;
	}
}


void Frame::setPose(Sophus::SE3d tf){
	this->poseEstimate = tf;
	this->poseEstimate_inv = this->poseEstimate.inverse();
}

void Frame::setPose_inv(Sophus::SE3d tf){
	this->poseEstimate_inv = tf;
	this->poseEstimate = this->poseEstimate_inv.inverse();
}


bool Frame::isPixelInBox(cv::Point2f px)
{
	int centerX = this->img.cols / 2.0;
	int centerY = this->img.rows / 2.0;
	if(abs((int)px.x - centerX) > KILL_BOX_WIDTH || abs((int)px.y - centerY) > KILL_BOX_HEIGHT)
	{
		ROS_DEBUG_STREAM("pixel is outside of kill box");
		return false;
	}
	else
	{
		return true;
	}
}



/*
 * uses the current pose and current pixels to determine the 3d position of the objects
 * assumes that all pixels lie on a plane
 */

/*void updateObjectPositions(cv::Mat_<float> K)
{
	for(int i = 0; i < features.size(); i++)
	{
		tf::Vector3 pixel = tf::Vector3((features.at(i).px.x - K(2)) / K(0), (features.at(i).px.y - K(5)) / K(4), 1.0);

		tf::Vector3 dir = currentPose * pixel - currentPose.getOrigin();

		double dt = (-currentPose.getOrigin().z() / dir.z());

		if(dt <= 0)
		{
			ROS_DEBUG("removing feature from planar odom because it is not on the xy plane");
			//remove this feature it is not on the plane
			features.erase(features.begin() + i);
			i--;
		}
		else
		{
			features.at(i).obj = currentPose.getOrigin() + dir * dt;
		}
	}

	if(features.size() <= 4)
	{
		ROS_WARN("planar odometry has too few features!");
	}
}*/
