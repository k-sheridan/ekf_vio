#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include "pauvsi_vio/vio.h"

VIO vio; // create an instance of the visual odometry algorithm

cv::Mat get3x3FromVector(boost::array<double, 9> vec)
{
	cv::Mat mat = cv::Mat(3, 3, CV_32F);
	for(int i = 0; i < 3; i++)
	{
		mat.at<float>(i, 0) = vec.at(3 * i + 0);
		mat.at<float>(i, 1) = vec.at(3 * i + 1);
		mat.at<float>(i, 2) = vec.at(3 * i + 2);
	}

	ROS_DEBUG_STREAM_ONCE("K = " << mat);
	return mat;
}



void cameraCallback(const sensor_msgs::ImageConstPtr& img, const sensor_msgs::CameraInfoConstPtr& cam)
{
	ros::Time start = ros::Time::now();
	cv::Mat temp = cv_bridge::toCvShare(img, "mono8")->image.clone();

	//set the K and D matrices
	vio.setK(get3x3FromVector(cam->K));
	vio.setD(cv::Mat(cam->D, false));

	/* sets the current frame and its time created
	 * It also runs a series of functions which ultimately estimate
	 * the motion of the camera
	 */
	vio.setCurrentFrame(temp, cv_bridge::toCvCopy(img, "mono8")->header.stamp);

	ROS_DEBUG_STREAM_THROTTLE(0.5, "message #" << cv_bridge::toCvShare(img, "mono8")->header.seq << " finished in " << (ros::Time::now().toSec() - start.toSec()) * 1000 << " milliseconds");

	//vio.viewImage(vio.getCurrentFrame());
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pauvsi_vio", ros::init_options::AnonymousName); // initializes with a randomish name
	ros::NodeHandle nh;

	vio.readROSParameters(); // read parameters into vio class from parameter server

	//set up image transport
	image_transport::ImageTransport it(nh);
	image_transport::CameraSubscriber cameraSub;
	cameraSub = it.subscribeCamera(vio.getCameraTopic(), 1, cameraCallback);

	ros::spin();

	return 0;
}
