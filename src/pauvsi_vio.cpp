#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include "pauvsi_vio/vio.h"

VIO vio; // create an instance of the visual odometry algorithm

void cameraCallback(const sensor_msgs::ImageConstPtr& img, const sensor_msgs::CameraInfoConstPtr& cam)
{
	ros::Time start = ros::Time::now();
	cv::Mat temp = cv_bridge::toCvShare(img, "mono8")->image.clone();

	/* sets the current frame and its time created
	 * It also runs a series of functions which ultimately estimate
	 * the motion of the camera
	 */
	vio.setCurrentFrame(temp, cv_bridge::toCvCopy(img, "mono8")->header.stamp);

	vio.viewImage(vio.getCurrentFrame());

	ROS_DEBUG_STREAM_THROTTLE(2, "message #" << cv_bridge::toCvShare(img, "mono8")->header.seq << " finished in " << (ros::Time::now().toSec() - start.toSec()) * 1000 << " milliseconds");
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
