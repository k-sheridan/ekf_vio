/*
 * VIO.cpp
 *
 *  Created on: Jul 8, 2017
 *      Author: kevin
 */

#include "VIO.h"

VIO::VIO() {

	ros::NodeHandle nh; // we all know what this is

}

VIO::~VIO() {
	// TODO Auto-generated destructor stub
}

void VIO::addFrame(cv::Mat img, cv::Mat_<float> k, ros::Time t)
{
	Frame f = Frame(img, k, t);

	if(this->frame_buffer.size() == 0) // if this is the first frame that we are receiving
	{
		ROS_DEBUG("adding the first frame");
		f.setPose(Sophus::SE3d(Eigen::Quaterniond(1.0, 0, 0, 0), Eigen::Vector3d(0.0, 0.0, 0.0))); // set the initial position to 0 (this is world to camera)

		this->frame_buffer.push_front(f); // add the frame to the front of the buffer

		this->replenishFeatures((this->frame_buffer.front()));
	}
	else // we have atleast 1 frame in the buffer
	{

	}
}

void VIO::updateFeatures(Frame& last_f, Frame& new_f) {

	std::vector<cv::Point2f> oldPoints = this->getPixels2fInOrder(last_f);

	ROS_ASSERT(oldPoints.size() > 0);

	//ROS_DEBUG_STREAM_ONCE("got " << oldPoints.size() << " old point2fs from the oldframe which has " << oldFrame.features.size() << " features");
	std::vector<cv::Point2f> newPoints;

	std::vector<uchar> status; // status vector for each point
	cv::Mat error; // error vector for each point

	ROS_DEBUG("before klt");

	cv::calcOpticalFlowPyrLK(last_f.img, new_f.img, oldPoints, newPoints,
			status, error, cv::Size(21, 21), 3,
			cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS,
					30, 0.01), 0, KLT_MIN_EIGEN);

	ROS_DEBUG("after klt");

	std::vector<Feature> flowedFeatures;

	int lostFeatures = 0;

	for (int i = 0; i < status.size(); i++) {
		if (status.at(i) == 1) {
			Feature updated_feature = last_f.features.at(i);

			updated_feature.px = newPoints.at(i); // set feature's new pixel location

			updated_feature.setParentFrame(&new_f); // set this features parent frame

			new_f.features.push_back(updated_feature); // add the new feature

			//add this feature to the point's observation buffer with the frame's feature buffer memory location
			new_f.features.back().observations.push_front(&(new_f.features.back()));

		} else {
			lostFeatures++;
			// this feature is lost so we should/must safely null all references to it and remove it from the map
			last_f.features.at(i).getPoint()->safelyDeletePoint();
		}
	}

	ROS_DEBUG_STREAM("VO LOST " << lostFeatures << "FEATURES");

}

bool VIO::computePose(double& perPixelError) {

	ROS_DEBUG("computing motion");

	ROS_DEBUG("done computing motion");

	return true;
}

/*
 * get more features after updating the pose
 */
void VIO::replenishFeatures(Frame& f) {
	//add more features if needed
	cv::Mat img;
	if (FAST_BLUR_SIGMA != 0.0){
		cv::GaussianBlur(f.img, img, cv::Size(5, 5), FAST_BLUR_SIGMA);
	}
	else{
		img = f.img;
	}

	if (f.features.size() < NUM_FEATURES) {
		std::vector<cv::KeyPoint> fast_kp;

		cv::FAST(img, fast_kp, FAST_THRESHOLD, true);

		int needed = NUM_FEATURES - f.features.size();

		ROS_DEBUG_STREAM("need " << needed << "more features");

		/*cv::flann::Index tree;

		if (this->state.features.size() > 0) {
			std::vector<cv::Point2f> prev = this->state.getPixels2fInOrder();
			tree = cv::flann::Index(cv::Mat(prev).reshape(1),
					cv::flann::KDTreeIndexParams());
		}*/

		//image which is used to check if a close feature already exists
		cv::Mat checkImg = cv::Mat::zeros(img.size(), CV_8U);
		for(auto e : f.features)
		{
			cv::drawMarker(checkImg, e.px, cv::Scalar(255), cv::MARKER_SQUARE, MIN_NEW_FEATURE_DIST * 2, MIN_NEW_FEATURE_DIST);
		}



		for (int i = 0; i < needed && i < fast_kp.size(); i++) {
			/*if (this->state.features.size() > 0) {
				//make sure that this corner is not too close to any old corners
				std::vector<float> query;
				query.push_back(fast_kp.at(i).pt.x);
				query.push_back(fast_kp.at(i).pt.y);

				std::vector<int> indexes;
				std::vector<float> dists;

				tree.knnSearch(query, indexes, dists, 1);

				if (dists.front() < MIN_NEW_FEATURE_DIST) // if this featrue is too close to a already tracked feature skip it
				{
					continue;
				}
			}*/

			//check if there is already a close feature
			if(checkImg.at<unsigned char>(fast_kp.at(i).pt))
			{
				ROS_DEBUG("feature too close to previous feature, not adding");
				needed++; // we need one more now
				continue;
			}

			Feature new_ft;

			new_ft.px = fast_kp.at(i).pt;
			new_ft.setParentFrame(&f);

			f.features.push_back(new_ft);

			// important - we need to push a point onto the map
			// pushes this feature onto the observation buffer's front
			this->map.push_back(Point(&(f.features.back()))); // the point needs to be linked to the feature's memory location in the frame's feature buffer

			f.features.back().setPoint(&(this->map.back())); // set the features point to the points pos in the map -- they are now linked with their pointers

			f.features.back().getPoint()->setupMapAndPointLocation((--this->map.end()), &(this->map)); // tell the point where it is in memory via an iterator and where the map is so it can delet itself later




#if USE_POINT_CLOUD
			//todo try to initialize using pointcloud projection
#else
			new_ft.computeObjectPositionWithAverageSceneDepth();
#endif

		}
	}
}


void VIO::tf2rvecAndtvec(tf::Transform tf, cv::Mat& tvec, cv::Mat& rvec){
	cv::Mat_<double> R = (cv::Mat_<double>(3, 3) << tf.getBasis().getRow(0).x(), tf.getBasis().getRow(0).y(), tf.getBasis().getRow(0).z(),
			tf.getBasis().getRow(1).x(), tf.getBasis().getRow(1).y(), tf.getBasis().getRow(1).z(),
			tf.getBasis().getRow(2).x(), tf.getBasis().getRow(2).y(), tf.getBasis().getRow(2).z());

	//ROS_DEBUG("setting up tvec and rvec");

	cv::Rodrigues(R, rvec);

	tvec = (cv::Mat_<double>(3, 1) << tf.getOrigin().x(), tf.getOrigin().y(), tf.getOrigin().z());

	//ROS_DEBUG_STREAM("tvec: " << tvec << "\nrvec: " << rvec);
}

tf::Transform VIO::rvecAndtvec2tf(cv::Mat tvec, cv::Mat rvec){
	//ROS_DEBUG("rvectvec to tf");
	cv::Mat_<double> rot;
	cv::Rodrigues(rvec, rot);
	/*ROS_DEBUG_STREAM("rot: " << rot);
		ROS_DEBUG_STREAM("rvec: " << rvec);*/
	//ROS_DEBUG_STREAM("tvec " << tvec);

	tf::Transform trans;

	trans.getBasis().setValue(rot(0), rot(1), rot(2), rot(3), rot(4), rot(5), rot(6), rot(7), rot(8));
	trans.setOrigin(tf::Vector3(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2)));

	//ROS_DEBUG_STREAM("rot: " << trans.getRotation().w() << ", " << trans.getRotation().x() << ", " << trans.getRotation().y() << ", " << trans.getRotation().z());
	/*double x, y, z;
		trans.getBasis().getRPY(x, y, z);
		ROS_DEBUG_STREAM("tf rvec " << x <<", "<<y<<", "<<z);*/
	//ROS_DEBUG_STREAM(trans.getOrigin().x() << ", " << trans.getOrigin().y() << ", " << trans.getOrigin().z());

	//ROS_DEBUG("finished");

	return trans;
}

/*std::vector<cv::Point2d> VIO::getPixelsInOrder(Frame& f){
	std::vector<cv::Point2d> pixels;

	for(auto e : f.features)
	{
		pixels.push_back(cv::Point2d(e.px.x, e.px.y));
	}

	return pixels;
}*/

std::vector<cv::Point2f> VIO::getPixels2fInOrder(Frame& f){
	std::vector<cv::Point2f> pixels;

	for(auto e : f.features)
	{
		pixels.push_back(e.px);
	}

	return pixels;
}

/*std::vector<cv::Point3d> VIO::getObjectsInOrder(Frame& f){
	std::vector<cv::Point3d> obj;

	for(auto e : f.features)
	{
		obj.push_back(cv::Point3d(e.obj.x(), e.obj.y(), e.obj.z()));
	}

	return obj;
}*/
