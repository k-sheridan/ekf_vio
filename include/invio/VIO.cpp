/*
 * VIO.cpp
 *
 *  Created on: Jul 8, 2017
 *      Author: kevin
 */

#include "../invio/VIO.h"

VIO::VIO() {

	//set uninitialized
	this->initialized = false;
	//set tracking lost to false initialially
	this->tracking_lost = false;

	ros::NodeHandle nh; // we all know what this is

	image_transport::ImageTransport it(nh);
	image_transport::CameraSubscriber bottom_cam_sub = it.subscribeCamera(
			CAMERA_TOPIC, 2, &VIO::camera_callback, this);

#if PUBLISH_INSIGHT
	this->insight_pub = nh.advertise<sensor_msgs::Image>(INSIGHT_TOPIC, 1);
#endif

	this->odom_pub = nh.advertise<nav_msgs::Odometry>(ODOM_TOPIC, 1);

	//get relevant transforms

	tf::StampedTransform b2c_st;
	ROS_INFO_STREAM("WAITING FOR TANSFORM FROM " << BASE_FRAME << " TO " << CAMERA_FRAME);
	if(this->tf_listener.waitForTransform(BASE_FRAME, CAMERA_FRAME, ros::Time(0), ros::Duration(10))){
		try {
			this->tf_listener.lookupTransform(BASE_FRAME, CAMERA_FRAME,
					ros::Time(0), b2c_st);
		} catch (tf::TransformException& e) {
			ROS_WARN_STREAM(e.what());
		}

		this->b2c = tf::Transform(b2c_st);
		this->c2b = this->b2c.inverse();
	}
	else
	{
		ROS_FATAL("COULD NOT GET TRANSFORM");
		ros::shutdown();
		return;
	}

	ros::spin();
}

VIO::~VIO() {
	// TODO Auto-generated destructor stub
}

void VIO::camera_callback(const sensor_msgs::ImageConstPtr& img,
		const sensor_msgs::CameraInfoConstPtr& cam) {
	cv::Mat temp = cv_bridge::toCvShare(img, img->encoding)->image.clone();

	this->addFrame(temp.clone(),
			(cv::Mat_<float>(3, 3) << cam->K.at(0), cam->K.at(1), cam->K.at(2), cam->K.at(3), cam->K.at(4), cam->K.at(5), cam->K.at(6), cam->K.at(7), cam->K.at(8)),
			img->header.stamp);
}

void VIO::addFrame(cv::Mat img, cv::Mat_<float> k, ros::Time t) {
	Frame f = Frame(img, k, t);

	if (this->frame_buffer.size() == 0) // if this is the first frame that we are receiving
	{
		ROS_DEBUG("adding the first frame");
		f.setPose(
				Sophus::SE3d(Eigen::Quaterniond(1.0, 0, 0, 0),
						Eigen::Vector3d(0.0, 0.0, 0.0))); // set the initial position to 0 (this is world to camera)

		this->frame_buffer.push_front(f); // add the frame to the front of the buffer

		this->replenishFeatures((this->frame_buffer.front()));
	} else // we have atleast 1 frame in the buffer
	{

		this->frame_buffer.push_front(f); // add the frame to the front of the buffer

		// attempt to flow features into the next frame
		this->updateFeatures(*std::next(this->frame_buffer.begin(), 1), this->frame_buffer.front());

		//predict and set the current pose guess
		this->frame_buffer.front().setPose(std::next(this->frame_buffer.begin(), 1)->getPose()); // temp assume zero velocity

		// make the final determination whether or not we are initialized
		if(!this->initialized)
		{
			if(this->frame_buffer.front().getValidCount() >= START_FEATURE_COUNT)
			{
				// set all current valid features to mature
				this->frame_buffer.front().setAllPointsMature();
				this->initialized = true; // ready to run motion estimation
			}
			else
			{
				ROS_WARN_STREAM("it is taking too long to reach " << START_FEATURE_COUNT << " features. lower tweak the feature detection parameters");
			}
		}

		if(this->initialized) // run moba and sba if initialized
		{
			// attempt to compute our new camera pose from flowed features and their respective depth/position
			double ppe = 0;
			bool moba_passed = this->optimizePose(this->frame_buffer.front(), ppe);

			this->replenishFeatures((this->frame_buffer.front())); // try to get more features if needed

			if(moba_passed)
			{
				this->publishOdometry(*std::next(this->frame_buffer.begin(), 1), this->frame_buffer.front());
			}
			else
			{
				// moba did not pass so tracking is lost
				this->tracking_lost = true;
			}
		}

	}

#if PUBLISH_INSIGHT
	if(this->frame_buffer.size() > 0)
	{
		this->publishInsight(this->frame_buffer.front());
	}
#endif

	ROS_DEBUG_STREAM("map size: " << this->map.size());

	ROS_ERROR_COND(this->tracking_lost, "lost tracking!");
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

	std::list<Feature>::iterator last_f_feat_iter = last_f.features.begin();

	for (int i = 0; i < status.size(); i++) {
		//TODO remove features at too high of a radius
		if (status.at(i) == 1 && !last_f_feat_iter->obsolete && new_f.isPixelInBox(newPoints.at(i))) { // new - check if the feature is obsolete and remove it if it is
			Feature updated_feature = (*last_f_feat_iter);

			updated_feature.px = newPoints.at(i); // set feature's new pixel location

			updated_feature.setParentFrame(&new_f); // set this features parent frame

			new_f.features.push_back(updated_feature); // add the new feature

			//add this feature to the point's observation buffer with the frame's feature buffer memory location
			new_f.features.back().getPoint()->addObservation(&(new_f.features.back()));

		} else {
			lostFeatures++;
			// this feature is lost so we should/must safely null all references to it and remove it from the map
			last_f_feat_iter->getPoint()->safelyDeletePoint();
		}

		last_f_feat_iter++;
	}

	ROS_DEBUG_STREAM("VO LOST " << lostFeatures << "FEATURES");

}

void VIO::optimizePoints(Frame& f){
	ROS_INFO("OPTIMIZING POINTS");
	for(auto& e : f.features)
	{
		if(!e.obsolete && e.getPoint()->isImmature())
		{
			e.getPoint()->SBA(SBA_MAX_ITERATIONS);

			if(e.getPoint()->getSigma() <= MAXIMUM_FEATURE_DEPTH_ERROR)
			{
				ROS_DEBUG("succesful converge");
			}
			else
			{
				e.obsolete = true; // set the feature to obsolete
				ROS_DEBUG("point failed to converge");
			}
		}
	}
	ROS_DEBUG("DONE OPTIMIZING");
}

void VIO::keyFrameUpdate(){
	Frame* kf; // get the key frame pointer

	// find the key frame
	for(auto& e : this->frame_buffer)
	{
		if(e.isKeyframe())
		{
			kf = &e;
			break;
		}
	}

	//if the camera has translated enough make the current frame a keyframe and begin the point optimization
	double dr = (kf->getPose_inv() * this->frame_buffer.front().getPose()).translation().norm();

	double avgDepth = (kf->getAverageFeatureDepth() + this->frame_buffer.front().getAverageFeatureDepth()) / 2.0;

	double ratio = dr / avgDepth;

	if(ratio > T2ASD)
	{
		ROS_INFO_STREAM("ADDING KEYFRAME WITH TRANSLATION TO SCENE DEPTH RATIO OF: " << ratio);
		this->frame_buffer.front().setKeyFrame(true); // make this frame a keyframe
		this->optimizePoints(this->frame_buffer.front()); // attempt to optimize immature points if they have enough keyframes
	}

}

bool VIO::optimizePose(Frame& f, double& ppe)
{
	bool pass = false;

	ROS_DEBUG_STREAM("found " << f.getMatureCount() << " mature pixels");

	//ROS_WARN_COND(f.getMatureCount() < DANGEROUS_MATURE_FEATURE_COUNT_LEVEL, "mature feature count at dangerous level! " << f.getMatureCount());

	if(f.getMatureCount() >= MINIMUM_TRACKABLE_FEATURES)
	{
		ROS_DEBUG_STREAM("running moba with " << f.getMatureCount() << " mature features only. out of: " << f.getValidCount());
		pass = this->MOBA(f, ppe, false);
	}
	else if(f.getValidCount() >= MINIMUM_TRACKABLE_FEATURES)
	{
		ROS_WARN_STREAM("pauvsi_vio: DANGEROUS! too few MATURE features. forced to run motion only bundle adjustment with all " << f.getValidCount() << " features");
		pass = this->MOBA(f, ppe, true);
	}
	else
	{
		ROS_ERROR("pauvsi_vio: ran out of valid features lost track of pose. try lowering the FAST feature threshold.");
		pass = false;
	}

	return pass;
}

bool VIO::MOBA(Frame& f, double& perPixelError, bool useImmature)
{
	double chi2(0.0);
	std::vector<double> chi2_vec_init, chi2_vec_final;

	std::vector<Feature*> edges;

	Sophus::SE3d currentGuess = f.getPose(); // stores the current best guess

	Sophus::Matrix6d A; //LHS
	Sophus::Vector6d b; //RHS

	ROS_DEBUG_STREAM("start trans " << currentGuess.translation());

	//store all valid edges
	for(auto& e : f.features)
	{
		if(!e.obsolete)
		{
			if(useImmature || !e.getPoint()->isImmature())
			{
				edges.push_back(&e);
			}
		}
	}

	int edgeCount = edges.size();

	if(edgeCount < 3)
	{
		ROS_DEBUG_STREAM("too few edges to do motion only BA");
		return false;
	}

	ROS_DEBUG_STREAM("found " << edgeCount << " valid points for MOBA");

	//reserve the space for all chi2 of edges
	chi2_vec_init.reserve(edgeCount);
	chi2_vec_final.reserve(edgeCount);

	//run the motion only bundle adjustment
	for(size_t iter = 0; iter < MOBA_MAX_ITERATIONS; iter++)
	{

		b.setZero();
		A.setZero();
		double new_chi2(0.0);

		// compute residual
		for(auto it=edges.begin(); it!=edges.end(); ++it)
		{
			Matrix26d J;
			Eigen::Vector3d xyz_f(currentGuess * (*it)->getPoint()->getWorldCoordinate());
			Frame::jacobian_xyz2uv(xyz_f, J);
			Eigen::Vector2d e = (*it)->getMetricPixel() - Point::toMetricPixel(xyz_f);


			ROS_DEBUG_STREAM("edge chi: " << e.squaredNorm());

			A.noalias() += J.transpose()*J;
			b.noalias() -= J.transpose()*e;
			new_chi2 += e.squaredNorm();
		}

		// solve linear system
		const Sophus::Vector6d dT(A.ldlt().solve(b));

		// check if error increased
		if((iter > 0 && new_chi2 > chi2) || (bool) std::isnan((double)dT[0]))
		{
			ROS_DEBUG_STREAM("it " << iter << "\t FAILURE \t new_chi2 = " << new_chi2);
			break;
		}

		// update the model
		currentGuess = Sophus::SE3d::exp(dT)*currentGuess;
		chi2 = new_chi2;

		ROS_DEBUG_STREAM("optimized trans: " << currentGuess.translation());

		ROS_DEBUG_STREAM("it " << iter << "\t Success \t new_chi2 = " << new_chi2 << "\t norm(dT) = " << dT.norm());

		// stop when converged
		if(dT.norm() <= EPS_MOBA)
			break;
	}

	ROS_DEBUG_STREAM("optimized trans: " << currentGuess.translation());

	f.setPose(currentGuess); // set the new optimized pose


	return true;
}

/*
 * get more features after updating the pose
 */
void VIO::replenishFeatures(Frame& f) {
	//add more features if needed
	cv::Mat img;
	if (FAST_BLUR_SIGMA != 0.0) {
		cv::GaussianBlur(f.img, img, cv::Size(5, 5), FAST_BLUR_SIGMA);
	} else {
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
		for (auto& e : f.features) {
			cv::circle(checkImg, e.px, MIN_NEW_FEATURE_DIST, cv::Scalar(255), -1);
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
			if (checkImg.at<unsigned char>(fast_kp.at(i).pt)) {
				ROS_DEBUG("feature too close to previous feature, not adding");
				needed++; // we need one more now
				continue;
			}


			// remove features at too high of a radius
			if(!f.isPixelInBox(fast_kp.at(i).pt))
			{
				ROS_DEBUG("feature is out of keep box");
				needed++; // we need one more now
				continue;
			}

			// add this new ft to the check img
			cv::circle(checkImg, fast_kp.at(i).pt, MIN_NEW_FEATURE_DIST, cv::Scalar(255), -1);


			ROS_DEBUG_STREAM("adding feature " << fast_kp.at(i).pt);

			Feature new_ft;

			new_ft.px = fast_kp.at(i).pt;
			new_ft.setParentFrame(&f);

			f.features.push_back(new_ft);

			// important - we need to push a point onto the map
			// pushes this feature onto the observation buffer's front
			this->map.push_back(Point(&(f.features.back()))); // the point needs to be linked to the feature's memory location in the frame's feature buffer

			f.features.back().setPoint(&(this->map.back())); // set the features point to the points pos in the map -- they are now linked with their pointers

			f.features.back().getPoint()->setupMapAndPointLocation(
					(--this->map.end()), &(this->map)); // tell the point where it is in memory via an iterator and where the map is so it can delet itself later

#if USE_POINT_CLOUD
			//todo try to initialize using pointcloud projection
#else
			f.features.back().computeObjectPositionWithAverageSceneDepth();
#endif

		}
	}
}

void VIO::tf2rvecAndtvec(tf::Transform tf, cv::Mat& tvec, cv::Mat& rvec) {
	cv::Mat_<double> R =
			(cv::Mat_<double>(3, 3) << tf.getBasis().getRow(0).x(), tf.getBasis().getRow(
					0).y(), tf.getBasis().getRow(0).z(), tf.getBasis().getRow(1).x(), tf.getBasis().getRow(
							1).y(), tf.getBasis().getRow(1).z(), tf.getBasis().getRow(2).x(), tf.getBasis().getRow(
									2).y(), tf.getBasis().getRow(2).z());

	//ROS_DEBUG("setting up tvec and rvec");

	cv::Rodrigues(R, rvec);

	tvec =
			(cv::Mat_<double>(3, 1) << tf.getOrigin().x(), tf.getOrigin().y(), tf.getOrigin().z());

	//ROS_DEBUG_STREAM("tvec: " << tvec << "\nrvec: " << rvec);
}

tf::Transform VIO::rvecAndtvec2tf(cv::Mat tvec, cv::Mat rvec) {
	//ROS_DEBUG("rvectvec to tf");
	cv::Mat_<double> rot;
	cv::Rodrigues(rvec, rot);
	/*ROS_DEBUG_STREAM("rot: " << rot);
	 ROS_DEBUG_STREAM("rvec: " << rvec);*/
	//ROS_DEBUG_STREAM("tvec " << tvec);
	tf::Transform trans;

	trans.getBasis().setValue(rot(0), rot(1), rot(2), rot(3), rot(4), rot(5),
			rot(6), rot(7), rot(8));
	trans.setOrigin(
			tf::Vector3(tvec.at<double>(0), tvec.at<double>(1),
					tvec.at<double>(2)));

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

std::vector<cv::Point2f> VIO::getPixels2fInOrder(Frame& f) {
	std::vector<cv::Point2f> pixels;

	for (auto e : f.features) {
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

void VIO::correctPointers(bool allFrames)
{
	ROS_ASSERT(this->frame_buffer.size() > 0);

	if(allFrames)
	{
		ROS_WARN("cannot correct all frame's pointers");
	}
	else
	{
		for(std::list<Feature>::iterator it = this->frame_buffer.front().features.begin(); it != this->frame_buffer.front().features.end(); it++)
		{
			//ROS_ASSERT(&(*it) == it->getPoint()->getObservations().front());
			it->getPoint()->getObservations().front() = &(*it);
		}
	}
}

void VIO::publishInsight(Frame& f)
{
	cv::Mat img;

	cv::cvtColor(f.img, img, CV_GRAY2BGR);

	for(auto& e : frame_buffer.front().features)
	{
		if(!e.obsolete)
		{
			if(e.getPoint()->isImmature())
			{
				cv::drawMarker(img, e.px, cv::Scalar(0, 255, 255), cv::MARKER_SQUARE, 5);
			}
			else
			{
				cv::drawMarker(img, e.px, cv::Scalar(0, 255, 0), cv::MARKER_SQUARE, 5);
			}
		}
	}

	cv_bridge::CvImage cv_img;

	cv_img.image = img;
	cv_img.header.frame_id = CAMERA_FRAME;
	cv_img.encoding = sensor_msgs::image_encodings::BGR8;

	this->insight_pub.publish(cv_img.toImageMsg());
	ROS_DEBUG("end publish");
}

void VIO::publishOdometry(Frame& last_f, Frame& new_f)
{
	nav_msgs::Odometry msg;

	tf::Transform currentPose = (c2b * Frame::sophus2tf(new_f.getPose()));

	tf::Transform delta = (c2b * Frame::sophus2tf(last_f.getPose())).inverse() * currentPose;

	double dt = (new_f.t - last_f.t).toSec();

	double r, p, y;
	delta.getBasis().getRPY(r, p, y);

	msg.child_frame_id = BASE_FRAME;
	msg.header.frame_id = WORLD_FRAME;
	msg.twist.twist.angular.x = r / dt;
	msg.twist.twist.angular.y = p / dt;
	msg.twist.twist.angular.z = y / dt;

	msg.twist.twist.linear.x = delta.getOrigin().x() / dt;
	msg.twist.twist.linear.y = delta.getOrigin().y() / dt;
	msg.twist.twist.linear.z = delta.getOrigin().z() / dt;

	msg.pose.pose.orientation.w = new_f.getPose().unit_quaternion().w();
	msg.pose.pose.orientation.x = new_f.getPose().unit_quaternion().x();
	msg.pose.pose.orientation.y = new_f.getPose().unit_quaternion().y();
	msg.pose.pose.orientation.z = new_f.getPose().unit_quaternion().z();

	msg.pose.pose.position.x = new_f.getPose().translation().x();
	msg.pose.pose.position.y = new_f.getPose().translation().y();
	msg.pose.pose.position.z = new_f.getPose().translation().z();

	//TODO add convariance computation

	this->odom_pub.publish(msg); // publish

}
