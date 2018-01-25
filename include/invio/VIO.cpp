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
	//set tracking lost to false initially
	this->tracking_lost = false;

	ros::NodeHandle nh; // we all know what this is

	//parseROSParams();
	ros::param::param<bool>("~publish_insight", PUBLISH_INSIGHT, D_PUBLISH_INSIGHT);
	ros::param::param<std::string>("~insight_topic", INSIGHT_TOPIC, D_INSIGHT_TOPIC);
	ros::param::param<std::string>("~insight_camera_info_topic", INSIGHT_CINFO_TOPIC, D_INSIGHT_CINFO_TOPIC);
	ros::param::param<int>("~fast_threshold", FAST_THRESHOLD, D_FAST_THRESHOLD);
	ros::param::param<double>("~fast_blur_sigma", FAST_BLUR_SIGMA, D_FAST_BLUR_SIGMA);
	ros::param::param<double>("~inverse_image_scale", INVERSE_IMAGE_SCALE, D_INVERSE_IMAGE_SCALE);
	ros::param::param<bool>("~analyze_runtime", ANALYZE_RUNTIME, D_ANALYZE_RUNTIME);
	ros::param::param<int>("~kill_pad", KILL_PAD, D_KILL_PAD);
	ros::param::param<double>("~border_weight_exponent", BORDER_WEIGHT_EXPONENT, D_BORDER_WEIGHT_EXPONENT);
	ros::param::param<double>("~min_klt_eigen_val", KLT_MIN_EIGEN, D_KLT_MIN_EIGEN);
	ros::param::param<double>("~min_new_feature_dist", MIN_NEW_FEATURE_DIST, D_MIN_NEW_FEATURE_DIST);
	ros::param::param<int>("~num_features", NUM_FEATURES, D_NUM_FEATURES);
	ros::param::param<int>("~start_feature_count", START_FEATURE_COUNT, D_START_FEATURE_COUNT);
	ros::param::param<int>("~dangerous_mature_feature_count", DANGEROUS_MATURE_FEATURE_COUNT_LEVEL, D_DANGEROUS_MATURE_FEATURE_COUNT_LEVEL);
	ros::param::param<int>("~minimum_trackable_features", MINIMUM_TRACKABLE_FEATURES, D_MINIMUM_TRACKABLE_FEATURES);
	ros::param::param<int>("~frame_buffer_size", FRAME_BUFFER_SIZE, D_FRAME_BUFFER_SIZE);
	ros::param::param<int>("~minimum_keyframe_count_for_optimization", MINIMUM_KEYFRAME_COUNT_FOR_OPTIMIZATION, D_MINIMUM_KEYFRAME_COUNT_FOR_OPTIMIZATION);
	ros::param::param<int>("~maximum_keyframe_count_for_optimization", MAXIMUM_KEYFRAME_COUNT_FOR_OPTIMIZATION, D_MAXIMUM_KEYFRAME_COUNT_FOR_OPTIMIZATION);
	ros::param::param<double>("~depth_translation_ratio", MIN_T2D, D_MIN_T2D);
	ros::param::param<double>("~max_depth_updates_per_frame", MAX_DEPTH_UPDATES_PER_FRAME, D_MAX_DEPTH_UPDATES_PER_FRAME);
	ros::param::param<double>("~maximum_reprojection_error", MAXIMUM_REPROJECTION_ERROR, D_MAXIMUM_REPROJECTION_ERROR);
	ros::param::param<double>("~moba_candidate_variance", MOBA_CANDIDATE_VARIANCE, D_MOBA_CANDIDATE_VARIANCE);
	ros::param::param<double>("~maximum_candidate_reprojection_error", MAXIMUM_CANDIDATE_REPROJECTION_ERROR, D_MAXIMUM_CANDIDATE_REPROJECTION_ERROR);
	ros::param::param<double>("~default_point_depth", DEFAULT_POINT_DEPTH, D_DEFAULT_POINT_DEPTH);
	ros::param::param<double>("~default_point_depth_variance", DEFAULT_POINT_DEPTH_VARIANCE, D_DEFAULT_POINT_DEPTH_VARIANCE);
	ros::param::param<double>("~default_point_homogenous_variance", DEFAULT_POINT_HOMOGENOUS_VARIANCE, D_DEFAULT_POINT_HOMOGENOUS_VARIANCE);
	ros::param::param<double>("~eps_moba", EPS_MOBA, D_EPS_MOBA);
	ros::param::param<double>("~eps_sba", EPS_SBA, D_EPS_SBA);
	ros::param::param<double>("~huber_width", HUBER_WIDTH, D_HUBER_WIDTH);
	ros::param::param<double>("~minumum_depth_determinant", MINIMUM_DEPTH_DETERMINANT, D_MINIMUM_DEPTH_DETERMINANT);
	ros::param::param<int>("~moba_max_iterations", MOBA_MAX_ITERATIONS, D_MOBA_MAX_ITERATIONS);
	ros::param::param<int>("~sba_max_iterations", SBA_MAX_ITERATIONS, D_SBA_MAX_ITERATIONS);
	ros::param::param<double>("~max_point_z", MAX_POINT_Z, D_MAX_POINT_Z);
	ros::param::param<double>("~min_point_z", MIN_POINT_Z, D_MIN_POINT_Z);
	ros::param::param<std::string>("~odom_topic", ODOM_TOPIC, D_ODOM_TOPIC);
	ros::param::param<std::string>("~odom_frame", ODOM_FRAME, D_ODOM_FRAME);
	ros::param::param<std::string>("~point_topic", POINTS_TOPIC, D_POINTS_TOPIC);
	ros::param::param<std::string>("~camera_topic", CAMERA_TOPIC, D_CAMERA_TOPIC);
	ros::param::param<std::string>("~base_frame", BASE_FRAME, D_BASE_FRAME);
	ros::param::param<std::string>("~world_frame", WORLD_FRAME, D_WORLD_FRAME);
	ros::param::param<std::string>("~camera_frame", CAMERA_FRAME, D_CAMERA_FRAME);
	ros::param::param<std::string>("~imu_topic", IMU_TOPIC, D_IMU_TOPIC);
	ros::param::param<std::string>("~imu_frame", IMU_FRAME, D_IMU_FRAME);
	ros::param::param<bool>("~use_imu", USE_IMU, D_USE_IMU);
	ros::param::param<int>("~min_variance_box_size", MIN_VARIANCE_SIZE, D_MIN_VARIANCE_SIZE);
	ros::param::param<int>("~max_variance_box_size", MAX_VARIANCE_SIZE, D_MAX_VARIANCE_SIZE);
	ros::param::param<int>("~max_pyramids", MAX_PYRAMID_LEVEL, D_MAX_PYRAMID_LEVEL);
	ros::param::param<int>("~klt_window_size", WINDOW_SIZE, D_WINDOW_SIZE);

	image_transport::ImageTransport it(nh);
	image_transport::CameraSubscriber bottom_cam_sub = it.subscribeCamera(
			CAMERA_TOPIC, 10, &VIO::camera_callback, this);

	if(PUBLISH_INSIGHT){
		this->insight_pub = nh.advertise<sensor_msgs::Image>(INSIGHT_TOPIC, 1);
		this->insight_cinfo_pub = nh.advertise<sensor_msgs::CameraInfo>(INSIGHT_CINFO_TOPIC, 1);
	}

	//set up IMU sub
	if(USE_IMU){
		this->imu_sub = nh.subscribe(IMU_TOPIC, 1000, &VIO::imu_callback, this);
	}

	this->odom_pub = nh.advertise<nav_msgs::Odometry>(ODOM_TOPIC, 1);

	this->points_pub = nh.advertise<sensor_msgs::PointCloud>(POINTS_TOPIC, 1);

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


void VIO::imu_callback(const sensor_msgs::ImuConstPtr& msg){
	ROS_DEBUG_STREAM("got imu message: " << msg->header.stamp);
}

void VIO::camera_callback(const sensor_msgs::ImageConstPtr& img,
		const sensor_msgs::CameraInfoConstPtr& cam) {
	static int dt_count = 1;
	static double dt_sum = 0;

	ros::Time start = ros::Time::now();

	cv::Mat temp = cv_bridge::toCvShare(img, img->encoding)->image.clone();

	Frame f = Frame(INVERSE_IMAGE_SCALE, temp.clone(), cam->K, cam->D, img->header.stamp);

	ROS_DEBUG_STREAM("start");

	this->addFrame(f);

	double current_dt = (ros::Time::now() - start).toSec() * 1000.0;
	dt_sum += current_dt;

	ROS_INFO_STREAM("average dt: " << dt_sum/dt_count << " this dt: " << current_dt);
	dt_count++;
}

void VIO::addFrame(Frame f) {

	if (this->frame_buffer.size() == 0) // if this is the first frame that we are receiving
	{
		ROS_DEBUG("adding the first frame");
		//f.setPose(Frame::tf2sophus(b2c)); // set the initial position to 0 (this is world to camera)

		this->frame_buffer.push_front(f); // add the frame to the front of the buffer

		this->replenishFeatures((this->frame_buffer.front()));
	} else // we have atleast 1 frame in the buffer
	{

		this->frame_buffer.push_front(f); // add the frame to the front of the buffer

		// make the final determination whether or not we are initialized
		if(!this->initialized)
		{
			ROS_ASSERT(START_FEATURE_COUNT >= 0);
			if(this->tc_ekf.features.size() >= (unsigned)START_FEATURE_COUNT)
			{
				this->initialized = true; // ready to run motion estimation
			}
			else
			{
				ROS_WARN_STREAM("it is taking too long to reach " << START_FEATURE_COUNT << " features. lower tweak the feature detection parameters");
			}
		}


		if(this->initialized) // run moba and depth update if initialized
		{
			//set the predicted pose of the current frame


			// attempt to flow features into the next frame if there are features
			this->updateStateWithNewImage(this->frame_buffer.at(1), this->frame_buffer.front());
		}

		this->replenishFeatures((this->frame_buffer.front())); // try to get more features if needed
	}


	// publish visualization info

	if( PUBLISH_INSIGHT)
	{
		if(this->frame_buffer.size() > 0)
		{
			this->publishInsight(this->frame_buffer.front());
		}
	}

	// publish odometry
	this->publishOdometry(this->frame_buffer.front());

	//publish the mature 3d points
	this->publishPoints(this->frame_buffer.front());

	ROS_ERROR_COND(this->tracking_lost, "lost tracking!");

	//finally remove excess frames from the buffer
	this->removeExcessFrames(this->frame_buffer);
}

void VIO::removeExcessFrames(std::deque<Frame>& buffer)
{
	// remove the last element if the buffer is larger than the desired size
	if(buffer.size() > (size_t)FRAME_BUFFER_SIZE)
	{
		buffer.pop_back();
	}
}

void VIO::updateStateWithNewImage(Frame& lf, Frame& cf){

	//create the containers for the results of the klt tracking
	std::vector<Eigen::Vector2f> new_positions;
	std::vector<Eigen::Matrix2f> covariance_estimate;
	std::vector<bool> pass;

	//run the klt tracker
	this->tracker.findNewFeaturePositions(lf, cf, this->tc_ekf.previousFeaturePositionVector(), this->tc_ekf.features, new_positions, covariance_estimate, pass);

	this->tc_ekf.updateWithFeaturePositions(new_positions, covariance_estimate, pass);

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

	ROS_DEBUG_STREAM("current 2d feature count: " << tc_ekf.features.size());

	if (tc_ekf.features.size() < (size_t)NUM_FEATURES) {

		std::vector<Eigen::Vector2f> new_features;

		std::vector<cv::KeyPoint> fast_kp;

		cv::FAST(img, fast_kp, FAST_THRESHOLD, true);

		int needed = NUM_FEATURES - tc_ekf.features.size();

		ROS_DEBUG_STREAM("need " << needed << "more features");

		/*cv::flann::Index tree;

		 if (this->state.features.size() > 0) {
		 std::vector<cv::Point2f> prev = this->state.getPixels2fInOrder();
		 tree = cv::flann::Index(cv::Mat(prev).reshape(1),
		 cv::flann::KDTreeIndexParams());
		 }*/

		//image which is used to check if a close feature already exists
		cv::Mat checkImg = cv::Mat::zeros(img.size(), CV_8U);
		for (auto& e : tc_ekf.features) {
			cv::circle(checkImg, e.getPixel(f), MIN_NEW_FEATURE_DIST, cv::Scalar(255), -1);
		}

		for (int i = 0; i < needed && (size_t)i < fast_kp.size(); i++) {
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
				//ROS_DEBUG("feature too close to previous feature, not adding");
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


			//ROS_DEBUG_STREAM("adding feature " << fast_kp.at(i).pt);


			new_features.push_back(Feature::pixel2Metric(f, fast_kp.at(i).pt));

		}

		//add the new features to the current state
		tc_ekf.addNewFeatures(new_features);
	}

}

/*
 * covariance and mean must be in pixels
 */
cv::RotatedRect VIO::getErrorEllipse(double chisquare_val, cv::Point2f mean, Eigen::Matrix2f eig_covmat){

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

void VIO::publishInsight(Frame& f)
{
	cv::Mat img;

	cv::cvtColor(f.img, img, CV_GRAY2BGR);

	int i = 0; // track the feature count
	for(auto& e : tc_ekf.features)
	{
		if(!e.flaggedForDeletion())
		{
			//ROS_DEBUG_STREAM(e.getPixel(f));
			cv::drawMarker(img, e.getPixel(f), cv::Scalar(0, 255, 0), cv::MARKER_SQUARE, 22, 1);

			//ROS_DEBUG_STREAM("plotting covariance in pixels: " << this->tc_ekf.getMetric2PixelMap(f.K)*this->tc_ekf.getFeatureHomogenousCovariance(i)*this->tc_ekf.getMetric2PixelMap(f.K).transpose());
			Eigen::SparseMatrix<float> J = this->tc_ekf.getMetric2PixelMap(f.K);

			cv::RotatedRect rr = this->getErrorEllipse(0.99, e.getPixel(f), J*this->tc_ekf.getFeatureHomogenousCovariance(i)*J);
			//ROS_DEBUG_STREAM(rr.size);
			cv::ellipse(img, rr, cv::Scalar(255, 255, 0), 1);
		}
		// next feature
		i++;
	}

	sensor_msgs::CameraInfo cinfo;

	cinfo.header.frame_id = ODOM_FRAME;
	cinfo.header.stamp = f.t;

	cinfo.height = img.rows;
	cinfo.width = img.cols;

	cinfo.K.at(0) = f.K(0);
	cinfo.K.at(1) = f.K(1);
	cinfo.K.at(2) = f.K(2);
	cinfo.K.at(3) = f.K(3);
	cinfo.K.at(4) = f.K(4);
	cinfo.K.at(5) = f.K(5);
	cinfo.K.at(6) = f.K(6);
	cinfo.K.at(7) = f.K(7);
	cinfo.K.at(8) = f.K(8);

	//TODO make it the actual projection mat
	cinfo.P.at(0) = f.K(0);
	cinfo.P.at(2) = f.K(2);
	cinfo.P.at(5) = f.K(4);
	cinfo.P.at(6) = f.K(5);
	cinfo.P.at(10) = 1.0;

	//TODO add distortion coeffs

	this->insight_cinfo_pub.publish(cinfo);

	cv_bridge::CvImage cv_img;

	cv_img.image = img;
	cv_img.header.frame_id = ODOM_FRAME;
	cv_img.header.stamp = f.t;
	cv_img.encoding = sensor_msgs::image_encodings::BGR8;

	this->insight_pub.publish(cv_img.toImageMsg());
	ROS_DEBUG("end publish");
}

void VIO::publishOdometry(Frame& cf)
{
	nav_msgs::Odometry msg;
	static tf::TransformBroadcaster br;

	tf::Transform currentPose = (c2b);

	br.sendTransform(tf::StampedTransform(currentPose, cf.t, WORLD_FRAME, ODOM_FRAME));

	/*tf::Transform delta = (c2b * Frame::sophus2tf(last_f.getPose())).inverse() * currentPose;

	double dt = (new_f.t - last_f.t).toSec();

	if(dt == 0)
	{
		ROS_WARN_STREAM("dt bewteen frames is zero skipping odom derivation!");
		return;
	}

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

	//set the velocities globally
	this->velocity_set = true;
	this->velocity = Eigen::Vector3d(msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z);
	this->omega = Eigen::Vector3d(msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z);

	msg.pose.pose.orientation.w = new_f.getPose().unit_quaternion().w();
	msg.pose.pose.orientation.x = new_f.getPose().unit_quaternion().x();
	msg.pose.pose.orientation.y = new_f.getPose().unit_quaternion().y();
	msg.pose.pose.orientation.z = new_f.getPose().unit_quaternion().z();

	msg.pose.pose.position.x = new_f.getPose().translation().x();
	msg.pose.pose.position.y = new_f.getPose().translation().y();
	msg.pose.pose.position.z = new_f.getPose().translation().z();

	//TODO add convariance computation

	this->odom_pub.publish(msg); // publish*/

}

void VIO::publishPoints(Frame& f)
{

	sensor_msgs::PointCloud msg;

	sensor_msgs::ChannelFloat32 ch;

	ch.name = "intensity";

	msg.header.stamp = f.t;
	msg.header.frame_id = ODOM_FRAME;

	/*
	for(auto e : f.features)
	{
		if(!e.obsolete)
		{
			if(!e.getPoint()->isImmature())
			{
				Eigen::Vector3d p_in_f = f.getPose_inv() * e.getPoint()->getWorldCoordinate();

				geometry_msgs::Point32 pt;

				pt.x = p_in_f.x();
				pt.y = p_in_f.y();
				pt.z = p_in_f.z();

				ch.values.push_back(f.img.at<uchar>(e.px));

				msg.points.push_back(pt);
			}
		}
	}

	msg.channels.push_back(ch);

	this->points_pub.publish(msg);
	 */

}



