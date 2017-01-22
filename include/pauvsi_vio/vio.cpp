/*
 * vio.cpp
 *
 *  Created on: Sep 19, 2016
 *      Author: kevinsheridan
 */

#include "vio.h"


/*
 * starts all state vectors at 0.0
 */
VIO::VIO()
{
	this->readROSParameters();

	//tf2_ros::TransformListener tf_listener(tfBuffer); // starts a thread which keeps track of transforms in the system

	//feature tracker pass it its params
	this->feature_tracker.setParams(FEATURE_SIMILARITY_THRESHOLD, MIN_EIGEN_VALUE,
			KILL_BY_DISSIMILARITY, NUM_FEATURES, MIN_EIGEN_VALUE);

	//set up image transport
	image_transport::ImageTransport it(nh);
	this->cameraSub = it.subscribeCamera(this->getCameraTopic(), 1, &VIO::cameraCallback, this);

	//setup imu sub
	this->imuSub = nh.subscribe(this->getIMUTopic(), 100, &VIO::imuCallback, this);

	ekf.setGravityMagnitude(this->GRAVITY_MAG); // set the gravity mag

	this->broadcastWorldToOdomTF();

	//setup pointcloudPublisher
	if(PUBLISH_ACTIVE_FEATURES)
		activePointsPub = nh.advertise<sensor_msgs::PointCloud>("/vio/activefeatures", 100);

	initialized = false; //not intialized yet

	//push two frames into the fb
	this->frameBuffer.push_front(Frame());
	this->frameBuffer.push_front(Frame());

	//ensure that both frames have a valid state
	this->currentFrame().state = this->state;
	this->lastFrame().state = this->state;

	//keyframe setup
	this->keyFrames = std::vector<KeyFrameInfo>(NUM_KEYFRAMES);
}

VIO::~VIO()
{

}

void VIO::cameraCallback(const sensor_msgs::ImageConstPtr& img, const sensor_msgs::CameraInfoConstPtr& cam)
{
	ros::Time start = ros::Time::now();
	cv::Mat temp = cv_bridge::toCvShare(img, "mono8")->image.clone();

	//set the K and D matrices
	this->setK(get3x3FromVector(cam->K));
	this->setD(cv::Mat(cam->D, false));

	//undistort the image using the fisheye model
	//ROS_ASSERT(cam->distortion_model == "fisheye");
	//cv::fisheye::undistortImage(temp, temp, this->K, this->D, this->K);

	// set the current frame
	this->setCurrentFrame(temp, cv_bridge::toCvCopy(img, "mono8")->header.stamp);

	//set the current frame's K & D
	this->currentFrame().K = this->K;
	this->currentFrame().D = this->D;

	// process the frame correspondences
	this->run();

	//get the run time
	ROS_DEBUG_STREAM((ros::Time::now().toSec() - start.toSec()) * 1000 << " milliseconds runtime");

	//this->viewImage(this->currentFrame());
}

void VIO::imuCallback(const sensor_msgs::ImuConstPtr& msg)
{
	//ROS_DEBUG_STREAM_THROTTLE(0.1, "accel: " << msg->linear_acceleration);
	this->ekf.addIMUMessage(*msg);
	//ROS_DEBUG_STREAM("time compare " << ros::Time::now().toNSec() - msg->header.stamp.toNSec());
}

cv::Mat VIO::get3x3FromVector(boost::array<double, 9> vec)
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


/*
 * sets the current frame and computes important
 * info about it
 * finds corners
 * describes corners
 */
void VIO::setCurrentFrame(cv::Mat img, ros::Time t)
{
	this->frameBuffer.push_front(Frame(img, t, lastFrame().nextFeatureID)); // create a frame with a starting ID of the last frame's next id

	// pop back if que is longer than the size
	if(this->frameBuffer.size() > this->FRAME_BUFFER_LENGTH)
	{
		this->frameBuffer.pop_back();
	}
}

/*
 * runs:
 * feature detection, ranking, flowing
 * motion estimation
 * feature mapping
 */
void VIO::run()
{
	// if there is a last frame, flow features and estimate motion
	if(lastFrame().isFrameSet())
	{
		if(lastFrame().features.size() > 0)
		{
			feature_tracker.flowFeaturesToNewFrame(this->frameBuffer.at(1), this->frameBuffer.at(0));
			currentFrame().cleanUpFeaturesByKillRadius(this->KILL_RADIUS);
			//this->checkFeatureConsistency(currentFrame, this->FEATURE_SIMILARITY_THRESHOLD);

			currentFrame().undistortFeatures(); // undistort the new features
		}

		//MOTION ESTIMATION
		this->lastState = this->state;
		this->state = this->estimateMotion(this->lastState, this->lastFrame(), this->currentFrame());
		//set the currentFrames new state
		this->currentFrame().state = this->state;

		if(this->initialized) // if initialized
		{
			//UPDATE 3D ACTIVE AND INACTIVE FEATURES
			//this->update3DFeatures();
		}
	}

	//check the number of 2d features in the current frame
	//if this is below the required amount refill the feature vector with
	//the best new feature. It must not be redundant.

	//ROS_DEBUG_STREAM("feature count: " << currentFrame.features.size());

	if(currentFrame().features.size() < this->NUM_FEATURES)
	{
		//add n new unique features
		//ROS_DEBUG("low on features getting more");
		currentFrame().getAndAddNewFeatures(this->NUM_FEATURES - currentFrame().features.size(), this->FAST_THRESHOLD, this->KILL_RADIUS, this->MIN_NEW_FEATURE_DISTANCE);
		//currentFrame.describeFeaturesWithBRIEF();

		currentFrame().undistortFeatures(); // undistort the new features
	}

	this->broadcastWorldToOdomTF();
	this->publishActivePoints();

	//ROS_DEBUG_STREAM("imu readings: " << this->imuMessageBuffer.size());
	ROS_DEBUG_STREAM("frame: " << this->frameBuffer.size() << " init: " << initialized);
}


/*
 * recalibrates the state using average pixel motion
 * uses an Extended Kalman Filter to predict and update the state and its
 * covariance.
 */
VIOState VIO::estimateMotion(VIOState x, Frame lf, Frame cf)
{
	//RECALIBRATION
	static bool consecutiveRecalibration = false;
	double avgFeatureChange = feature_tracker.averageFeatureChange(lf, cf); // get the feature change between f1 and f2

	//recalibrate the state using avg pixel change and track consecutive runs
	if(avgFeatureChange <= this->RECALIBRATION_THRESHOLD)
	{
		this->recalibrateState(avgFeatureChange, this->RECALIBRATION_THRESHOLD, consecutiveRecalibration);
		consecutiveRecalibration = true;
	}
	else
	{
		consecutiveRecalibration = false;
	}

	//MOTION ESTIMATION
	VIOState newX = x; // set newX to last x

	//if the camera moves more than the minimum START distance
	//start the motion estimate
	//set the system to initialized
	if(this->initialized == true || avgFeatureChange > this->INIT_PXL_DELTA)
	{
		this->initialized = true; // this is the initialize step

		//run ekf predict step.
		//this will update the state using imu measurements
		//it will also propagate the error throughout the predction step into the states covariance matrix
		VIOState pred = ekf.predict(x, cf.timeImageCreated);

		//NEXT
		//We must predict motion using either the triangulated 3d points or the key frames and their corresponding points
		this->updateKeyFrameInfo(); // finds new keyframes for the currentframe
		this->sortActive3DFeaturesByVariance(); // this sorts the active 3d features according to their variances in ascending order

		for(int i = 0; i < keyFrames.size(); i++)
		{
			cv::Mat E;
			cv::Matx33f R;
			cv::Matx31f t;
			double essential_error = this->computeFundamentalMatrix(E, R, t, keyFrames.at(i));

			ROS_DEBUG_STREAM("KF " << i << " t: " << t << " error: " << essential_error << " pixel delta: " << keyFrames.at(i).pixelDelta);
			ROS_DEBUG_STREAM(R);
		}

		newX = pred;

	}
	else //REMOVE ALL IMU MESSAGES WHICH WERE NOT USED FROM THE BUFFER IF NOT INITIALIZED YET
	{
		std::vector<sensor_msgs::Imu> newBuff;

		// empty the imu buffer
		for(int i = 0; i < this->ekf.imuMessageBuffer.size(); i++)
		{
			if(this->ekf.imuMessageBuffer.at(i).header.stamp.toSec() >= cf.timeImageCreated.toSec())
			{
				newBuff.push_back(this->ekf.imuMessageBuffer.at(i));
			}
		}

		this->ekf.imuMessageBuffer = newBuff; // replace the buffer
	}

	this->drawKeyFrames();
	return newX;
}


/*
 * publishes all active points in the list using the publisher if the user has specified
 */
void VIO::publishActivePoints()
{
	if(this->PUBLISH_ACTIVE_FEATURES)
	{

		sensor_msgs::PointCloud pc;

		std::vector<geometry_msgs::Point32> point;
		std::vector<sensor_msgs::ChannelFloat32> colors;

		pc.header.frame_id = this->world_frame;

		for(int i = 0; i < this->active3DFeatures.size(); i++)
		{
			//debugFeature(this->active3DFeatures.at(i));

			std::vector<float> intensity;
			sensor_msgs::ChannelFloat32 c;

			intensity.push_back(this->active3DFeatures.at(i).color.val[0]);
			//intensity.push_back(this->active3DFeatures.at(i).color[1]);
			//intensity.push_back(this->active3DFeatures.at(i).color[2]);

			c.values = intensity;
			c.name = "intensity";

			geometry_msgs::Point32 pt;
			pt.x = this->active3DFeatures.at(i).position.x();
			pt.y = this->active3DFeatures.at(i).position.y();
			pt.z = this->active3DFeatures.at(i).position.z();

			point.push_back(pt);
			colors.push_back(c);

		}

		pc.points = point;
		pc.channels = colors;

		this->activePointsPub.publish(pc); // publish!
	}
}


/*
 * gets parameters from ROS param server
 */
void VIO::readROSParameters()
{
	//CAMERA TOPIC
	ROS_WARN_COND(!ros::param::has("~cameraTopic"), "Parameter for 'cameraTopic' has not been set");
	ros::param::param<std::string>("~cameraTopic", cameraTopic, DEFAULT_CAMERA_TOPIC);
	ROS_DEBUG_STREAM("camera topic is: " << cameraTopic);

	//IMU TOPIC
	ROS_WARN_COND(!ros::param::has("~imuTopic"), "Parameter for 'imuTopic' has not been set");
	ros::param::param<std::string>("~imuTopic", imuTopic, DEFAULT_IMU_TOPIC);
	ROS_DEBUG_STREAM("IMU topic is: " << imuTopic);

	ros::param::param<std::string>("~imu_frame_name", imu_frame, DEFAULT_IMU_FRAME_NAME);
	ros::param::param<std::string>("~camera_frame_name", camera_frame, DEFAULT_CAMERA_FRAME_NAME);
	ros::param::param<std::string>("~odom_frame_name", odom_frame, DEFAULT_ODOM_FRAME_NAME);
	ros::param::param<std::string>("~center_of_mass_frame_name", CoM_frame, DEFAULT_COM_FRAME_NAME);
	ros::param::param<std::string>("~world_frame_name", world_frame, DEFAULT_WORLD_FRAME_NAME);
	ekf.imu_frame = imu_frame;
	ekf.camera_frame = camera_frame;
	ekf.odom_frame = odom_frame;
	ekf.CoM_frame = CoM_frame;
	ekf.world_frame = world_frame;

	ros::param::param<int>("~fast_threshold", FAST_THRESHOLD, DEFAULT_FAST_THRESHOLD);

	ros::param::param<float>("~feature_kill_radius", KILL_RADIUS, DEFAULT_2D_KILL_RADIUS);

	ros::param::param<int>("~feature_similarity_threshold", FEATURE_SIMILARITY_THRESHOLD, DEFAULT_FEATURE_SIMILARITY_THRESHOLD);
	ros::param::param<bool>("~kill_by_dissimilarity", KILL_BY_DISSIMILARITY, false);

	ros::param::param<float>("~min_eigen_value", MIN_EIGEN_VALUE, DEFAULT_MIN_EIGEN_VALUE);

	ros::param::param<int>("~num_features", NUM_FEATURES, DEFAULT_NUM_FEATURES);

	ros::param::param<int>("~min_new_feature_distance", MIN_NEW_FEATURE_DISTANCE, DEFAULT_MIN_NEW_FEATURE_DIST);

	ros::param::param<double>("~starting_gravity_mag", GRAVITY_MAG, DEFAULT_GRAVITY_MAGNITUDE);

	ros::param::param<double>("~recalibration_threshold", RECALIBRATION_THRESHOLD, DEFAULT_RECALIBRATION_THRESHOLD);

	ros::param::param<bool>("~publish_active_features", PUBLISH_ACTIVE_FEATURES, DEFAULT_PUBLISH_ACTIVE_FEATURES);

	ros::param::param<std::string>("~active_features_topic", ACTIVE_FEATURES_TOPIC, DEFAULT_ACTIVE_FEATURES_TOPIC);

	ros::param::param<double>("~min_triag_dist", MIN_TRIANGUALTION_DIST, DEFAULT_MIN_TRIANGUALTION_DIST);

	ros::param::param<double>("~pixel_delta_init_thresh", INIT_PXL_DELTA, DEFAULT_INIT_PXL_DELTA);

	ros::param::param<int>("~frame_buffer_length", FRAME_BUFFER_LENGTH, DEFAULT_FRAME_BUFFER_LENGTH);

	ros::param::param<double>("~max_triangulation_error", MAX_TRIAG_ERROR, DEFAULT_MAX_TRIAG_ERROR);
	ros::param::param<double>("~min_triangulation_z", MIN_TRIAG_Z, DEFAULT_MIN_TRIAG_Z);

	ros::param::param<double>("~ideal_fundamental_matrix_pxl_delta", IDEAL_FUNDAMENTAL_PXL_DELTA, DEFAULT_IDEAL_FUNDAMENTAL_PXL_DELTA);
	ros::param::param<double>("~min_fundamental_matrix_pxl_delta", MIN_FUNDAMENTAL_PXL_DELTA, DEFAULT_MIN_FUNDAMENTAL_PXL_DELTA);
	ros::param::param<double>("~max_fundamental_error", MAXIMUM_FUNDAMENTAL_ERROR, DEFAULT_MAX_FUNDAMENTAL_ERROR);

	ros::param::param<int>("~min_triag_features", MIN_TRIAG_FEATURES, DEFAULT_MIN_TRIAG_FEATURES);

	ros::param::param<int>("~max_gauss_newton_iterations", MAX_GN_ITERS, DEFAULT_MAX_GN_ITERS);
}

/*
 * broadcasts the world to odom transform
 */
void VIO::broadcastWorldToOdomTF()
{
	//ROS_DEBUG_STREAM("state " << this->state.vector);
	static tf::TransformBroadcaster br;
	tf::Transform transform;
	transform.setOrigin(tf::Vector3(state.x(), state.y(), state.z()));

	//ROS_DEBUG_STREAM(this->pose.pose.orientation.w << " " << this->pose.pose.orientation.x);
	transform.setRotation(state.getTFQuaternion());
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), this->world_frame, this->odom_frame));
}

/*
 * broadcasts the odom to tempIMU trans
 */
ros::Time VIO::broadcastOdomToTempIMUTF(double roll, double pitch, double yaw, double x, double y, double z)
{
	static tf::TransformBroadcaster br;
	tf::Transform transform;
	transform.setOrigin(tf::Vector3(x, y, z));
	tf::Quaternion q;
	q.setRPY(roll, pitch, yaw);
	//ROS_DEBUG_STREAM(q.getW() << ", " << q.getX() << ", " << q.getY() << ", " << q.getZ());
	transform.setRotation(q);
	ros::Time sendTime = ros::Time::now();
	br.sendTransform(tf::StampedTransform(transform, sendTime, this->camera_frame, "temp_imu_frame"));
	return sendTime;
}

void VIO::correctOrientation(tf::Quaternion q, double certainty)
{
	//check if quats are nan
	ROS_ASSERT(state.q0() == state.q0());
	ROS_ASSERT(q.getW() == q.getW());
	//Takes orientation and rotates it towards q.
	state.setQuaternion(state.getTFQuaternion().slerp(q, certainty));
}


/*
 * map average pixel change from zero to threshold and then make that a value from 0 to 1
 * So, now you have this value if you multiply it times velocity
 */
void VIO::recalibrateState(double avgPixelChange, double threshold, bool consecutive)
{
	//ROS_DEBUG_STREAM("recalibrating with " << avgPixelChange);

	static double lastNormalize = 0;
	static sensor_msgs::Imu lastImu;
	double normalize = avgPixelChange/threshold;
	sensor_msgs::Imu currentImu = ekf.getMostRecentImu();

	//ROS_DEBUG_STREAM("normalized pixel change " << normalize);

	state.setVelocity(normalize * state.getVelocity());
	//state.setVelocity(0 * state.getVelocity());

	//TODO make a gyro bias measurment vector in the inertial motion estimator and do a weighted average

	gyroNode gNode;
	gNode.gyroBias.setX(currentImu.angular_velocity.x);
	gNode.gyroBias.setY(currentImu.angular_velocity.y);
	gNode.gyroBias.setZ(currentImu.angular_velocity.z);
	gNode.certainty = (1-normalize);
	if(gyroQueue.size() >= DEFAULT_QUEUE_SIZE)
	{
		gyroQueue.pop_back();
		gyroQueue.insert(gyroQueue.begin(), gNode);
	}
	else
	{
		gyroQueue.insert(gyroQueue.begin(), gNode);
	}

	double gyroCertaintySum = 0;
	for(int i=0; i<gyroQueue.size(); ++i)
	{
		gyroCertaintySum += gyroQueue.at(i).certainty;
	}
	std::vector<double> gyroNormlizedCertainty;
	for(int i=0; i<gyroQueue.size(); ++i)
	{
		gyroNormlizedCertainty.push_back(gyroQueue.at(i).certainty / gyroCertaintySum);
	}

	//FINAL WEIGHTED GYROBIASES
	gyroNode gWeightedNode;
	gWeightedNode.gyroBias.setX(0);
	gWeightedNode.gyroBias.setY(0);
	gWeightedNode.gyroBias.setZ(0);
	gWeightedNode.certainty = 0;
	for(int i=0; i<gyroQueue.size(); ++i)
	{
		gWeightedNode.gyroBias.setX(gWeightedNode.gyroBias.getX() + gyroNormlizedCertainty.at(i)*gyroQueue.at(i).gyroBias.getX());
		gWeightedNode.gyroBias.setY(gWeightedNode.gyroBias.getY() + gyroNormlizedCertainty.at(i)*gyroQueue.at(i).gyroBias.getY());
		gWeightedNode.gyroBias.setZ(gWeightedNode.gyroBias.getZ() + gyroNormlizedCertainty.at(i)*gyroQueue.at(i).gyroBias.getZ());
	}

	ekf.gyroBiasX = gWeightedNode.gyroBias.getX();
	ekf.gyroBiasY = gWeightedNode.gyroBias.getY();
	ekf.gyroBiasZ = gWeightedNode.gyroBias.getZ();


	//POTENTIAL BUG
	if(consecutive)
	{
		normalize = (normalize+lastNormalize)/2;

		ROS_DEBUG_STREAM("running consecutive calibration with new normalized " << normalize);

		tf::Vector3 accel(lastImu.linear_acceleration.x*ekf.scaleAccelerometer
				, lastImu.linear_acceleration.y*ekf.scaleAccelerometer
				, lastImu.linear_acceleration.z*ekf.scaleAccelerometer);
		double scale = accel.length();

		//Vector with size DEFAULT_QUEUE_SIZE, elements added at front and dequeued at back
		accelNode aNode;
		aNode.certainty = (1-normalize);
		aNode.accelScale = GRAVITY_MAG/scale;
		if(accelQueue.size() >= DEFAULT_QUEUE_SIZE)
		{
			accelQueue.pop_back();
			accelQueue.insert(accelQueue.begin(), aNode);

		}
		else
		{
			accelQueue.insert(accelQueue.begin(), aNode);
		}
		//Calculating weighted values of gyroBiases and scale
		double accelCertaintySum = 0;
		//		queueNode WeightedValues;
		//		WeightedValues.gyroBias.setX(0);
		//		WeightedValues.gyroBias.setY(0);
		//		WeightedValues.gyroBias.setZ(0);
		//		WeightedValues.certainty = 0;
		//		WeightedValues.scale = 0;
		for(int i=0; i<accelQueue.size(); ++i)
		{
			accelCertaintySum += accelQueue.at(i).certainty;
			//			sum.gyroBias.setX(queue.at(i).gyroBias.getX()+sum.gyroBias.getX());
			//			sum.gyroBias.setY(queue.at(i).gyroBias.getY()+sum.gyroBias.getY());
			//			sum.gyroBias.setZ(queue.at(i).gyroBias.getZ()+sum.gyroBias.getZ());
			//			sum.certainty += queue.at(i).certainty;
			//			sum.scale += queue.at(i).scale;//sum += queue.at(i).certainty;//queue.at(i).scale;
		}

		std::vector<double> accelNormalizedCertainty;
		for(int i=0; i<accelQueue.size(); ++i)
		{
			accelNormalizedCertainty.push_back(accelQueue.at(i).certainty / accelCertaintySum);
			//			Node.certainty = queue.at(i).certainty / sum.certainty;
			//			Node.gyroBias.setX(queue.at(i).gyroBias.getX() / sum.gyroBias.getX());
			//			Node.gyroBias.setY(queue.at(i).gyroBias.getY() / sum.gyroBias.getY());
			//			Node.gyroBias.setZ(queue.at(i).gyroBias.getZ() / sum.gyroBias.getZ());
			//			Node.scale = queue.at(i).scale / sum.scale;
			//
			//			weigthedQueue.push_back(Node);
		}

		//FINAL WEIGHTED ACCELERATION SCALE
		accelNode aWeightedNode;
		aWeightedNode.certainty = 0;
		aWeightedNode.accelScale = 0;

		for(int i=0; i<accelQueue.size(); ++i)
		{
			aWeightedNode.accelScale += accelNormalizedCertainty.at(i)*accelQueue.at(i).accelScale;
			//			WeightedValues.gyroBias.setX(WeightedValues.gyroBias.getX() + normalizedCertainty.at(i)*queue.at(i).gyroBias.getX());
			//			WeightedValues.gyroBias.setY(WeightedValues.gyroBias.getY() + normalizedCertainty.at(i)*queue.at(i).gyroBias.getY());
			//			WeightedValues.gyroBias.setZ(WeightedValues.gyroBias.getZ() + normalizedCertainty.at(i)*queue.at(i).gyroBias.getZ());
			//			WeightedValues.scale += normalizedCertainty.at(i)*queue.at(i).scale;
		}
		//sum *= GRAVITY_MAG/queue.size();
		//TODO create a ten element running wieghted average of the accelerometer scale.
		if(scale != 0)
			ekf.scaleAccelerometer = aWeightedNode.accelScale; // + (normalize)*ekf.scaleAccelerometer;

		//tf::Vector3 gravity(0,0,GRAVITY_MAG);

		//correctOrientation(ekf.getDifferenceQuaternion(gravity, accel), (1-normalize));

		//ROS_DEBUG_STREAM("new acceleration after scaling " << ekf.scaleAccelerometer * accel);
	}

	//ROS_DEBUG_STREAM("new accel scale " << ekf.scaleAccelerometer << " new gyro biases " << ekf.gyroBiasX << ", " << ekf.gyroBiasY << ", " << ekf.gyroBiasZ);


	lastImu = currentImu;
	lastNormalize = normalize;
	return;
}


// -=-=-=-=-=-=-=-=-=--=-=-=-=-=-=-=-=-=-=-=-=-=-=
//LEGACY - Version 2

/*



 * computes the F mat
 * gets scales translation


double VIO::computeFundamentalMatrix(cv::Mat& F, cv::Matx33f& R, cv::Matx31f& t, std::vector<cv::Point2f>& pt1_out, std::vector<cv::Point2f>& pt2_out, bool& pass, std::vector<VIOFeature2D>& ft1_out, std::vector<VIOFeature2D>& ft2_out, int& match_frame_index)
{
	double pixel_delta = 0;
	VIOState x1, x2;
	std::vector<VIOFeature2D> ft1, ft2;
	int match_index;

	double error = 0;

	//std::vector<cv::Point2f> pt1_new, pt2_new;
	std::vector<cv::Point2f> pt1, pt2;

	this->getBestCorrespondences(pixel_delta, ft1, ft2, x1, x2, match_index);

	match_frame_index = match_index;

	if(pixel_delta > MIN_FUNDAMENTAL_PXL_DELTA)
	{
		pass = true;

		for(int i = 0; i < ft1.size(); i++)
		{
			pt1.push_back(ft1.at(i).getUndistorted());
			pt2.push_back(ft2.at(i).getUndistorted());
		}

		cv::Mat mask;
		cv::Mat E = cv::findEssentialMat(pt1, pt2, cv::Mat::eye(cv::Size(3, 3), CV_32F), cv::RANSAC, 0.999, 1.0, mask);

		//std::vector<cv::Point2f> pt1_copy, pt2_copy;
		//pt1_copy = pt1;
		//pt2_copy = pt2;
		//cv::correctMatches(E, pt1_copy, pt2_copy, pt1_new, pt2_new);

		//pt1_new = pt1;
		//pt2_new = pt2;

		for(int i = 0; i < pt1.size(); i++)
		{
			cv::Matx31f u1, u2;

			u1(0) = pt1.at(i).x;
			u1(1) = pt1.at(i).y;
			u1(2) = 1.0;

			u2(0) = pt2.at(i).x;
			u2(1) = pt2.at(i).y;
			u2(2) = 1.0;

			Eigen::Matrix<float, 3, 1> u1_eig, u2_eig;
			Eigen::Matrix<float, 3, 3> E_eig;

			cv::cv2eigen(u1, u1_eig);
			cv::cv2eigen(u2, u2_eig);
			cv::cv2eigen(E, E_eig);

			error += abs((u2_eig.transpose() * E_eig * u1_eig)(0, 0));

		}
		//error = 0;

		cv::Mat R_temp, t_hat;

		double goodProb = this->recoverPoseV2(E, pt1, pt2, cv::Mat::eye(cv::Size(3, 3), CV_32F), R_temp, t_hat, mask, frameBuffer.at(match_index).state, currentFrame().state);
		//int goodPoints = cv::recoverPose(E, pt1, pt2, cv::Mat::eye(cv::Size(3, 3), CV_32F), R_temp, t_hat, mask);

		ROS_DEBUG_STREAM("error: " << error);

		R_temp.convertTo(R_temp,  R.type);
		R_temp.copyTo(R);
		t_hat.convertTo(t_hat,  t.type);
		t_hat.copyTo(t);

		//t = ((currentFrame().state.getr() - frameBuffer.at(match_index).state.getr()).norm() * t); // return the scaled translation using the two frames states

		ft1_out = ft1;
		ft2_out = ft2;

		pt1_out = pt1;
		pt2_out = pt2;

		F = E;

		ROS_DEBUG_STREAM("tFinal: " << t);

	}
	else
	{
		ROS_DEBUG_STREAM("pixel delta too small for fundamental mat computation");
		error = 1e9;
		pass = false;
	}

	this->viewMatches(ft1, ft2, this->frameBuffer.at(match_index), currentFrame(), pt1, pt2);

	return error;
}


void VIO::getBestCorrespondences(double& pixel_delta, std::vector<VIOFeature2D>& ft1, std::vector<VIOFeature2D>& ft2, VIOState& x1, VIOState& x2, int& match_index)
{
	ros::Time start = ros::Time::now();

	int bestFtIndex = 0;
	match_index = 0;

	double bestPxlDelta = 0;

	std::deque<std::deque<VIOFeature2D> > temp_ft1;
	std::deque<std::deque<VIOFeature2D> > temp_ft2;

	std::deque<VIOFeature2D> dq;
	for(int i = 0; i < currentFrame().features.size(); i++)
		dq.push_back(currentFrame().features.at(i));

	temp_ft1.push_back(dq);
	temp_ft2.push_back(dq);

	for(int i = 1; i < this->frameBuffer.size(); i++)
	{
		float pxlSum = 0;

		//ROS_DEBUG("t0");

		//ROS_DEBUG_STREAM("last vec size: " << temp_ft1.at(i - 1).size());

		std::deque<VIOFeature2D> t1, t2;

		temp_ft1.push_back(t1);
		temp_ft2.push_back(t2);

		//ros::Time start2 = ros::Time::now();

		for(int j = 0; j < temp_ft1.at(i - 1).size(); j++)
		{
			if(temp_ft1.at(i - 1).at(j).isMatched())
			{
				temp_ft2.at(i).push_back(temp_ft2.at(i - 1).at(j));

				temp_ft1.at(i).push_back(this->getCorrespondingFeature(temp_ft1.at(i - 1).at(j), this->frameBuffer.at(i)));

				pxlSum += this->manhattan(temp_ft1.at(i).at(temp_ft1[i].size() - 1).getUndistorted(), temp_ft2.at(i - 1).at(j).getUndistorted());

			}
		}

		//pxlSum = i;

		//ROS_DEBUG_STREAM((ros::Time::now().toSec() - start2.toSec()) * 1000 << " milliseconds runtime (feature correspondence inner loop)");

		//ROS_DEBUG("t1");

		if(temp_ft1.at(i).size() < MIN_TRIAG_FEATURES)
		{
			//ROS_DEBUG_STREAM("too few features break");
			break;
		}

		if(pxlSum / temp_ft1.at(i).size() > IDEAL_FUNDAMENTAL_PXL_DELTA)
		{
			//ROS_DEBUG_STREAM("ideal pxl delta break");

			bestFtIndex = i;

			bestPxlDelta = pxlSum / temp_ft1.at(i).size();

			match_index = i;
			break;
		}

		if(pxlSum / temp_ft1.size() > bestPxlDelta)
		{
			//ROS_DEBUG_STREAM("better pxlDelta");

			bestFtIndex = i;

			bestPxlDelta = pxlSum / temp_ft1.at(i).size();

			match_index = i;
		}

		//ROS_DEBUG("t2");

	}

	std::copy(temp_ft1.at(bestFtIndex).begin(), temp_ft1.at(bestFtIndex).end(), std::back_inserter(ft1));
	std::copy(temp_ft2.at(bestFtIndex).begin(), temp_ft2.at(bestFtIndex).end(), std::back_inserter(ft2));
	x1 = frameBuffer.at(bestFtIndex).state;
	x2 = frameBuffer.at(0).state;
	pixel_delta = bestPxlDelta;

	ROS_DEBUG_STREAM("best pixel delta " << pixel_delta);
	ROS_DEBUG_STREAM("match index: " << match_index);
	ROS_DEBUG_STREAM((ros::Time::now().toSec() - start.toSec()) * 1000 << " milliseconds runtime (feature correspondence)");
}

*/


//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//LEGACY - Version 1

/*


 * uses epipolar geometry from two frames to
 * estimate relative motion of the frame;

bool VIO::visualMotionInference(Frame frame1, Frame frame2, tf::Vector3 angleChangePrediction,
		tf::Vector3& rotationInference, tf::Vector3& unitVelocityInference, double& averageMovement)
{
	//first get the feature deltas from the two frames
	std::vector<cv::Point2f> prevPoints, currentPoints;
	feature_tracker.getCorrespondingPointsFromFrames(frame1, frame2, prevPoints, currentPoints);

	//undistort points using fisheye model
	//cv::fisheye::undistortPoints(prevPoints, prevPoints, this->K, this->D);
	//cv::fisheye::undistortPoints(currentPoints, currentPoints, this->K, this->D);

	//get average movement bewteen images
	averageMovement = feature_tracker.averageFeatureChange(prevPoints, currentPoints);

	//ensure that there are enough points to estimate motion with vo
	if(currentPoints.size() < 5)
	{
		return false;
	}

	cv::Mat mask;

	//calculate the essential matrix
	cv::Mat essentialMatrix = cv::findEssentialMat(prevPoints, currentPoints, this->K, cv::RANSAC, 0.999, 1.0, mask);

	//ensure that the essential matrix is the correct size
	if(essentialMatrix.rows != 3 || essentialMatrix.cols != 3)
	{
		return false;
	}

	//recover pose change from essential matrix
	cv::Mat translation;
	cv::Mat rotation;

	//decompose matrix to get possible deltas
	cv::recoverPose(essentialMatrix, prevPoints, currentPoints, this->K, rotation, translation, mask);


	//set the unit velocity inference
	unitVelocityInference.setX(translation.at<double>(0, 0));
	unitVelocityInference.setY(translation.at<double>(1, 0));
	unitVelocityInference.setZ(translation.at<double>(2, 0));

	return true;
}

 *
 From "Triangulation", Hartley, R.I. and Sturm, P., Computer vision and image understanding, 1997

cv::Mat_<double> VIO::IterativeLinearLSTriangulation(cv::Point3d u,    //homogenous image point (u,v,1)
		cv::Matx34d P,          //camera 1 matrix
		cv::Point3d u1,         //homogenous image point in 2nd camera
		cv::Matx34d P1          //camera 2 matrix
) {
	//double error;
	double wi = 1, wi1 = 1;
	cv::Mat_<double> X(4,1);
	for (int i=0; i<10; i++) { //Hartley suggests 10 iterations at most
		cv::Mat_<double> X_ = this->LinearLSTriangulation(u,P,u1,P1);
		X(0) = X_(0); X(1) = X_(1); X(2) = X_(2);
		X(3) = X_(3);

		//recalculate weights
		double p2x = cv::Mat_<double>(cv::Mat_<double>(P).row(2)*X)(0);
		double p2x1 = cv::Mat_<double>(cv::Mat_<double>(P1).row(2)*X)(0);

		//breaking point
		//error = fabsf(wi - p2x) + fabsf(wi1 - p2x1);
		if(fabsf(wi - p2x) <= 1.0 && fabsf(wi1 - p2x1) <= 1.0) break;

		wi = p2x;
		wi1 = p2x1;

		//reweight equations and solve
		cv::Matx43d A((u.x*P(2,0)-P(0,0))/wi,       (u.x*P(2,1)-P(0,1))/wi,         (u.x*P(2,2)-P(0,2))/wi,
				(u.y*P(2,0)-P(1,0))/wi,       (u.y*P(2,1)-P(1,1))/wi,         (u.y*P(2,2)-P(1,2))/wi,
				(u1.x*P1(2,0)-P1(0,0))/wi1,   (u1.x*P1(2,1)-P1(0,1))/wi1,     (u1.x*P1(2,2)-P1(0,2))/wi1,
				(u1.y*P1(2,0)-P1(1,0))/wi1,   (u1.y*P1(2,1)-P1(1,1))/wi1,     (u1.y*P1(2,2)-P1(1,2))/wi1
		);
		cv::Mat_<double> B = (cv::Mat_<double>(4,1) <<    -(u.x*P(2,3)    -P(0,3))/wi,
				-(u.y*P(2,3)  -P(1,3))/wi,
				-(u1.x*P1(2,3)    -P1(0,3))/wi1,
				-(u1.y*P1(2,3)    -P1(1,3))/wi1
		);

		cv::solve(A,B,X_,cv::DECOMP_SVD);
		X(0) = X_(0); X(1) = X_(1); X(2) = X_(2);
		X(3) = X_(3);
	}

	//ROS_DEBUG_STREAM("triag error: " << error);
	return X;
}

 *
 From "Triangulation", Hartley, R.I. and Sturm, P., Computer vision and image understanding, 1997

cv::Mat_<double> VIO::LinearLSTriangulation(cv::Point3d u,       //homogenous image point (u,v,1)
		cv::Matx34d P,       //camera 1 matrix
		cv::Point3d u1,      //homogenous image point in 2nd camera
		cv::Matx34d P1       //camera 2 matrix
)
{
	//build matrix A for homogenous equation system Ax = 0
	//assume X = (x,y,z,1), for Linear-LS method
	//which turns it into a AX = B system, where A is 4x3, X is 3x1 and B is 4x1
	cv::Matx43d A(u.x*P(2,0)-P(0,0),    u.x*P(2,1)-P(0,1),      u.x*P(2,2)-P(0,2),
			u.y*P(2,0)-P(1,0),    u.y*P(2,1)-P(1,1),      u.y*P(2,2)-P(1,2),
			u1.x*P1(2,0)-P1(0,0), u1.x*P1(2,1)-P1(0,1),   u1.x*P1(2,2)-P1(0,2),
			u1.y*P1(2,0)-P1(1,0), u1.y*P1(2,1)-P1(1,1),   u1.y*P1(2,2)-P1(1,2)
	);
	cv::Mat_<double> B = (cv::Mat_<double>(4,1) <<    -(u.x*P(2,3)    -P(0,3)),
			-(u.y*P(2,3)  -P(1,3)),
			-(u1.x*P1(2,3)    -P1(0,3)),
			-(u1.y*P1(2,3)    -P1(1,3)));

	cv::Mat_<double> X;
	cv::solve(A,B,X, cv::DECOMP_SVD);

	return X;
}



 * triangulates 3d points and reprojects them giving an error
 * if false the point is invalid

bool VIO::triangulateAndCheck(cv::Point2f pt1, cv::Point2f pt2, cv::Matx33d K1, cv::Matx33d K2, VIOState x1_b, VIOState x2_b, double& error, cv::Matx31d& r, tf::Transform base2cam)
{

	VIOState x1_c, x2_c;

	Eigen::Quaterniond q1_b = Eigen::Quaterniond(x1_b.q1(), x1_b.q2(), x1_b.q3(), x1_b.q0());
	Eigen::Quaterniond q2_b = Eigen::Quaterniond(x2_b.q1(), x2_b.q2(), x2_b.q3(), x2_b.q0());

	tf::Quaternion temp_q = base2cam.getRotation();
	Eigen::Quaterniond q_b_c = Eigen::Quaterniond(temp_q.getX(), temp_q.getY(), temp_q.getZ(), temp_q.getW());

	base2cam * base2cam;

	Eigen::Quaterniond q1_c = q1_b * q_b_c;
	Eigen::Quaterniond q2_c = q2_b * q_b_c;

	//q1 * diff = q2 => inv(q1)* q2 = diff
	Eigen::Quaterniond diff = q1_c.inverse() * q2_c; // the relative rotation quaternion

	Eigen::Vector3d r1_b = Eigen::Vector3d(x1_b.x(), x1_b.y(), x1_b.z());
	Eigen::Vector3d r2_b = Eigen::Vector3d(x2_b.x(), x2_b.y(), x2_b.z());

	Eigen::Vector3d r_b_c = Eigen::Vector3d(base2cam.getOrigin().getX(), base2cam.getOrigin().getY(), base2cam.getOrigin().getZ());

	Eigen::Vector3d r1_c = (q1_b * r_b_c + r1_b);
	Eigen::Matrix<double, 3, 1> dr = (q2_b * r_b_c + r2_b) - r1_c;

	cv::Matx34d P1;
	cv::hconcat(cv::Mat::eye(3, 3, CV_64F), cv::Mat::zeros(3, 1, CV_64F), P1);

	//this creates a transformation from the first camera to the second
	Eigen::Matrix<double, 3, 3> R_ = diff.inverse().toRotationMatrix();
	Eigen::Matrix<double, 3, 1> t_ = R_ * -dr;

	cv::Mat R_cv, t_cv;
	cv::eigen2cv(R_, R_cv);
	cv::eigen2cv(t_, t_cv);

	cv::Matx34d P2;
	cv::hconcat(R_cv, t_cv, P2);

	//ROS_DEBUG_STREAM("P1: " << P1);
	//ROS_DEBUG_STREAM("P2: " << P2);
	//now P1 and P2 are made
	//results will be in camera 1 coordinate system

	//using tf to compute P2

	tf::Quaternion q1_b, q2_b;
	q1_b = tf::Quaternion(x1_b.q1(), x1_b.q2(), x1_b.q3(), x1_b.q0());
	q2_b = tf::Quaternion(x2_b.q1(), x2_b.q2(), x2_b.q3(), x2_b.q0());

	tf::Vector3 r1_b, r2_b;
	r1_b = tf::Vector3(x1_b.x(), x1_b.y(), x1_b.z());
	r2_b = tf::Vector3(x2_b.x(), x2_b.y(), x2_b.z());

	tf::Transform t1_b, t2_b;
	t1_b = tf::Transform(q1_b, r1_b);
	t2_b = tf::Transform(q2_b, r2_b);

	tf::Transform t1_c, t2_c;
	t1_c = t1_b * base2cam;
	t2_c = t2_b * base2cam;

	tf::Quaternion q1_c, q2_c;
	q1_c = t1_c.getRotation();
	q2_c = t2_c.getRotation();

	//q1 * diff = q2 => diff = q1.inv * q2

	tf::Transform temp = tf::Transform(q1_c.inverse() * q2_c, t2_c.getOrigin() - t1_c.getOrigin());
	tf::Transform tf_p2 = temp.inverse();

	cv::Matx34d P2;
	P2(0, 0) = tf_p2.getBasis().getRow(0).x();
	P2(0, 1) = tf_p2.getBasis().getRow(0).y();
	P2(0, 2) = tf_p2.getBasis().getRow(0).z();
	P2(1, 0) = tf_p2.getBasis().getRow(1).x();
	P2(1, 1) = tf_p2.getBasis().getRow(1).y();
	P2(1, 2) = tf_p2.getBasis().getRow(1).z();
	P2(2, 0) = tf_p2.getBasis().getRow(2).x();
	P2(2, 1) = tf_p2.getBasis().getRow(2).y();
	P2(2, 2) = tf_p2.getBasis().getRow(2).z();

	P2(0, 3) = tf_p2.getOrigin().x();
	P2(1, 3) = tf_p2.getOrigin().y();
	P2(2, 3) = tf_p2.getOrigin().z();

	//break if not moved enough

	double d = sqrt(P2.col(3).dot(P2.col(3)));
	if(d < MIN_TRIANGUALTION_DIST){
		ROS_DEBUG_STREAM("d too small: " << d);
		return false;
	}

	//ROS_DEBUG_STREAM(P2);

	cv::Matx34d P1;
	cv::hconcat(cv::Mat::eye(3, 3, CV_64F), cv::Mat::zeros(3, 1, CV_64F), P1);


	cv::Matx41d X;
	cv::Matx61d b;
	cv::Mat_<double> A = cv::Mat_<double>(6, 4);

	b(0) = pt1.x;
	b(1) = pt1.y;
	b(2) = 1.0;
	b(3) = pt2.x;
	b(4) = pt2.y;
	b(5) = 1.0;

	cv::vconcat(K1 * P1, K2 * P2, A);

	//ROS_DEBUG_STREAM("K1: " << K1);
	//ROS_DEBUG_STREAM("K2: " << K2);
	//ROS_DEBUG_STREAM("A: " << A);
	//ROS_DEBUG_STREAM("b: " << b);

	//now we can triangulate

	cv::solve(A, b, X, cv::DECOMP_SVD);

	r(0) = X(0) / X(3);
	r(1) = X(1) / X(3);
	r(2) = X(2) / X(3);

	X(0) = r(0);
	X(1) = r(1);
	X(2) = r(2);
	X(3) = r(3);

	//reproject

	cv::Matx31d b1 = K1 * P1 * X;
	b1(0) = b1(0) / b1(2);
	b1(1) = b1(1) / b1(2);
	b1(2) = 1.0;

	cv::Matx31d b2 = K2 * P2 * X;
	b2(0) = b2(0) / b2(2);
	b2(1) = b2(1) / b2(2);
	b2(2) = 1.0;

	cv::Matx61d b_;
	cv::vconcat(b1, b2, b_);

	error = (b_ - b).dot(b_ - b);

	if(r(2) > MIN_TRIAG_Z && error < MAX_TRIAG_ERROR)
	{
		ROS_DEBUG_STREAM("r: " << r);
		ROS_DEBUG_STREAM("error: " << error);
		ROS_DEBUG_STREAM("pt: " << pt2);
		//ROS_DEBUG_STREAM("du: " << (b_ - b).t());
		//transform the point into world coordinates

		tf::Vector3 tf_r = tf::Vector3(r(0), r(1), r(2));

		tf::Vector3 tf_r_w = t1_c.inverse() * tf_r;

		r(0) = tf_r_w.getX();
		r(1) = tf_r_w.getY();
		r(2) = tf_r_w.getZ();

		ROS_DEBUG_STREAM("r world: " << r);
		return true;
	}
	else
	{
		return false;
	}
}


 * this function uses the oldest state possible and the current state
 * along with the previous frame and current frame to
 * update each 3d feature and add new 3d features if necessary
 * If 3d feature is not updated it will be either removed or added to the inactive list.

void VIO::update3DFeatures(VIOState x, VIOState x_last, Frame cf, Frame lf, std::deque<Frame> fb)
{
	std::vector<VIOFeature3D> inactives = this->active3DFeatures; // set the new inactive features to be the current active features
	std::vector<VIOFeature3D> actives;

	tf::StampedTransform base2cam;
	try{
		this->ekf.tf_listener.lookupTransform(this->camera_frame, this->CoM_frame, ros::Time(0), base2cam);
	}
	catch(tf::TransformException e){
		ROS_WARN_STREAM(e.what());
	}


	for(int i = 0; i < cf.features.size(); i++)
	{
		if(cf.features.at(i).isMatched()) // if this feature is matched
		{

			// store the currentFeature and lastFeature
			VIOFeature2D current2DFeature = cf.features.at(i);
			int frame_index = 0;
			VIOFeature2D last2DFeature;
			this->findBestCorresponding2DFeature(current2DFeature, lf, fb, last2DFeature, frame_index); // finds the best corresponding 2d feature

			//set up each of the relavant matrices depending on the frame which the oldest
			// corresponding feature lies in
			cv::Matx33d K1;
			cv::Matx33d K2 = currentFrame.K;
			if(frame_index == -1)
			{
				K1 = this->lastFrame.K;
			}
			else
			{
				K1 = fb.at(frame_index).K;
			}

			//ROS_DEBUG_STREAM(frame_index);
			//ROS_DEBUG_STREAM("P1: " << P1.col(3) << "\nP2: " << P2.col(3));

			//check if this feature has a matched 3d feature
			bool match3D = false;
			VIOFeature3D matched3DFeature;
			for(int j = 0; j < inactives.size(); j++)
			{
				if(inactives.at(j).current2DFeatureMatchIndex == current2DFeature.getMatchedIndex())
				{
					matched3DFeature = inactives.at(j); // found a matched feature

					ROS_ASSERT(matched3DFeature.current2DFeatureMatchID == lf.features.at(current2DFeature.getMatchedIndex()).getFeatureID()); // ensure that everything matches up

					match3D = true; //set the flag for later

					inactives.erase(inactives.begin() + j); // erase the jth feature from inactives

					break;
				}
			}

			//TRIANGULATION

			VIOState x1, x2;
			x2 = this->state;
			if(frame_index == -1)
				x1 = this->lastState;
			else
				x1 = fb.at(frame_index).state;
			//ROS_DEBUG_STREAM("x1: " << x1.vector.transpose());
			//ROS_DEBUG_STREAM("x2: " << x2.vector.transpose());
			//ROS_DEBUG_STREAM("pt1: " << last2DFeature.getUndistorted());
			//ROS_DEBUG_STREAM("pt2: " << current2DFeature.getUndistorted() << "\n");



			cv::Matx31d r; //resulting point
			double error; //error in pixels

			bool successful = this->triangulateAndCheck(last2DFeature.getUndistorted(), current2DFeature.getUndistorted(), K1, K2, x1, x2, error, r, base2cam); // triangulate the points if possible

			if(successful)
			{
				double d = (x2.getr() - x1.getr()).norm();
				double r_cov_sum = (x1.covariance(0, 0) + x1.covariance(1, 1) + x1.covariance(2, 2) + x2.covariance(0, 0) + x2.covariance(1, 1) + x2.covariance(1, 1));

				ROS_DEBUG_STREAM("frame used: " << frame_index);
				ROS_DEBUG_STREAM("dist traveled: " << d);
				ROS_DEBUG_STREAM("r_cov_sum: " << r_cov_sum);

				if(match3D)
				{


					matched3DFeature.update(Eigen::Vector3d(r(0), r(1), r(2)), error + (1/d) + r_cov_sum);
					matched3DFeature.current2DFeatureMatchID = current2DFeature.getFeatureID();
					matched3DFeature.current2DFeatureMatchIndex = i;

					ROS_DEBUG_STREAM("updating feature new cov: " << matched3DFeature.variance);
					ROS_DEBUG_STREAM("after pos: " << matched3DFeature.position);

					actives.push_back(matched3DFeature);
				}
				else
				{


					ROS_DEBUG_STREAM("adding feature");
					VIOFeature3D newFeat;
					newFeat.color = cv::Scalar(255, 255, 255);
					newFeat.current2DFeatureMatchID = current2DFeature.getFeatureID();
					newFeat.current2DFeatureMatchIndex = i;
					newFeat.position = Eigen::Vector3d(r(0), r(1), r(2));
					newFeat.variance = 10000; // starting cov
					newFeat.colorSet = true;

					actives.push_back(newFeat);
				}
			}
			else if(match3D)
			{
				matched3DFeature.current2DFeatureMatchID = current2DFeature.getFeatureID();
				matched3DFeature.current2DFeatureMatchIndex = i;
				actives.push_back(matched3DFeature);
			}
		}
	}

	//set each of the 3d feature buffers to be published
	this->active3DFeatures = actives;
	this->inactive3DFeatures = inactives;
}

void VIO::findBestCorresponding2DFeature(VIOFeature2D start, Frame lf, std::deque<Frame> fb, VIOFeature2D& end, int& frameIndex)
{
	if(!start.isMatched()) ROS_ERROR("feature has no match!");


	VIOFeature2D temp = lf.features.at(start.getMatchedIndex());
	ROS_ASSERT(temp.getFeatureID() == start.getMatchedID());
	end = temp;

	//ROS_DEBUG_STREAM("fb size: " << fb.size());

	if(end.isMatched() && fb.size() > 0)
	{
		for(int i = 0; i < fb.size(); i++)
		{
			temp = fb.at(i).features.at(end.getMatchedIndex());
			ROS_ASSERT(temp.getFeatureID() == end.getMatchedID());
			end = temp;

			//ROS_DEBUG_STREAM("end match " << end.getMatchedID());
			//ROS_DEBUG_STREAM("this frame: " << fb.at(i).nextFeatureID);
			//ROS_DEBUG_STREAM("next frame: " << fb.at(i+1).nextFeatureID);

			if(!end.isMatched())
			{
				frameIndex = i;
				break;
			}
			frameIndex = i;
		}
	}
	else
	{
		frameIndex = -1;
	}

	//ROS_DEBUG_STREAM("frame index: " << frameIndex);
}



 */
