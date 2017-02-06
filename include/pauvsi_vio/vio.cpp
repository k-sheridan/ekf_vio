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
	{
		featurePub = nh.advertise<sensor_msgs::PointCloud>("/vio/features", 100);
	}
	initialized = false; //not intialized yet

	//push two frames into the fb
	this->frameBuffer.push_front(Frame());
	this->frameBuffer.push_front(Frame());

	//ensure that both frames have a valid state
	this->currentFrame().state = this->state;
	this->lastFrame().state = this->state;
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
		//TODO remove any reference
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
			ROS_DEBUG_STREAM("current frame address: " << &frameBuffer.at(0));
			ROS_DEBUG_STREAM("last frame address: " << &frameBuffer.at(1));
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
	}

	//check the number of 2d features in the current frame
	//if this is below the required amount refill the feature vector with
	//the best new feature. It must not be redundant.

	//ROS_DEBUG_STREAM("feature count: " << currentFrame.features.size());

	if(currentFrame().features.size() < this->NUM_FEATURES)
	{
		//add n new unique features
		ROS_DEBUG_STREAM("low on features getting more: " << currentFrame().features.size());
		int featuresAdded = currentFrame().getAndAddNewFeatures(this->NUM_FEATURES - currentFrame().features.size(), this->FAST_THRESHOLD, this->KILL_RADIUS, this->MIN_NEW_FEATURE_DISTANCE);
		ROS_DEBUG_STREAM("got more: " << currentFrame().features.size());
		//TODO check that everything is linked correctly
		//this block adds a map point for the new feature added and links it to the new feature
		for(std::vector<Feature>::iterator it = currentFrame().features.end() - featuresAdded; it != currentFrame().features.end(); it++)
		{
			feature_tracker.map.push_back(Point(&(*it))); // add a new map point linking it to the feature and therefore the frame
			it->point = &feature_tracker.map.back(); // link the feature to the point and therefore all other matches
			//ROS_DEBUG_STREAM("point map pointer: " << &feature_tracker.map.back() << " feature point: " << it->point);
		}

		//currentFrame.describeFeaturesWithBRIEF();

		currentFrame().undistortFeatures(); // undistort the new features
	}

	this->broadcastWorldToOdomTF();

	//ROS_DEBUG_STREAM("imu readings: " << this->imuMessageBuffer.size());
	ROS_DEBUG_STREAM("frame: " << this->frameBuffer.size() << " init: " << initialized);
	ROS_DEBUG_STREAM("map points: " << this->feature_tracker.map.size());
	ROS_DEBUG_STREAM(currentFrame().features.size());
}


/*
 * recalibrates the state using average pixel motion
 * uses an Extended Kalman Filter to predict and update the state and its
 * covariance.
 */
VIOState VIO::estimateMotion(VIOState x, Frame& lf, Frame& cf)
{
	//RECALIBRATION
	static bool consecutiveRecalibration = false;
	//ROS_DEBUG_STREAM_COND(cf.features.size() && cf.features.at(0).point->observations.size() ," test ##### " << cf.features.at(0).point->observations.back()->frame);
	double avgFeatureChange = feature_tracker.averageFeatureChange(lf, cf); // get the feature change between f1 and f2

	ROS_DEBUG_STREAM("avg pixel change: " << avgFeatureChange);

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


		//TODO run the ekf update method on the predicted state using either gausss newton estimate or the fundamental + predict mag estimate


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

#if SUPER_DEBUG
	this->updateKeyFrameInfo();
	this->drawKeyFrames();
#endif

	return newX;
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

	//state.setVelocity(normalize * state.getVelocity());
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

