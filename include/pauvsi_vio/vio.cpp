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

	//set up image transport
	image_transport::ImageTransport it(nh);
	this->cameraSub = it.subscribeCamera(this->getCameraTopic(), 1, &VIO::cameraCallback, this);

	//setup imu sub
	this->imuSub = nh.subscribe(this->getIMUTopic(), 100, &VIO::imuCallback, this);

	this->pose = geometry_msgs::PoseStamped();
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

	/* sets the current frame and its time created
	 * It also runs a series of functions which ultimately estimate
	 * the motion of the camera
	 */
	this->setCurrentFrame(temp, cv_bridge::toCvCopy(img, "mono8")->header.stamp);

	ROS_DEBUG_STREAM_THROTTLE(0.5, (ros::Time::now().toSec() - start.toSec()) * 1000 << " milliseconds runtime");

	this->viewImage(this->getCurrentFrame());

	//ros::Duration d = ros::Duration(0.1);
	//d.sleep();
}

void VIO::imuCallback(const sensor_msgs::ImuConstPtr& msg)
{
	//ROS_DEBUG_STREAM_THROTTLE(0.1, "accel: " << msg->linear_acceleration);
	this->addIMUReading(*msg);
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
 * shows cv::Mat
 */
void VIO::viewImage(cv::Mat img, bool rectify){
	if(rectify)
	{
		cv::Matx33d newK = K;
		newK(0, 0) = 100;
		newK(1, 1) = 100;
		cv::fisheye::undistortImage(img, img, this->K, this->D, newK);
		//ROS_DEBUG_STREAM(newK);
	}
	cv::imshow("test", img);
	cv::waitKey(30);
}

/*
 * draws frame with its features
 */
void VIO::viewImage(Frame frame){
	cv::Mat img;
	cv::drawKeypoints(frame.image, frame.getKeyPointVectorFromFeatures(), img, cv::Scalar(0, 0, 255));
	this->viewImage(img, false);

}

/*
 * sets the current frame and computes important
 * info about it
 * finds corners
 * describes corners
 */
void VIO::setCurrentFrame(cv::Mat img, ros::Time t)
{
	if(currentFrame.isFrameSet())
	{
		//first set the last frame to current frame
		lastFrame = currentFrame;
	}

	currentFrame = Frame(img, t, lastFrame.nextFeatureID); // create a frame with a starting ID of the last frame's next id

	this->run();
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
	if(lastFrame.isFrameSet())
	{
		if(lastFrame.features.size() > 0)
		{
			this->flowFeaturesToNewFrame(lastFrame, currentFrame);
			currentFrame.cleanUpFeaturesByKillRadius(this->KILL_RADIUS);
			//this->checkFeatureConsistency(currentFrame, this->FEATURE_SIMILARITY_THRESHOLD);
		}

		//MOTION ESTIMATION

		double certainty = this->estimateMotion();
	}

	//check the number of 2d features in the current frame
	//if this is below the required amount refill the feature vector with
	//the best new feature. It must not be redundant.

	//ROS_DEBUG_STREAM("feature count: " << currentFrame.features.size());

	if(currentFrame.features.size() < this->NUM_FEATURES)
	{
		//add n new unique features
		//ROS_DEBUG("low on features getting more");
		currentFrame.getAndAddNewFeatures(this->NUM_FEATURES - currentFrame.features.size(), this->FAST_THRESHOLD, this->KILL_RADIUS, this->MIN_NEW_FEATURE_DISTANCE);
		//currentFrame.describeFeaturesWithBRIEF();
	}

	ROS_DEBUG_STREAM("imu readings: " << this->imuMessageBuffer.size());
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

	ros::param::param<int>("~fast_threshold", FAST_THRESHOLD, DEFAULT_FAST_THRESHOLD);

	ros::param::param<float>("~feature_kill_radius", KILL_RADIUS, DEFAULT_2D_KILL_RADIUS);

	ros::param::param<int>("~feature_similarity_threshold", FEATURE_SIMILARITY_THRESHOLD, DEFAULT_FEATURE_SIMILARITY_THRESHOLD);
	ros::param::param<bool>("~kill_by_dissimilarity", KILL_BY_DISSIMILARITY, false);

	ros::param::param<float>("~min_eigen_value", MIN_EIGEN_VALUE, DEFAULT_MIN_EIGEN_VALUE);

	ros::param::param<int>("~num_features", NUM_FEATURES, DEFAULT_NUM_FEATURES);

	ros::param::param<int>("~min_new_feature_distance", MIN_NEW_FEATURE_DISTANCE, DEFAULT_MIN_NEW_FEATURE_DIST);
}


/*
 * This will match feature descriptors between two images
 *
 * In the first variant of this method, the train descriptors are passed as an input argument. In the
 * second variant of the method, train descriptors collection that was set by DescriptorMatcher::add is
 * used. Optional mask (or masks) can be passed to specify which query and training descriptors can be
 * matched. Namely, queryDescriptors[i] can be matched with trainDescriptors[j] only if
 * mask.at\<uchar\>(i,j) is non-zero.
 */
std::vector<cv::DMatch> VIO::matchFeaturesWithFlann(cv::Mat query, cv::Mat train){
	std::vector<cv::DMatch> matches;
	cv::FlannBasedMatcher matcher(new cv::flann::LshIndexParams(20, 10, 2));
	matcher.match(query, train, matches);

	ROS_DEBUG_STREAM_THROTTLE(2, "query size: " << query.rows << " train size: " << train.rows << " matches size: " << matches.size());

	return matches;
}

/*
 * uses optical flow to find a vector of features in another image
 * This function does not require a prediction
 * This will set the feature vector within the new frame with the
 * flowed points
 */
bool VIO::flowFeaturesToNewFrame(Frame& oldFrame, Frame& newFrame){

	std::vector<cv::Point2f> oldPoints = oldFrame.getPoint2fVectorFromFeatures();
	ROS_DEBUG_STREAM_ONCE("got " << oldPoints.size() << " old point2fs from the oldframe which has " << oldFrame.features.size() << " features");
	std::vector<cv::Point2f> newPoints;

	std::vector<uchar> status; // status vector for each point
	cv::Mat error; // error vector for each point

	ROS_DEBUG_ONCE("running lucas kande optical flow algorithm");
	/*
	 * this calculates the new positions of the old features in the new image
	 * status tells us whether or not a point index has been flowed over to the new frame
	 * last value is a minimum eigen value thresh
	 * it will kill bad features
	 */
	cv::calcOpticalFlowPyrLK(oldFrame.image, newFrame.image, oldPoints, newPoints, status, error, cv::Size(21, 21), 3,
			cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01), 0, this->MIN_EIGEN_VALUE);

	ROS_DEBUG_STREAM_ONCE("ran optical flow and got " << newPoints.size() << " points out");

	int lostFeatures = 0;
	//next add these features into the new Frame
	for (int i = 0; i < newPoints.size(); i++)
	{
		//check if the point was able to flow
		if(status.at(i) == 1)
		{
			// the id number is not that important because it will be handled by the frame
			VIOFeature2D feat(newPoints.at(i), oldFrame.features.at(i).getFeatureID(), i, -1); // create a matched feature with id = -1
			//if the previous feature was described
			if(oldFrame.features.at(i).isFeatureDescribed())
			{
				//ROS_DEBUG("transferring feature description");
				feat.setFeatureDescription(oldFrame.features.at(i).getFeatureDescription()); // transfer previous description to new feature
				//ROS_DEBUG_STREAM_THROTTLE(0, feat.getFeatureDescription());
			}

			newFrame.addFeature(feat); // add this feature to the new frame
		}
		else
		{
			lostFeatures++;
		}
	}

	//ROS_DEBUG_STREAM_COND(lostFeatures, "optical flow lost " << lostFeatures <<  " feature(s)");

	//if user wants to kill by similarity
	if(KILL_BY_DISSIMILARITY)
	{
		//ROS_DEBUG("killing by similarity");
		this->checkFeatureConsistency(newFrame, this->FEATURE_SIMILARITY_THRESHOLD);
	}

	return true;
}

/*
 * gets corresponding points between the two frames as two vectors of point2f
 * checks if index and id match for saftey
 */
void VIO::getCorrespondingPointsFromFrames(Frame lastFrame, Frame currentFrame, std::vector<cv::Point2f>& lastPoints, std::vector<cv::Point2f>& currentPoints)
{

	for (int i = 0; i < currentFrame.features.size(); i++)
	{
		if(currentFrame.features.at(i).isMatched() &&
				lastFrame.features.at(currentFrame.features.at(i).getMatchedIndex()).getFeatureID() ==
						currentFrame.features.at(i).getMatchedID()){
			lastPoints.push_back(lastFrame.features.at(currentFrame.features.at(i).getMatchedIndex()).getFeature().pt);
			currentPoints.push_back(currentFrame.features.at(i).getFeature().pt);
		}
		else
		{
			ROS_WARN("could not match feature id to index");
		}
	}
}

/*
 * checks to see if current descriptor is similar to actual feature
 * if similarity is bellow threshold, feature is kept and descriptor is updated
 * otherwise feature is removed from feature vector
 */
void VIO::checkFeatureConsistency(Frame& checkFrame, int killThreshold ){
	cv::Mat newDescription = checkFrame.describeFeaturesWithBRIEF(checkFrame.image, checkFrame.features);

	std::vector<VIOFeature2D> tempFeatures;

	for (int i = 0; i < checkFrame.features.size(); i++){

		if(!checkFrame.features.at(i).isFeatureDescribed())
			break;

		cv::Mat row = newDescription.row(i);

		//ROS_DEBUG_STREAM_ONCE("got feature description " << row);

		int x = checkFrame.compareDescriptors(row, checkFrame.features.at(i).getFeatureDescription());
		//int x = checkFrame.compareDescriptors(row, row);

		if (x <= killThreshold){

			//ROS_DEBUG_STREAM("features match " << i <<" : "<<checkFrame.features.size()<<" : "<< newDescription.rows <<" : " << x);
			//ROS_DEBUG_STREAM("i+1: "<< checkFrame.features.at(i+1).getFeatureDescription()<<":"<<checkFrame.features.at(i+1).isFeatureDescribed());
			//ROS_DEBUG_STREAM("description size " << checkFrame.features.at(i).getFeatureDescription().cols);

			checkFrame.features.at(i).setFeatureDescription(row);

			//ROS_DEBUG("modified feature");

			tempFeatures.push_back(checkFrame.features.at(i));

			//ROS_DEBUG("pushed back modified feature");
		}
		else{
			ROS_DEBUG("feature does'nt match enough, killing");
		}
	}

	//ROS_DEBUG("setting new features");
	checkFrame.features = tempFeatures;
	//ROS_DEBUG("set new features");
}

/*
 * find the average change in position
 * for all feature correspondences
 * vectors must be same sizes
 */
double VIO::averageFeatureChange(std::vector<cv::Point2f> points1, std::vector<cv::Point2f> points2)
{
	double diff = 0;
	double dx, dy;
	for(int i = 0; i < points1.size(); i++)
	{
		dx = (double)(points1.at(i).x - points2.at(i).x);
		dy = (double)(points1.at(i).y - points2.at(i).y);
		diff += sqrt(pow(dx, 2) + pow(dy, 2));
	}

	return diff / (double)points1.size();
}

void VIO::broadcastWorldToOdomTF()
{
	static tf::TransformBroadcaster br;
	tf::Transform transform;
	transform.setOrigin(tf::Vector3(this->pose.pose.position.x, this->pose.pose.position.y, this->pose.pose.position.z));
	tf::Quaternion q;
	q.setValue(this->pose.pose.orientation.x, this->pose.pose.orientation.y,
			this->pose.pose.orientation.z, this->pose.pose.orientation.w);
	transform.setRotation(q);
	br.sendTransform(tf::StampedTransform(transform, this->pose.header.stamp, this->world_frame, this->odom_frame));
}

/*
 * returns the certainty
 * predicts the new rotation and position of the camera.
 * transfroms it to the odometry frame
 * and publishes a pose estimate
 */
double VIO::estimateMotion()
{
	std::vector<double> inertialAngleChange, inertialPositionChange; // change in angle and pos from imu
	std::vector<double> visualAngleChange, visualPositionChange;
	double visualMotionCertainty;
	double averageMovement;

	//infer motion from images
	std::vector<double> unitVelocityInference;
	bool visualMotionInferenceSuccessful = false;

	//get motion inference from visual odometry
	visualMotionInferenceSuccessful = this->visualMotionInference(lastFrame, currentFrame, inertialAngleChange,
			visualAngleChange, unitVelocityInference, averageMovement);

}

/*
 * uses epipolar geometry from two frames to
 * estimate relative motion of the frame;
 */
bool VIO::visualMotionInference(Frame frame1, Frame frame2, std::vector<double> angleChangePrediction, std::vector<double>& rotationInference, std::vector<double>& unitVelocityInference, double& averageMovement)
{
	//first get the feature deltas from the two frames
	std::vector<cv::Point2f> prevPoints, currentPoints;
	this->getCorrespondingPointsFromFrames(frame1, frame2, prevPoints, currentPoints);

	//undistort points using fisheye model
	cv::fisheye::undistortPoints(prevPoints, prevPoints, this->K, this->D);
	cv::fisheye::undistortPoints(currentPoints, currentPoints, this->K, this->D);

	//get average movement bewteen images
	averageMovement = this->averageFeatureChange(prevPoints, currentPoints);

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
	cv::Mat rotation1;
	cv::Mat rotation2;

	//decompose matrix to get possible deltas
	cv::decomposeEssentialMat(essentialMatrix, rotation1, rotation2, translation);

	cv::Mat mtxR, mtxQ;
	cv::Vec3d angle1 = cv::RQDecomp3x3(rotation1, mtxR, mtxQ);
	cv::Mat mtxR2, mtxQ2;
	cv::Vec3d angle2 = cv::RQDecomp3x3(rotation2, mtxR2, mtxQ2);

	return true;
}

/*
 * this gets the inertial motion estimate from the imu buffer to the specified time
 * this is gotten from the fromPose and fromVelocity at their times.
 * the results are output in the angle, pos, vel change vectors
 * it returns the number of IMU readings used
 *
 * angle change is in radians RPY
 */
int VIO::getInertialMotionEstimate(ros::Time fromTime, ros::Time toTime, std::vector<double> fromVelocity,
		std::vector<double> fromAngularVelocity, std::vector<double>& angleChange,
			std::vector<double>& positionChange, std::vector<double>& velocityChange)
{
	//check if there are any imu readings
	if(this->imuMessageBuffer.size() == 0)
	{
		return 0;
	}

	int startingIMUIndex, endingIMUIndex;

	/* first find the IMU reading index such that the toTime is greater than
	 * the stamp of the IMU reading.
	 */
	for(int i = this->imuMessageBuffer.size() - 1; i >= 0; i--)
	{
		if(this->imuMessageBuffer.at(i).header.stamp.toSec() < toTime.toSec())
		{
			endingIMUIndex = i;
			break;
		}
	}

	/*
	 * second find the IMU reading index such that the fromTime is lesser than the
	 * stamp of the IMU message
	 */
	for(int i = 0; i < this->imuMessageBuffer.size(); i++)
	{
		if(this->imuMessageBuffer.at(i).header.stamp.toSec() > fromTime.toSec())
		{
			startingIMUIndex = i;
			break;
		}
	}

	//now we have all IMU readings between the two times.

	/*
	 * third create the corrected accelerations vector
	 * 1) remove centripetal acceleration
	 * 2) remove gravity
	 */




}

