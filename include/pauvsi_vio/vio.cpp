/*
 * vio.cpp
 *
 *  Created on: Sep 19, 2016
 *      Author: kevinsheridan
 */

#include "vio.h"

VIO::VIO()
{

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
	}
	cv::imshow("test", img);
	cv::waitKey(30);
}

/*
 * draws frame with its features
 */
void VIO::viewImage(Frame frame){
	cv::Mat img;
	cv::drawKeypoints(frame.image, frame.getKeyPointVectorFromFeatures(), img, cv::Scalar(255, 0, 0));
	this->viewImage(img, true);

}

/*
 * corrects the drift in position
 */
void VIO::correctPosition(std::vector<double> pos)
{
	position = pos;
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
	if(!lastFrame.isFrameSet()){
		currentFrame.getFASTCorners(this->FAST_THRESHOLD);
		currentFrame.describeFeaturesWithBRIEF();
		currentFrame.cleanUpFeaturesByKillRadius(this->KILL_RADIUS); // cleans features by killing them if their radius to too large
	}

	// if there is a last frame, flow features and estimate motion
	if(lastFrame.isFrameSet())
	{
		ROS_DEBUG_ONCE("starting optical flow");
		this->flowFeaturesToNewFrame(lastFrame, currentFrame);
		currentFrame.cleanUpFeaturesByKillRadius(this->KILL_RADIUS); // clean features by killing ones with a large radius from center

		//estimate motion
		this->estimateMotion(lastFrame, currentFrame);
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

	ros::param::param<int>("~fast_threshold", FAST_THRESHOLD, DEFAULT_FAST_THRESHOLD);

	ros::param::param<float>("~feature_kill_radius", KILL_RADIUS, DEFAULT_2D_KILL_RADIUS);

	ros::param::param<int>("~feature_similarity_threshold", FEATURE_SIMILARITY_THRESHOLD, DEFAULT_FEATURE_SIMILARITY_THRESHOLD);
	ros::param::param<bool>("~kill_by_dissimilarity", KILL_BY_DISSIMILARITY, false);

	ros::param::param<float>("~min_eigen_value", MIN_EIGEN_VALUE, DEFAULT_MIN_EIGEN_VALUE);
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

	ROS_DEBUG_STREAM_COND(lostFeatures, "optical flow lost " << lostFeatures <<  " feature(s)");

	//if user wants to kill by similarity
	if(KILL_BY_DISSIMILARITY)
	{
		ROS_DEBUG("killing by similarity");
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
 * uses epipolar geometry from two frames to
 * estimate relative motion of the frame;
 */
bool VIO::estimateMotion(Frame frame1, Frame frame2)
{
	//first get the feature deltas from the two frames
	std::vector<cv::Point2f> prevPoints, currentPoints;
	this->getCorrespondingPointsFromFrames(frame1, frame2, prevPoints, currentPoints);

	//calculate the essential matrix
	cv::Mat essentialMatrix = cv::findEssentialMat(prevPoints, currentPoints, this->K);

	//undistort points using fisheye model
	//cv::fisheye::undistortPoints(prevPoints, prevPoints, this->K, this->D);
	//cv::fisheye::undistortPoints(currentPoints, currentPoints, this->K, this->D);

	//recover pose change from essential matrix
	cv::Mat translation;
	cv::Mat rotation1;
	cv::Mat rotation2;
	cv::decomposeEssentialMat(essentialMatrix, rotation1, rotation2, translation);

	ROS_DEBUG_STREAM("translation: " << translation.t());

	return true;
}

