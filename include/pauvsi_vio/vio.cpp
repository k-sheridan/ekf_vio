/*
 * vio.cpp
 *
 *  Created on: Sep 19, 2016
 *      Author: kevinsheridan
 */

#include "vio.h"

VIO::VIO()
{
	this->lastFrameSet = false;
	this->currentFrameSet = false;
}

/*
 * shows cv::Mat
 */
void VIO::viewImage(cv::Mat img){
	cv::imshow("test", img);
	cv::waitKey(30);
}

/*
 * draws frame with its features
 */
void VIO::viewImage(cv::Mat img, std::vector<cv::KeyPoint> keypoints){
	cv::drawKeypoints(img, keypoints, img);
	this->viewImage(img);
}

/*
 * draws the matches between two frames
 */
void VIO::viewImage(Frame frame1, Frame frame2, std::vector<cv::DMatch> matches){
	cv::Mat img;
	cv::drawMatches(frame1.image, frame1.corners, frame2.image, frame2.corners, matches, img);
	this->viewImage(img);
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
void VIO::setCurrentFrame(cv::Mat frame, ros::Time t)
{
	if(currentFrameSet)
	{
		//first set the last frame to current frame
		lastFrame = currentFrame;
		lastFrameSet = true; // the last frame has been set
	}

	currentFrame = Frame(frame, t);
	currentFrame.corners = this->computeFASTFeatures(currentFrame.image, this->fastThreshold); // get frame's features
	currentFrame.descriptors = this->extractBRIEFDescriptors(currentFrame.image, currentFrame.corners); //describes frame's features
	currentFrameSet = true;

	// if there is a last frame to process off of and it has descriptors
	if(this->isLastFrameSet())
	{
		//match the new last frame to the new current frame
		this->matchesFromLastToCurrentFrame = this->matchFeaturesWithFlann(lastFrame.descriptors, currentFrame.descriptors);
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

	ros::param::param<int>("~fast_threshold", fastThreshold, DEFAULT_FAST_THRESHOLD);
}

/*
 * finds the features within an image
 */
std::vector<cv::KeyPoint> VIO::computeFASTFeatures(cv::Mat img, int threshold){
	std::vector<cv::KeyPoint> corners;
	cv::FAST(img, corners, threshold, true); // detect with nonmax suppression
	return corners;
}

/*
 * uses the FREAK algorithm to extract feature descriptors
 */
cv::Mat VIO::extractFREAKDescriptors(cv::Mat img, std::vector<cv::KeyPoint> corners){
	cv::Ptr<cv::xfeatures2d::FREAK> extractor = cv::xfeatures2d::FREAK::create();
	cv::Mat descriptors;
	extractor->compute(img, corners, descriptors);
	//ROS_DEBUG_STREAM_THROTTLE(2, "descriptor size: " << descriptors.cols << " X " << descriptors.rows);
	return descriptors;
}

/*
 * uses the BRIEF algorithm to extract feature descriptors
 * This is the faster than FREAK
 */
cv::Mat VIO::extractBRIEFDescriptors(cv::Mat img, std::vector<cv::KeyPoint> corners){
	cv::Ptr<cv::xfeatures2d::BriefDescriptorExtractor> extractor = cv::xfeatures2d::BriefDescriptorExtractor::create();
	cv::Mat descriptors;
	extractor->compute(img, corners, descriptors);
	ROS_DEBUG_STREAM_THROTTLE(2, "descriptor size: " << descriptors.cols << " X " << descriptors.rows);
	return descriptors;
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
 */
std::vector<cv::KeyPoint> VIO::findFeaturesInNewImage(Frame oldFrame, cv::Mat newImage){
	cv::KeyPoint pointConverter;
	std::vector<cv::Point2f> newPoints;
	std::vector<cv::Point2f> oldPoints;
	pointConverter.convert(oldFrame.corners, oldPoints); // convert the Keypoints to point2fs

	std::vector<unsigned char> status; // status vector for each point
	std::vector<int> error; // error vector for each point

	cv::calcOpticalFlowPyrLK(oldFrame.image, newImage, oldPoints, newPoints, status, error);
}



