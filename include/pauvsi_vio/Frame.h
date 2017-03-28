/*
 * frame.h
 *
 *  Created on: Sep 20, 2016
 *      Author: kevinsheridan
 */

#ifndef PAUVSI_M7_INCLUDE_PAUVSI_VO_FRAME_H_
#define PAUVSI_M7_INCLUDE_PAUVSI_VO_FRAME_H_

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/video.hpp"
#include <vector>
#include <algorithm>
#include <string>
#include <ros/ros.h>

#include "VIOState.hpp"

#include "Point.h"

#define DEFAULT_FEATURE_SEARCH_RANGE 5
#define MAXIMUM_ID_NUM 1000000000 //this is the maximum size that a feature ID should be to ensure there are no overflow issues.
#define DEFAULT_SCENE_DEPTH_LOCAL 0.5

class Feature;

class KeyFrame;

class Frame
{

private:
	cv::Ptr<cv::xfeatures2d::BriefDescriptorExtractor> descriptionExtractor;
	bool frameSet;

public:
	ros::Time timeImageCreated;

	cv::Mat image;

	cv::Mat K;

	cv::Mat D;

	std::vector<Feature> features; //the feature vector for this frame

	VIOState state;

	double avgSceneDepth;

	Eigen::Affine3d transform_frame_to_world; // this is a transform which maps a point in the world coordinate frame to a coordinate in the camera frame

	bool finalFrame;
	bool isKeyframe;

	//this ensures that all features have a unique ID
	int nextFeatureID; // the id of the next feature that is added to this frame or the next frame

	bool operator==(const Frame& f){
		return f.nextFeatureID == this->nextFeatureID;
	}

	/*
	 * initilize and set frame
	 * the next feature id will be 0;
	 */
	Frame(cv::Mat img, ros::Time t);

	/*
	 * initilize and set frame
	 * The startingID  should be equal to the lastFrame's last feature's ID
	 * if the startingID is too large it will wrap to 0
	 */
	Frame(cv::Mat img, ros::Time t, int startingID);


	/*
	 * initilize frame but don't set it
	 * nextFeatureID will be set to zero
	 */
	Frame();

	bool isFrameSet(){
		return frameSet;
	}

	/*
	 * gets the time since this image was captured
	 */
	double getAgeSeconds()
	{
		return ros::Time::now().toSec() - this->timeImageCreated.toSec();
	}


	bool addFeature(cv::KeyPoint _corner);

	bool addFeature(Feature feat);
	/*
	 * this will use the fast algorithm to find new features in the image
	 * it will then kill all features outside the kill radius
	 * then it will compare each feature with all features in the frame and ensure that it is not the same
	 * finally the remaining features will be ranked and the top n features will be added to the feature vector.
	 *
	 * nFeatures: number of features to add to the frame
	 * fast_threshold: the threshold to use with the fast algorithm
	 * kill_radius: the radius to kill features at
	 * min_feature_dist: the minimum distance between new and old features
	 */
	int getAndAddNewFeatures(int nFeatures, int fast_threshold, float kill_radius, int min_feature_dist);

	/*
	 * get the fast corners from this image
	 * and add them to the frames features
	 *
	 * This function does not check for identical features
	 */
	std::vector<Feature> getFASTCorners(int threshold);

	/*
	 * gets the a keypoint vector form the feature vector
	 */
	/*
	std::vector<cv::KeyPoint> getKeyPointVectorFromFeatures(){
		std::vector<cv::KeyPoint> kp;
		for (int i = 0; i < this->features.size(); i++)
		{
			kp.push_back(this->features.at(i).getFeature());
		}
		return kp;
	}*/
	/*
	std::vector<cv::KeyPoint> getUndistortedKeyPointVectorFromFeatures(){
		std::vector<cv::KeyPoint> kp;
		for (int i = 0; i < this->features.size(); i++)
		{
			if(this->features.at(i).isUndistorted())
			{
				cv::KeyPoint p;
				p.pt = (this->features.at(i).getUndistorted(true));
				kp.push_back(p);
			}
		}
		return kp;
	}*/

	/*
	 * gets the a keypoint vector form the feature vector
	 */
	/*
	std::vector<cv::KeyPoint> getKeyPointVectorFromFeatures(std::vector<Feature> featureVector){
		std::vector<cv::KeyPoint> kp;
		for (int i = 0; i < featureVector.size(); i++)
		{
			kp.push_back(featureVector.at(i).getFeature());
		}
		return kp;
	}*/


	/*
	 * gets a point2f vector from the local feature vector
	 */
	std::vector<cv::Point2f> getPoint2fVectorFromFeatures();

	cv::Mat extractBRIEFDescriptor(Feature feature);

	/*
	 * checks all features and undistorts them if they are not undistorted
	 */
	void undistortFeatures();

	/*
	 * searches for all features in a the local feature vector that have not been described
	 * and describes them using the BRIEF algorithm
	 */
	bool describeFeaturesWithBRIEF();
	/*
	 * take a feature vector and describe each of the features
	 */

	cv::Mat describeFeaturesWithBRIEF(cv::Mat image, std::vector<Feature> featureVector);

	/*
	 * compares two descriptors
	 * 0 is perfect match
	 * 256 is the worst possible match
	 * expects two row vectors in cvMats with uchars
	 * assumes that the two description vectors match in size
	 */
	int compareDescriptors(cv::Mat desc1, cv::Mat desc2);

	//static bool wayToSort(Feature i, Feature j);

	/* Takes Threshold for FAST corner detection and KillRadius of the Region of Interest
	 * Defines the quality of all the features and then Sorts them in ascending order of quality.
	 * 80% of quality depends on feature response and 20% on radius within region of interest.
	 */
	void rankFeatures(int fastThreshold, int killRadius);

	/* Takes Threshold for FAST corner detection and KillRadius of the Region of Interest
	 * Defines the quality of all the features and then Sorts them in ascending order of quality.
	 * 80% of quality depends on feature response and 20% on radius within region of interest.
	 *
	 * overloaded uses referenced feature vector
	 */
	void rankFeatures(std::vector<Feature>& features, int fastThreshold, int killRadius);

	/*
	 * Checks all local frame features for whether or not a feature is outside of the kill radius.
	 * It will kill the feature if it is
	 * It will set the feature's distance to center if it is not
	 * this function ensures that the feature id's remain in ascending order
	 */
	void cleanUpFeaturesByKillRadius(float killRadius);

	/*
	 * Checks all local frame features for whether or not a feature is outside of the kill radius.
	 * It will kill the feature if it is
	 * It will set the feature's distance to center if it is not
	 * this function ensures that the feature id's remain in ascending order
	 *
	 * overloaded:
	 * uses a referenced feature vector
	 */
	void cleanUpFeaturesByKillRadius(std::vector<Feature>& feats, float killRadius);

	/*
	 * this will check the feature's radius and both set an its radius
	 */
	float getAndSetFeatureRadius(Feature& feat, cv::Point2f imageCenter);

	/*
	 * uses manhattan method to find distance between two pixels
	 */
	float manhattan(cv::Point2f p1, cv::Point2f p2);

	/*
	 * removes redundant features from the clean vector by comparing
	 * its features with the compare vector and checking their
	 * distances with min_feature_dist
	 */
	void removeRedundantFeature(std::vector<Feature>& toClean, std::vector<Feature> compare, int min_feature_dist);


	double computeAverageSceneDepth(tf::Transform w2c);

	void static tfTransform2EigenAffine(tf::Transform& in , Eigen::Affine3d& out){
		for(int i=0; i<3; i++)
		{
			out.matrix()(i,3) = in.getOrigin()[i];
			for(int j=0; j<3; j++)
			{
				out.matrix()(i,j) = in.getBasis()[i][j];
			}
		}
		// Fill in identity in last row
		for (int col = 0 ; col < 3; col ++)
			out.matrix()(3, col) = 0;
		out.matrix()(3,3) = 1;
	};


};


#endif /* PAUVSI_M7_INCLUDE_PAUVSI_VO_FRAME_H_ */
