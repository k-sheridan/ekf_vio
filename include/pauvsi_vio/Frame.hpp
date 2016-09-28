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
#include <string>
#include <ros/ros.h>

#include "VIOFeature2D.hpp"

#define DEFAULT_FEATURE_SEARCH_RANGE 5

class Frame
{

private:
	cv::Ptr<cv::xfeatures2d::BriefDescriptorExtractor> descriptionExtractor;
	bool frameSet;

public:
	ros::Time timeImageCreated;
	cv::Mat image;
	std::vector<VIOFeature2D> features; //the feature vector for this frame

	/*
	 * initilize and set frame
	 */
	Frame(cv::Mat img, ros::Time t)
	{
		this->image = img;
		this->timeImageCreated = t;
		descriptionExtractor = cv::xfeatures2d::BriefDescriptorExtractor::create();
		frameSet = true;
	}

	/*
	 * initilize frame but don't set it
	 */
	Frame()
	{
		descriptionExtractor = cv::xfeatures2d::BriefDescriptorExtractor::create();
		frameSet = false;
	}

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

	/*
	 * get the fast corners from this image
	 * and add them to the frames features
	 *
	 * This function does not check for identical features
	 */
	bool getFASTCornersFromImage(cv::Mat img, int threshold){
		std::vector<cv::KeyPoint> corners;
		cv::FAST(img, corners, threshold, true); // detect with nonmax suppression

		int startingID = features.size(); // the starting ID is the current size of the feature vector

		for(int i=0; i < corners.size(); i++)
		{
			this->features.push_back(VIOFeature2D(corners.at(i), startingID + i));
		}
	}

	/*
	 * describes an individual feature
	 */
	cv::Mat extractBRIEFDescriptor(VIOFeature2D feature){
		cv::Mat descriptor;
		std::vector<cv::KeyPoint> kp(1); // must be in the form of a vector
		kp.push_back((feature.getFASTCorner()));
		descriptionExtractor->compute(this->image, kp, descriptor);
		return descriptor;
	}

	/*
	 * searches for all features in a the global feature vector that have not been described
	 * and describes them using the BRIEF algorithm
	 */
	bool describeFeaturesWithBRIEF(){
		std::vector<unsigned int> featureIndexes; // the index of the feature keypoints
		std::vector<cv::KeyPoint> featureKPs; // feature keypoints to be described

		// find all features which must be described
		for(int i = 0; i < features.size(); i++)
		{
			if(!features.at(i).isFeatureDescribed())
			{
				featureIndexes.push_back(i);
				featureKPs.push_back(features.at(i).getFASTCorner());
			}
		}

		cv::Mat descriptions;
		descriptionExtractor->compute(this->image, featureKPs, descriptions); //describe all un-described keypoints

		for(int i = 0; i < descriptions.rows; i++)
		{
			cv::Mat desc = descriptions.row(i);
			features.at(featureIndexes.at(i)).setFeatureDescription(desc);
		}

	}

};



#endif /* PAUVSI_M7_INCLUDE_PAUVSI_VO_FRAME_H_ */
