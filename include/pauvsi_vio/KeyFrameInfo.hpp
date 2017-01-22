/*
 * KeyFrameInfo.hpp
 *
 *  Created on: Jan 16, 2017
 *      Author: kevin
 */

#ifndef PAUVSI_VIO_INCLUDE_PAUVSI_VIO_KEYFRAMEINFO_HPP_
#define PAUVSI_VIO_INCLUDE_PAUVSI_VIO_KEYFRAMEINFO_HPP_


class KeyFrameInfo
{
public:
	int frameBufferIndex; // the index of the frame this keyframe corresponds to in the frame buffer
	int nextFeatureID; // the next feature ID of the frame this KeyFrame corresponds to

	double pixelDelta; // the pixel delta between this frame and the current frame

	std::vector<VIOFeature2D> matchedFeatures; // the features from this frame that are matched to the current frame indexes

	std::vector<int> currentFrameIndexes; // these are the indexes of the features in the current frame which are matched with this frame.

	cv::Matx34f P; //this is the scaled projection matrix for this keyframe

	KeyFrameInfo()
	{
		frameBufferIndex = 0;
		nextFeatureID = 0;
		pixelDelta = 0;
	}
};



#endif /* PAUVSI_VIO_INCLUDE_PAUVSI_VIO_KEYFRAMEINFO_HPP_ */
