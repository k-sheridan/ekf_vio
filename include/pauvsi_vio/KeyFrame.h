/*
 * KeyFrame.h
 *
 *  Created on: Jan 29, 2017
 *      Author: pauvsi
 */

#ifndef PAUVSI_VIO_INCLUDE_PAUVSI_VIO_KEYFRAME_H_
#define PAUVSI_VIO_INCLUDE_PAUVSI_VIO_KEYFRAME_H_

#include <Frame.h>

class Frame;

struct KeyFrame
{

	Frame* frame;

	int numFeatures;

	KeyFrame()
	{
		frame = NULL;
		numFeatures = 0;
	}

	KeyFrame(Frame* _f, int _numFeats)
	{
		frame = _f;
		numFeatures = _numFeats;
	}

};


#endif /* PAUVSI_VIO_INCLUDE_PAUVSI_VIO_KEYFRAME_H_ */
