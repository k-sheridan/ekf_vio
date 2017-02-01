/*
 * KeyFrame.h
 *
 *  Created on: Jan 29, 2017
 *      Author: pauvsi
 */

#ifndef PAUVSI_VIO_INCLUDE_PAUVSI_VIO_KEYFRAME_H_
#define PAUVSI_VIO_INCLUDE_PAUVSI_VIO_KEYFRAME_H_

#include <Frame.h>

struct KeyFrame
{

	Frame* frame;

	KeyFrame()
	{
		frame = NULL;
	}

};


#endif /* PAUVSI_VIO_INCLUDE_PAUVSI_VIO_KEYFRAME_H_ */
