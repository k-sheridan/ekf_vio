/*
 * KeyFrame.h
 *
 *  Created on: Jan 29, 2017
 *      Author: pauvsi
 */

#ifndef PAUVSI_VIO_INCLUDE_PAUVSI_VIO_KEYFRAME_H_
#define PAUVSI_VIO_INCLUDE_PAUVSI_VIO_KEYFRAME_H_

#include "Frame.hpp"

struct KeyFrame
{

	Frame* frame;

	KeyFrame()
	{
		frame = 0;
	}

};


#endif /* PAUVSI_VIO_INCLUDE_PAUVSI_VIO_KEYFRAME_H_ */
