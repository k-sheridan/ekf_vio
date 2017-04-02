/*
 * vioMotion.cpp
 *
 *  Created on: Jan 16, 2017
 *      Author: kevin
 */


#include "vio.h"

/*
 * this function will add new keyframes to the list and remove obsolete
 * keyframes from the list
 *
 * it adds a new keyframe is the baseline dist / avg scene depth > Threshold
 * this method was taken from
 * SVO, Forster, Christian and Pizzoli, Matia and Scaramuzza, Davide
 */
void VIO::updateKeyFrameInfo() {


	//clean up old keyframes and excess keyframes
	while(!keyFrames.empty() && (keyFrames.back().frame->finalFrame || keyFrames.size() > MAX_KEYFRAMES))
	{
		ROS_DEBUG("removing an old keyframe");
		ROS_ASSERT(keyFrames.back().frame != NULL);
		keyFrames.back().frame->isKeyframe = false;
		keyFrames.pop_back();
	}

	if(keyFrames.empty())
	{
		keyFrames.push_front(KeyFrame(&currentFrame(), currentFrame().features.size())); // in the case f no keyframes use the current frame
		keyFrames.front().frame->isKeyframe = true;
	}
	else
	{
		ROS_ASSERT(keyFrames.front().frame != NULL);
		double keyFrameRatio = (currentFrame().state.getr() - keyFrames.front().frame->state.getr()).norm() / ((currentFrame().avgSceneDepth + keyFrames.front().frame->avgSceneDepth) / 2);

		if(keyFrameRatio >= NEW_KEYFRAME_RATIO)
		{
			keyFrames.push_front(KeyFrame(&currentFrame(), currentFrame().features.size())); // in the case f no keyframes use the current frame
			keyFrames.front().frame->isKeyframe = true;

			// optimize the position of each point in the frame currently
			ROS_INFO("PERFORMING BA ON ALL POINTS");
			for(auto e : currentFrame().features)
			{
				e.point->SBA(10);
			}
		}

	}
}

tf::Transform VIO::cameraTransformFromState(VIOState x, tf::Transform b2c) {
	return tf::Transform(x.getTFQuaternion(), tf::Vector3(x.x(), x.y(), x.z())) * b2c;
}

VIOState VIO::transformState(VIOState x, tf::Transform trans) {
	tf::Transform transformed = cameraTransformFromState(x, trans);

	ROS_DEBUG_STREAM("cameraTransform from inside " << transformed.getOrigin().x() << ", " << transformed.getOrigin().y() << ", " << transformed.getOrigin().z());

	tf::Quaternion newQ = transformed.getRotation();

	x.vector(0, 0) = transformed.getOrigin().x();
	x.vector(1, 0) = transformed.getOrigin().y();
	x.vector(2, 0) = transformed.getOrigin().z();

	x.vector(6, 0) = newQ.w();
	x.vector(7, 0) = newQ.x();
	x.vector(8, 0) = newQ.y();
	x.vector(9, 0) = newQ.z();

	return x;
}

VIOState VIO::transformStateInverse(VIOState x, tf::Transform trans) {
	tf::Transform transformed = cameraTransformFromState(x, trans).inverse();

	ROS_DEBUG_STREAM("cameraTransform from inside " << transformed.getOrigin().x() << ", " << transformed.getOrigin().y() << ", " << transformed.getOrigin().z());

	tf::Quaternion newQ = transformed.getRotation();

	x.vector(0, 0) = transformed.getOrigin().x();
	x.vector(1, 0) = transformed.getOrigin().y();
	x.vector(2, 0) = transformed.getOrigin().z();

	x.vector(6, 0) = newQ.w();
	x.vector(7, 0) = newQ.x();
	x.vector(8, 0) = newQ.y();
	x.vector(9, 0) = newQ.z();

	return x;
}






