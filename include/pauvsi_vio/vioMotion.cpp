/*
 * vioMotion.cpp
 *
 *  Created on: Jan 16, 2017
 *      Author: kevin
 */


#include "vio.h"

void VIO::updateKeyFrameInfo()
{
	if(keyFrames.at(0).frameBufferIndex == keyFrames.at(1).frameBufferIndex == keyFrames.at(2).frameBufferIndex  == keyFrames.at(3).frameBufferIndex)
	{
		this->bruteForceKeyFrameUpdate();
	}
	else
	{
		//TODO create an optimal keyframe update algorithm using local search rather than global search
		this->bruteForceKeyFrameUpdate();
	}
}


void VIO::bruteForceKeyFrameUpdate()
{
	ros::Time start = ros::Time::now();

	Frame cf = this->frameBuffer.at(0);

	int keyFramesSet = 0;

	//initialize lastIndexes with all indexes from the current frame
	std::vector<int> lastIndexes;
	for(int i = 0; i < cf.features.size(); i++)
		lastIndexes.push_back(i);

	int kfLvl1 = round(cf.features.size() * KEYFRAME_LEVEL_1);
	int kfLvl2 = round(cf.features.size() * KEYFRAME_LEVEL_2);
	int kfLvl3 = round(cf.features.size() * KEYFRAME_LEVEL_3);
	int kfLvl4 = round(cf.features.size() * KEYFRAME_LEVEL_4);

	//ROS_DEBUG_STREAM(lastIndexes.size());

	for(int i = 1; i < this->frameBuffer.size();)
	{
		if(keyFramesSet == NUM_KEYFRAMES)
			break;

		std::vector<int> tempIndexes;
		tempIndexes.reserve(lastIndexes.size());

		// these are for collecting the smallest index
		int smallestDequeIndex = 0;
		int smallestDequeSize;
		if(lastIndexes.size() > 0){
			smallestDequeSize = cf.features.at(lastIndexes.at(0)).getMatchedIndexDeque().size();
		}
		else
		{
			break;
		}

		for(const int& cfIndex : lastIndexes)
		{

			int thisDequeSize = cf.features.at(cfIndex).getMatchedIndexDeque().size();
			if(thisDequeSize >= i)
			{
				if(cf.features.at(cfIndex).getMatchedIndexDeque().size() < smallestDequeSize)
				{
					smallestDequeSize = cf.features.at(cfIndex).getMatchedIndexDeque().size();
					smallestDequeIndex = cfIndex;
				}

				tempIndexes.push_back(cfIndex);
			}

		}

		int tempIndexSize = tempIndexes.size();

		//ROS_DEBUG_STREAM("features left: " << tempIndexSize);

		if(keyFramesSet < 1 && (tempIndexSize < kfLvl1 || tempIndexSize < 8))
		{
			keyFramesSet++;
			keyFrames.at(0).currentFrameIndexes = lastIndexes;
			keyFrames.at(0).frameBufferIndex = i - 1;
			keyFrames.at(0).nextFeatureID = this->frameBuffer.at(i - 1).nextFeatureID;
			keyFrames.at(0).pixelDelta = this->computeKeyFramePixelDelta(cf, keyFrames.at(0));

			//ROS_DEBUG_STREAM("setting keyframe 1 from inside with index " << keyFrames.at(0).frameBufferIndex);
		}

		if(keyFramesSet < 2 && (tempIndexSize < kfLvl2 || tempIndexSize < 8))
		{
			keyFramesSet++;
			keyFrames.at(1).currentFrameIndexes = lastIndexes;
			keyFrames.at(1).frameBufferIndex = i - 1;
			keyFrames.at(1).nextFeatureID = this->frameBuffer.at(i - 1).nextFeatureID;
			keyFrames.at(1).pixelDelta = this->computeKeyFramePixelDelta(cf, keyFrames.at(1));

			//ROS_DEBUG_STREAM("setting keyframe 2 from inside with index " << keyFrames.at(1).frameBufferIndex);
		}

		if(keyFramesSet < 3 && (tempIndexSize < kfLvl3 || tempIndexSize < 8))
		{
			keyFramesSet++;
			keyFrames.at(2).currentFrameIndexes = lastIndexes;
			keyFrames.at(2).frameBufferIndex = i - 1;
			keyFrames.at(2).nextFeatureID = this->frameBuffer.at(i - 1).nextFeatureID;
			keyFrames.at(2).pixelDelta = this->computeKeyFramePixelDelta(cf, keyFrames.at(2));

			//ROS_DEBUG_STREAM("setting keyframe 3 from inside with index " << keyFrames.at(2).frameBufferIndex);
		}

		if(keyFramesSet < 4 && (tempIndexSize < kfLvl4 || tempIndexSize < 8))
		{
			keyFramesSet++;
			keyFrames.at(3).currentFrameIndexes = lastIndexes;
			keyFrames.at(3).frameBufferIndex = i - 1;
			keyFrames.at(3).nextFeatureID = this->frameBuffer.at(i - 1).nextFeatureID;
			keyFrames.at(3).pixelDelta = this->computeKeyFramePixelDelta(cf, keyFrames.at(3));

			//ROS_DEBUG_STREAM("setting keyframe 4 from inside with index " << keyFrames.at(3).frameBufferIndex);
		}

		//ROS_DEBUG_STREAM("current index: " << i);
		//ROS_DEBUG_STREAM("smallest deque size: " << smallestDequeSize);
		//ROS_DEBUG_STREAM("optimization wants to skip to " << smallestDequeSize + 1 << " and get last features from " << smallestDequeSize);

		// now skip forward to the point just before a feature is lost
		if(smallestDequeSize < this->frameBuffer.size())
		{

			i = smallestDequeSize; // will be iterated once

			tempIndexes.clear(); // empty the temp indexes to be filled again

			for(const int& cfIndex : lastIndexes)
			{

				int thisDequeSize = cf.features.at(cfIndex).getMatchedIndexDeque().size();
				if(thisDequeSize >= smallestDequeSize)
				{
					tempIndexes.push_back(cfIndex);
				}
			}
		}
		else
		{
			lastIndexes = tempIndexes;
			break;
		}

		lastIndexes = tempIndexes;
		i++;
	}

	// in the case that a key frame is not set in the for loop above
	// set each keyframe to be the last frame

	int i = this->frameBuffer.size(); // assuming that I use i - 1 (which i do, this is pure laziness)
	if(keyFramesSet <= 3)
	{
		keyFrames.at(3).currentFrameIndexes = lastIndexes;
		keyFrames.at(3).frameBufferIndex = i - 1;
		keyFrames.at(3).nextFeatureID = this->frameBuffer.at(i - 1).nextFeatureID;
		keyFrames.at(3).pixelDelta = this->computeKeyFramePixelDelta(cf, keyFrames.at(3));

		//ROS_DEBUG_STREAM("setting keyframe 4 from outside with index " << keyFrames.at(3).frameBufferIndex);
	}

	if(keyFramesSet <= 2)
	{
		keyFrames.at(2).currentFrameIndexes = lastIndexes;
		keyFrames.at(2).frameBufferIndex = i - 1;
		keyFrames.at(2).nextFeatureID = this->frameBuffer.at(i - 1).nextFeatureID;
		keyFrames.at(2).pixelDelta = this->computeKeyFramePixelDelta(cf, keyFrames.at(2));

		//ROS_DEBUG_STREAM("setting keyframe 3 from outside with index " << keyFrames.at(2).frameBufferIndex);
	}

	if(keyFramesSet <= 1)
	{
		keyFrames.at(1).currentFrameIndexes = lastIndexes;
		keyFrames.at(1).frameBufferIndex = i - 1;
		keyFrames.at(1).nextFeatureID = this->frameBuffer.at(i - 1).nextFeatureID;
		keyFrames.at(1).pixelDelta = this->computeKeyFramePixelDelta(cf, keyFrames.at(1));

		//ROS_DEBUG_STREAM("setting keyframe 2 from outside with index " << keyFrames.at(1).frameBufferIndex);
	}
	if(keyFramesSet == 0)
	{
		keyFrames.at(0).currentFrameIndexes = lastIndexes;
		keyFrames.at(0).frameBufferIndex = i - 1;
		keyFrames.at(0).nextFeatureID = this->frameBuffer.at(i - 1).nextFeatureID;
		keyFrames.at(0).pixelDelta = this->computeKeyFramePixelDelta(cf, keyFrames.at(0));

		//ROS_DEBUG_STREAM("setting keyframe 1 from outside with index " << keyFrames.at(0).frameBufferIndex);
	}

	//DEBUG
	//ROS_DEBUG_STREAM("KeyFrame 1 index: " << keyFrames.at(0).frameBufferIndex << " features: " << keyFrames.at(0).currentFrameIndexes.size() << " pxlDelta: " << keyFrames.at(0).pixelDelta);
	//ROS_DEBUG_STREAM("KeyFrame 2 index: " << keyFrames.at(1).frameBufferIndex << " features: " << keyFrames.at(1).currentFrameIndexes.size() << " pxlDelta: " << keyFrames.at(1).pixelDelta);
	//ROS_DEBUG_STREAM("KeyFrame 3 index: " << keyFrames.at(2).frameBufferIndex << " features: " << keyFrames.at(2).currentFrameIndexes.size() << " pxlDelta: " << keyFrames.at(2).pixelDelta);
	//ROS_DEBUG_STREAM("KeyFrame 4 index: " << keyFrames.at(3).frameBufferIndex << " features: " << keyFrames.at(3).currentFrameIndexes.size() << " pxlDelta: " << keyFrames.at(3).pixelDelta);
	ROS_DEBUG_STREAM((ros::Time::now().toSec() - start.toSec()) * 1000 << " milliseconds runtime (key frame computation)");
}

double VIO::computeKeyFramePixelDelta(Frame cf, KeyFrameInfo& keyFrame)
{
	Frame matchFrame = this->frameBuffer.at(keyFrame.frameBufferIndex);
	ROS_ASSERT(matchFrame.nextFeatureID == keyFrame.nextFeatureID);

	std::vector<VIOFeature2D> matchedFeatures;

	double deltaSum = 0;

	for(const int& index : keyFrame.currentFrameIndexes)
	{
		if(keyFrame.frameBufferIndex == 0)
		{
			deltaSum = 0;
			break;
		}

		VIOFeature2D& ft2 = cf.features.at(index);

		int matchFeatureIndex = ft2.getMatchedIndexDeque().at(keyFrame.frameBufferIndex - 1);
		int matchFeatureID = ft2.getMatchedIDDeque().at(keyFrame.frameBufferIndex - 1);

		VIOFeature2D& ft1 = matchFrame.features.at(matchFeatureIndex);
		ROS_ASSERT(ft1.getFeatureID() == matchFeatureID);

		matchedFeatures.push_back(ft1);

		deltaSum += this->manhattan(ft1.getUndistorted(true), ft2.getUndistorted(true));
	}

	keyFrame.matchedFeatures = matchedFeatures;

	return deltaSum / (double)keyFrame.currentFrameIndexes.size();
}


VIOFeature2D VIO::getCorrespondingFeature(VIOFeature2D currFeature, Frame lastFrame)
{
	//ROS_ASSERT(currFeature.isMatched());

	VIOFeature2D lastFeature = lastFrame.features.at(currFeature.getMatchedIndex());

	ROS_ASSERT(lastFeature.getFeatureID() == currFeature.getMatchedID());

	return lastFeature;
}

double VIO::computeFundamentalMatrix(cv::Mat& F, KeyFrameInfo& kf)
{
	std::vector<cv::Point2f> pt1_temp, pt2_temp;
	cv::Mat mask;
	this->computeFundamentalMatrix(F, kf, pt1_temp, pt2_temp, mask);
}

double VIO::computeFundamentalMatrix(cv::Mat& F, KeyFrameInfo& kf, std::vector<cv::Point2f>& pt1_temp, std::vector<cv::Point2f>& pt2_temp, cv::Mat& mask)
{
	Frame& cf = currentFrame();
		Frame& mf = this->frameBuffer.at(kf.frameBufferIndex);
		ROS_ASSERT(mf.nextFeatureID == kf.nextFeatureID); // ensure that this kf is matched to the correct frame

		//std::vector<cv::Point2f> pt1_temp, pt2_temp;

		//push back undistorted and normalized image coordinates into their respective vecs
		for(auto e : kf.matchedFeatures)
		{
			pt1_temp.push_back(e.getUndistorted());
		}

		for(auto e : kf.currentFrameIndexes)
		{
			pt2_temp.push_back(cf.features.at(e).getUndistorted());
		}

		//cv::Mat mask; // this is the mask which shows what features were used for motion estimation
		cv::Mat E = cv::findEssentialMat(pt1_temp, pt2_temp, cv::Mat::eye(cv::Size(3, 3), CV_32F), cv::RANSAC, 0.999, 0.1, mask); // compute the essential/fundamental matrix

		//compute the error in the motion estimate
		double essential_error = 0;
		for(int i = 0; i < pt1_temp.size(); i++)
		{
			cv::Matx31f u1, u2;

			u1(0) = pt1_temp.at(i).x;
			u1(1) = pt1_temp.at(i).y;
			u1(2) = 1.0;

			u2(0) = pt2_temp.at(i).x;
			u2(1) = pt2_temp.at(i).y;
			u2(2) = 1.0;

			Eigen::Matrix<float, 3, 1> u1_eig, u2_eig;
			Eigen::Matrix<float, 3, 3> E_eig;

			cv::cv2eigen(u1, u1_eig);
			cv::cv2eigen(u2, u2_eig);
			cv::cv2eigen(E, E_eig);

			essential_error += abs((u2_eig.transpose() * E_eig * u1_eig)(0, 0));
		}

		//pt1 = pt1_temp;
		//pt2 = pt2_temp;

		F = E;
		return essential_error / pt1_temp.size();
}

/*
 * uses the features of the current frame and the keyframe to estimate the scaled motion between the two frames and returns a current pose estimate
 * along with the certianty of the estimate
 */
double VIO::computeFundamentalMatrix(cv::Mat& F, cv::Matx33d& R, cv::Matx31d& t, KeyFrameInfo& kf)
{
	std::vector<cv::Point2f> pt1, pt2;
	cv::Mat mask;

	double essential_error = this->computeFundamentalMatrix(F, kf, pt1, pt2, mask);

	cv::Mat R_temp, t_temp;

	cv::recoverPose(F, pt1, pt2, cv::Mat::eye(cv::Size(3, 3), CV_32F), R_temp, t_temp, mask); // chooses one of the four possible solutions for the motion using state estimates

	R_temp.convertTo(R_temp,  R.type);
	R_temp.copyTo(R);
	t_temp.convertTo(t_temp,  t.type);
	t_temp.copyTo(t);

	return essential_error;

}

tf::Transform VIO::cameraTransformFromState(VIOState x, tf::Transform b2c)
{
	return tf::Transform(x.getTFQuaternion(), tf::Vector3(x.x(), x.y(), x.z())) * b2c;
}



// OLD FOR REFERENCE

/*double VIO::recoverPoseV2( cv::InputArray E, cv::InputArray _points1, cv::InputArray _points2, cv::InputArray _cameraMatrix,
		cv::OutputArray _R, cv::OutputArray _t, cv::InputOutputArray _mask, VIOState x1, VIOState x2)
{

	cv::Mat points1, points2, cameraMatrix;
	_points1.getMat().convertTo(points1, CV_64F);
	_points2.getMat().convertTo(points2, CV_64F);
	_cameraMatrix.getMat().convertTo(cameraMatrix, CV_64F);

	int npoints = points1.checkVector(2);
	CV_Assert( npoints >= 0 && points2.checkVector(2) == npoints &&
			points1.type() == points2.type());

	CV_Assert(cameraMatrix.rows == 3 && cameraMatrix.cols == 3 && cameraMatrix.channels() == 1);

	if (points1.channels() > 1)
	{
		points1 = points1.reshape(1, npoints);
		points2 = points2.reshape(1, npoints);
	}

	double fx = cameraMatrix.at<double>(0,0);
	double fy = cameraMatrix.at<double>(1,1);
	double cx = cameraMatrix.at<double>(0,2);
	double cy = cameraMatrix.at<double>(1,2);

	points1.col(0) = (points1.col(0) - cx) / fx;
	points2.col(0) = (points2.col(0) - cx) / fx;
	points1.col(1) = (points1.col(1) - cy) / fy;
	points2.col(1) = (points2.col(1) - cy) / fy;

	points1 = points1.t();
	points2 = points2.t();

	cv::Mat R1, R2, t;
	cv::decomposeEssentialMat(E, R1, R2, t);
	cv::Mat P0 = cv::Mat::eye(3, 4, R1.type());
	cv::Mat P1(3, 4, R1.type()), P2(3, 4, R1.type()), P3(3, 4, R1.type()), P4(3, 4, R1.type());
	P1(cv::Range::all(), cv::Range(0, 3)) = R1 * 1.0; P1.col(3) = t * 1.0;
	P2(cv::Range::all(), cv::Range(0, 3)) = R2 * 1.0; P2.col(3) = t * 1.0;
	P3(cv::Range::all(), cv::Range(0, 3)) = R1 * 1.0; P3.col(3) = -t * 1.0;
	P4(cv::Range::all(), cv::Range(0, 3)) = R2 * 1.0; P4.col(3) = -t * 1.0;

	// Do the cheirality check.
	// Notice here a threshold dist is used to filter
	// out far away points (i.e. infinite points) since
	// there depth may vary between postive and negtive.
	double dist = 50.0;
	cv::Mat Q;
	cv::triangulatePoints(P0, P1, points1, points2, Q);
	cv::Mat mask1 = Q.row(2).mul(Q.row(3)) > 0;
	Q.row(0) /= Q.row(3);
	Q.row(1) /= Q.row(3);
	Q.row(2) /= Q.row(3);
	Q.row(3) /= Q.row(3);
	mask1 = (Q.row(2) < dist) & mask1;
	Q = P1 * Q;
	mask1 = (Q.row(2) > 0) & mask1;
	mask1 = (Q.row(2) < dist) & mask1;

	cv::triangulatePoints(P0, P2, points1, points2, Q);
	cv::Mat mask2 = Q.row(2).mul(Q.row(3)) > 0;
	Q.row(0) /= Q.row(3);
	Q.row(1) /= Q.row(3);
	Q.row(2) /= Q.row(3);
	Q.row(3) /= Q.row(3);
	mask2 = (Q.row(2) < dist) & mask2;
	Q = P2 * Q;
	mask2 = (Q.row(2) > 0) & mask2;
	mask2 = (Q.row(2) < dist) & mask2;

	cv::triangulatePoints(P0, P3, points1, points2, Q);
	cv::Mat mask3 = Q.row(2).mul(Q.row(3)) > 0;
	Q.row(0) /= Q.row(3);
	Q.row(1) /= Q.row(3);
	Q.row(2) /= Q.row(3);
	Q.row(3) /= Q.row(3);
	mask3 = (Q.row(2) < dist) & mask3;
	Q = P3 * Q;
	mask3 = (Q.row(2) > 0) & mask3;
	mask3 = (Q.row(2) < dist) & mask3;

	cv::triangulatePoints(P0, P4, points1, points2, Q);
	cv::Mat mask4 = Q.row(2).mul(Q.row(3)) > 0;
	Q.row(0) /= Q.row(3);
	Q.row(1) /= Q.row(3);
	Q.row(2) /= Q.row(3);
	Q.row(3) /= Q.row(3);
	mask4 = (Q.row(2) < dist) & mask4;
	Q = P4 * Q;
	mask4 = (Q.row(2) > 0) & mask4;
	mask4 = (Q.row(2) < dist) & mask4;

	mask1 = mask1.t();
	mask2 = mask2.t();
	mask3 = mask3.t();
	mask4 = mask4.t();

	// If _mask is given, then use it to filter outliers.
	if (!_mask.empty())
	{
		cv::Mat mask = _mask.getMat();
		CV_Assert(mask.size() == mask1.size());
		cv::bitwise_and(mask, mask1, mask1);
		cv::bitwise_and(mask, mask2, mask2);
		cv::bitwise_and(mask, mask3, mask3);
		cv::bitwise_and(mask, mask4, mask4);
	}
	if (_mask.empty() && _mask.needed())
	{
		_mask.create(mask1.size(), CV_8U);
	}

	CV_Assert(_R.needed() && _t.needed());
	_R.create(3, 3, R1.type());
	_t.create(3, 1, t.type());

	double chProb1 = (double)countNonZero(mask1) / (double)npoints;
	double chProb2 = (double)countNonZero(mask2) / (double)npoints;
	double chProb3 = (double)countNonZero(mask3) / (double)npoints;
	double chProb4 = (double)countNonZero(mask4) / (double)npoints;

	//1 - R1 t
	//2 - R2 t
	//3 - R1 -t
	//4 - R2 -t

	tf::StampedTransform base2cam;
	try{
		this->ekf.tf_listener.lookupTransform(this->camera_frame, this->CoM_frame, ros::Time(0), base2cam);
	}
	catch(tf::TransformException& e){
		ROS_WARN_STREAM(e.what());
	}

	tf::Transform tf_cam1 = tf::Transform(x1.getTFQuaternion(), tf::Vector3(x1.x(), x1.y(), x1.z())) * base2cam;
	tf::Transform tf_cam2 = tf::Transform(x2.getTFQuaternion(), tf::Vector3(x2.x(), x2.y(), x2.z())) * base2cam;

	tf::Transform tf_imu = (tf_cam1.inverse() * tf_cam2).inverse();

	tf::Vector3 t_imu = tf_imu.getOrigin().normalized();
	tf::Quaternion q_imu = tf_imu.getRotation();
	tf::Vector3 t_ego = tf::Vector3(t.at<double>(0), t.at<double>(1), t.at<double>(2));

	//r1 to q_r1;
	Eigen::Matrix3f eig_R1, eig_R2;
	Eigen::Quaternionf eig_R1_q, eig_R2_q;
	cv::cv2eigen(R1, eig_R1);
	cv::cv2eigen(R2, eig_R2);
	eig_R1_q = eig_R1;
	eig_R2_q = eig_R2;
	tf::Quaternion R1_q = tf::Quaternion(eig_R1_q.x(), eig_R1_q.y(), eig_R1_q.z(), eig_R1_q.w());
	tf::Quaternion R2_q = tf::Quaternion(eig_R2_q.x(), eig_R2_q.y(), eig_R2_q.z(), eig_R2_q.w());

	double negTProb = (t_imu.dot(-1 * t_ego) + 1.0) / 2.0;
	double posTProb = (t_imu.dot(t_ego) + 1.0) / 2.0;
	double R1Prob = q_imu.angleShortestPath(R1_q) / CV_PI;
	double R2Prob = q_imu.angleShortestPath(R2_q) / CV_PI;

	ROS_DEBUG_STREAM("R1: " << R1Prob << " R2: " << R2Prob);
	ROS_DEBUG_STREAM("t: " << posTProb << " -t: " << negTProb);
	ROS_DEBUG_STREAM("t: " << t.at<double>(0) << ", " << t.at<double>(1) << ", " << t.at<double>(2));

	double imuProb1 = 0.6 * R1Prob + 0.4 * posTProb;
	double imuProb2 = 0.6 * R2Prob + 0.4 * posTProb;
	double imuProb3 = 0.6 * R1Prob + 0.4 * negTProb;
	double imuProb4 = 0.6 * R2Prob + 0.4 * negTProb;
	ROS_ASSERT(imuProb1 >= 0 && imuProb1 <= 1 && imuProb2 >= 0 && imuProb2 <= 1);

	double prob1 = 0.3 * imuProb1 + 0.7 * chProb1;
	double prob2 = 0.3 * imuProb2 + 0.7 * chProb2;
	double prob3 = 0.3 * imuProb3 + 0.7 * chProb3;
	double prob4 = 0.3 * imuProb4 + 0.7 * chProb4;

	//finalize and return;
	if(prob1 > prob2 && prob1 > prob3 && prob1 > prob4)
	{
		R1.copyTo(_R);
		t = tf_imu.getOrigin().length() * t;
		t.copyTo(_t);
		ROS_DEBUG_STREAM("R1 & t chosen");
		if (_mask.needed()) mask1.copyTo(_mask);
		return prob1;
	}
	else if(prob2 > prob1 && prob2 > prob3 && prob2 > prob4)
	{
		R2.copyTo(_R);
		t = tf_imu.getOrigin().length() * t;
		t.copyTo(_t);
		ROS_DEBUG_STREAM("R2 & t chosen");
		if (_mask.needed()) mask2.copyTo(_mask);
		return prob2;
	}
	else if(prob3 > prob2 && prob3 > prob1 && prob3 > prob4)
	{
		R1.copyTo(_R);
		t = -1 * tf_imu.getOrigin().length() * t;
		t.copyTo(_t);
		ROS_DEBUG_STREAM("R1 & -t chosen");
		if (_mask.needed()) mask3.copyTo(_mask);
		return prob3;
	}
	else
	{
		R2.copyTo(_R);
		t = -1 * tf_imu.getOrigin().length() * t;
		t.copyTo(_t);
		ROS_DEBUG_STREAM("R2 & -t chosen");
		if (_mask.needed()) mask4.copyTo(_mask);
		return prob4;
	}



}



double VIO::poseFromPoints(std::vector<VIOFeature3D> actives, Frame lf, Frame cf, Eigen::Matrix<double, 7, 1>& Z, bool& pass)
{
	if(actives.size() < 3)
	{
		pass = false;
		return DBL_MAX;
	}

	std::vector<cv::Point3f> objectPoints;
	std::vector<cv::Point2f> imagePoints;

	cv::Mat tvec, rvec;

	double cov_sum = 0;
	int totalMatches = 0;

	for(int i = 0; i < actives.size(); i++)
	{
		if(lf.features.at(actives.at(i).current2DFeatureMatchIndex).forwardMatched)
		{
			objectPoints.push_back(cv::Point3f(actives.at(i).position(0), actives.at(i).position(1), actives.at(i).position(2)));

			VIOFeature2D pt = cf.features.at(lf.features.at(actives.at(i).current2DFeatureMatchIndex).forwardMatchIndex);

			ROS_ASSERT(pt.getFeatureID() == lf.features.at(actives.at(i).current2DFeatureMatchIndex).forwardMatchID);
			ROS_ASSERT(pt.getMatchedID() == lf.features.at(actives.at(i).current2DFeatureMatchIndex).getFeatureID());
			ROS_ASSERT(actives.at(i).current2DFeatureMatchID = lf.features.at(actives.at(i).current2DFeatureMatchIndex).getFeatureID());
			ROS_ASSERT(pt.getMatchedID() == actives.at(i).current2DFeatureMatchID);

			imagePoints.push_back(pt.getUndistorted(true));

			ROS_DEBUG_STREAM("3D Point: " << cv::Point3f(actives.at(i).position(0), actives.at(i).position(1), actives.at(i).position(2)) << " \nCorresponding to: " << pt.getFeaturePosition()
					<< "\nwith cov: " << actives.at(i).variance << "\n");

			cov_sum += actives.at(i).variance;
			totalMatches++;
		}
	}

	if(objectPoints.size() < 3)
	{
		pass = false;
		return DBL_MAX;
	}

	tf::StampedTransform base2cam;
	try{
		this->ekf.tf_listener.lookupTransform(this->camera_frame, this->CoM_frame, ros::Time(0), base2cam);
	}
	catch(tf::TransformException& e){
		ROS_WARN_STREAM(e.what());
	}

	float reprojError;
	cv::Mat inliers;
	//cv::Mat::eye(cv::Size(3, 3), CV_32F)
	//cv::solvePnP(objectPoints, imagePoints, cv::Mat::eye(cv::Size(3, 3), CV_32F), cv::noArray(), rvec, tvec, false, cv::SOLVEPNP_P3P);
	cv::solvePnPRansac(objectPoints, imagePoints, cv::Mat::eye(cv::Size(3, 3), CV_32F), cv::noArray(), rvec, tvec, false, 100, 0.5, 0.99, inliers, cv::SOLVEPNP_P3P);

	cv::Mat cv_R;

	cv::Rodrigues(rvec, cv_R);

	tf::Vector3 t = tf::Vector3(tvec.at<float>(0), tvec.at<float>(1), tvec.at<float>(2));

	Eigen::Matrix<float, 3, 3> R;
	cv::cv2eigen(cv_R, R);

	Eigen::Quaternionf q_eig(R.transpose());
	tf::Quaternion q = tf::Quaternion(q_eig.x(), q_eig.y(), q_eig.z(), q_eig.w());

	tf::Transform world2base = tf::Transform(q, t).inverse() * base2cam.inverse();

	Z(0, 0) = world2base.getOrigin().x();
	Z(1, 0) = world2base.getOrigin().y();
	Z(2, 0) = world2base.getOrigin().z();

	tf::Quaternion newQ = world2base.getRotation();

	Z(3, 0) = newQ.w();
	Z(4, 0) = newQ.x();
	Z(5, 0) = newQ.y();
	Z(6, 0) = newQ.z();

	ROS_DEBUG_STREAM("PNP: r: " << Z(0) << ", " << Z(1) << ", " << Z(2) << " q: " << Z(3) << ", " << Z(4) << ", " << Z(5) << ", " << Z(6));
	ROS_DEBUG_STREAM("PNP COV: " << cov_sum / totalMatches);

	pass = true;
	return cov_sum / totalMatches;
}*/
