/*
 * InertialMotionEstimator.cpp
 *
 *  Created on: Oct 18, 2016
 *      Author: kevinsheridan
 */

#include "InertialMotionEstimator.h"

InertialMotionEstimator::InertialMotionEstimator() {
	// TODO Auto-generated constructor stub

}

/*
 * this gets the inertial motion estimate from the imu buffer to the specified time
 * this is gotten from the fromPose and fromVelocity at their times.
 * the results are output in the angle, pos, vel change vectors
 * it returns the number of IMU readings used
 *
 * angle change is in radians RPY
 * removes all imu messages from before the toTime
 *
 * inputs in odom frame
 * outputs in camera frame
 *
 * sets the last measured angular velocity to the state vector
 *
 * this was very poorly written sorry
 * it contained many bugs and becae very messy as a result
 *
 * NOTE:
 * this assumes a rigid transformation between the IMU - Camera - odom-base
 * do not move the imu and camera relative to each other!
 *
 * NOTE 2:
 * the fromVelocity and from Angular velocity must be in terms of the camera frame
 */


//int InertialMotionEstimator::getInertialMotionEstimate(ros::Time fromTime, ros::Time toTime, tf::Vector3 fromVelocity,
//		tf::Vector3 fromAngularVelocity, tf::Vector3& angleChange,
//		tf::Vector3& positionChange, tf::Vector3& velocityChange, tf::Vector3& lastOmega)
//{
//	int startingIMUBufferSize = this->imuMessageBuffer.size();
//
//	//check if there are any imu readings
//	if(this->imuMessageBuffer.size() == 0)
//	{
//		return 0;
//	}
//
//	tf::StampedTransform odom2camera;
//	try{
//		vio.tf_listener.lookupTransform(vio.camera_frame, vio.odom_frame, ros::Time(0), odom2camera);
//	}
//	catch(tf::TransformException e)
//	{
//		ROS_WARN_STREAM("IN ME 2 " << e.what());
//	}
//
//	//transform the two from vectors
//	fromAngularVelocity = odom2camera * fromAngularVelocity - odom2camera * tf::Vector3(0.0, 0.0, 0.0);
//	fromVelocity = odom2camera * fromVelocity - odom2camera * tf::Vector3(0.0, 0.0, 0.0);
//
//	//ROS_DEBUG_STREAM("ang velo: " << fromAngularVelocity.getX() << " velo: " << fromVelocity.getX());
//
//	int startingIMUIndex, endingIMUIndex;
//
//	/* first find the IMU reading index such that the toTime is greater than
//	 * the stamp of the IMU reading.
//	 */
//	for(int i = this->imuMessageBuffer.size() - 1; i >= 0; i--)
//	{
//		if(this->imuMessageBuffer.at(i).header.stamp.toSec() < toTime.toSec())
//		{
//			endingIMUIndex = i;
//			break;
//		}
//	}
//
//	/*
//	 * second find the IMU reading index such that the fromTime is lesser than the
//	 * stamp of the IMU message
//	 */
//	//ROS_DEBUG_STREAM("buffer size " << this->imuMessageBuffer.size());
//	/*
//	 * this will catch the fromTime > toTime error
//	 */
//	if(fromTime.toNSec() < toTime.toNSec())
//	{
//		for(int i = 0; i < this->imuMessageBuffer.size(); i++)
//		{
//			if(this->imuMessageBuffer.at(i).header.stamp.toSec() > fromTime.toSec())
//			{
//				startingIMUIndex = i;
//				break;
//			}
//		}
//	}
//	else
//	{
//		ROS_ERROR_STREAM("from Time is " << fromTime.toNSec() << " to time is " << toTime.toNSec() << " starting from i = 0");
//		startingIMUIndex = 0;
//	}
//
//	//now we have all IMU readings between the two times.
//
//	/*
//	 * third create the corrected accelerations vector
//	 * 1) remove centripetal acceleration
//	 * 2) remove gravity
//	 */
//
//	tf::Vector3 dTheta(0, 0, 0);
//	double piOver180 = CV_PI / 180.0;
//
//	tf::Vector3 gravity(0.0, 0.0, vio.GRAVITY_MAG); // the -gravity accel vector in the world frame gravity magnitude is variable to adjust for biases
//
//	//get the world->odom transform
//	tf::StampedTransform world2Odom;
//	try{
//		vio.tf_listener.lookupTransform(vio.odom_frame, vio.world_frame, ros::Time(0), world2Odom);
//	}
//	catch(tf::TransformException e)
//	{
//		ROS_WARN_STREAM(e.what());
//	}
//
//	//get the imu->camera transform
//	tf::StampedTransform imu2camera;
//	try{
//		vio.tf_listener.lookupTransform(vio.camera_frame, vio.imu_frame, ros::Time(0), imu2camera);
//	}
//	catch(tf::TransformException e)
//	{
//		ROS_WARN_STREAM(e.what());
//	}
//
//	gravity = world2Odom * gravity - world2Odom * tf::Vector3(0.0, 0.0, 0.0); // rotate gravity vector into odom frame
//
//	//ROS_DEBUG("nothing to do with tf stuff!");
//
//	//std::vector<tf::Vector3> correctedIMUAccels;
//	std::vector<tf::Vector3> cameraAccels;
//	std::vector<ros::Time> cameraAccelTimes;
//
//	//ROS_DEBUG_STREAM("starting end indexes " << startingIMUIndex << ", " << endingIMUIndex << " buffer size " << this->imuMessageBuffer.size());
//
//	for(int i = startingIMUIndex; i <= endingIMUIndex; i++)
//	{
//		//get the tf::vectors for the raw IMU readings
//		//ROS_DEBUG_STREAM("i value is" << i << "starting value is " << startingIMUIndex << "ending value is " << endingIMUIndex);
//
//		if(i < 0 || i >= this->imuMessageBuffer.size())
//		{
//			ROS_ERROR_STREAM("i value is " << i << " starting value is " << startingIMUIndex << " ending value is " << endingIMUIndex << " vec size " << this->imuMessageBuffer.size() << " CONTINUE");
//			continue;
//		}
//
//		sensor_msgs::Imu msg = this->imuMessageBuffer.at(i);
//		//ROS_DEBUG("pass 1");
//		tf::Vector3 omegaIMU(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z);
//		//ROS_DEBUG_STREAM("1 omega " << omegaIMU.getX());
//		//convert the omega vec to rads
//		omegaIMU = piOver180 * omegaIMU;
//		//ROS_DEBUG_STREAM("2 omega " << omegaIMU.getX());
//		double omegaIMU_mag = omegaIMU.length();
//
//		//ROS_DEBUG_STREAM("3 omega length" << omegaIMU_mag);
//		tf::Vector3 omegaIMU_hat;
//		if(omegaIMU_mag != 0)
//		{
//			omegaIMU_hat = (1 / omegaIMU_mag) * omegaIMU;
//		}
//		else
//		{
//			omegaIMU_hat = omegaIMU;
//		}
//
//		tf::Vector3 alphaIMU(msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z);
//
//		//compute the centripetal accel
//		tf::StampedTransform distFromRotationAxisTF;
//
//		// the transformation to the CoM frame in the imu_frame
//		try{
//			vio.tf_listener.lookupTransform(vio.imu_frame, vio.CoM_frame, ros::Time(0), distFromRotationAxisTF);
//		}
//		catch(tf::TransformException e){
//			ROS_WARN_STREAM_ONCE("THIS! " << e.what());
//		}
//
//		//get the centripetal acceleration expected
//		tf::Vector3 deltaR = distFromRotationAxisTF.getOrigin();
//		double deltaR_mag = sqrt(pow(deltaR.getX(), 2) + pow(deltaR.getY(), 2) + pow(deltaR.getZ(), 2));
//		tf::Vector3 deltaR_hat = (1 / deltaR_mag) * deltaR;
//		//mag accel = omega^2 * r
//		//the accel is proportional to the perpendicularity of omega and deltaR
//		//ROS_DEBUG_STREAM("ca unit vecs: " << deltaR_hat.getX() << ", " << omegaIMU_hat.getX());
//		double perpCoeff = deltaR_hat.cross(omegaIMU_hat).length();
//		//calculate
//		tf::Vector3 centripetalAccel = perpCoeff * omegaIMU_mag * omegaIMU_mag * deltaR_mag * deltaR_hat;
//
//		//ROS_DEBUG_STREAM("ca: " << centripetalAccel.getX() << ", " << centripetalAccel.getY() << ", " << centripetalAccel.getZ() << " perp: " << perpCoeff);
//
//		//if this is not the first iteration
//		if(i != startingIMUIndex)
//		{
//			//get the last angular velocity
//			sensor_msgs::Imu last_msg = this->imuMessageBuffer.at(i - 1);
//			//ROS_DEBUG("pass 2");
//			tf::Vector3 last_omegaIMU(last_msg.angular_velocity.x, last_msg.angular_velocity.y, last_msg.angular_velocity.z);
//			//convert the omega vec to rads
//			last_omegaIMU = piOver180 * last_omegaIMU;
//
//			//get the new dTheta - dTheta = dTheta + omega * dt
//			dTheta = dTheta + last_omegaIMU * (msg.header.stamp.toSec() - last_msg.header.stamp.toSec());
//			//ROS_DEBUG_STREAM("dt: " << (msg.header.stamp.toSec() - last_msg.header.stamp.toSec()));
//		}
//		else
//		{
//			tf::Transform camera2IMU = imu2camera.inverse();
//
//			tf::Vector3 omega = camera2IMU * (piOver180 * fromAngularVelocity) - camera2IMU * tf::Vector3(0.0, 0.0, 0.0);
//
//			//get the new dTheta - dTheta = dTheta + omega * dt
//			dTheta = dTheta + omega * (msg.header.stamp.toSec() - fromTime.toSec());
//		}
//
//		//publish the temp IMU transform
//		//ROS_DEBUG_STREAM("dTheta: " << dTheta.getX() << ", " << dTheta.getY() << ", " << dTheta.getZ());
//
//		//this->broadcastWorldToOdomTF(); // tell tf what the current world -> odom is
//		//this->broadcastOdomToTempIMUTF(dTheta.getX(), dTheta.getY(), dTheta.getZ(), 0, 1, 0);
//
//		//create a transform from odom to the tempIMU
//		tf::Quaternion q;
//		q.setRPY(dTheta.getX(), dTheta.getY(), dTheta.getZ());
//		tf::Transform odom2TempIMU(q);
//
//		//transform the gravity vector into the temp IMU frame
//		tf::Vector3 imuGravity = odom2TempIMU * gravity - odom2TempIMU * tf::Vector3(0.0, 0.0, 0.0);
//
//		//push the corrected acceleration to the correct accel vector
//
//		tf::Vector3 correctedIMUAccel = (alphaIMU - imuGravity - centripetalAccel);
//		//tf::Vector3 correctedIMUAccel = (alphaIMU - imuGravity);
//
//		//transform the imu accel to the camera frame
//		cameraAccels.push_back(imu2camera * correctedIMUAccel - tf::Vector3(0.0, 0.0, 0.0));
//		cameraAccelTimes.push_back(msg.header.stamp);
//
//		//output
//		//ROS_DEBUG_STREAM("raw accel: " << alphaIMU.getX() << ", " << alphaIMU.getY() << ", " << alphaIMU.getZ());
//		//ROS_DEBUG_STREAM("grav: " << gravity.getX() << ", " << gravity.getY() << ", " << gravity.getZ());
//		//ROS_DEBUG_STREAM("ca: " << centripetalAccel.getX() << ", " << centripetalAccel.getY() << ", " << centripetalAccel.getZ() << " perp: " << perpCoeff);
//		//ROS_DEBUG_STREAM("trans grav: " << imuGravity.getX() << ", " << imuGravity.getY() << ", " << imuGravity.getZ());
//		//ROS_DEBUG_STREAM("corrected accels: " << correctedIMUAccel.getX() << ", " << correctedIMUAccel.getY() << ", " << correctedIMUAccel.getZ());
//		//ROS_DEBUG_STREAM("camera accels: " << cameraAccels.at(cameraAccels.size()-1).getX() << ", " << cameraAccels.at(cameraAccels.size()-1).getY() << ", " << cameraAccels.at(cameraAccels.size()-1).getZ());
//	}
//
//	//finish the dTheta for the last time segment
//	sensor_msgs::Imu lastMsg = this->imuMessageBuffer.at(endingIMUIndex);
//	tf::Vector3 omega(lastMsg.angular_velocity.x, lastMsg.angular_velocity.y, lastMsg.angular_velocity.z);
//
//	dTheta = dTheta + (piOver180 * omega) * (toTime.toSec() - lastMsg.header.stamp.toSec());
//
//	/*
//	 * transform and set the last measured
//	 * omega into the odom frame
//	 */
//	/*
//	tf::Stamped<tf::Vector3> w;
//	tf::Stamped<tf::Vector3> w_odom;
//	w.setData(piOver180 * omega);
//	w.stamp_ = ros::Time(0);
//	w.frame_id_ = vio.imu_frame;
//	try{
//		vio.tf_listener.transformVector(vio.odom_frame, w, w_odom);
//	}
//	catch(tf::TransformException e){
//		ROS_WARN_STREAM(e.what());
//	}
//	vio.angular_velocity.vector.x = w_odom.getX();
//	vio.angular_velocity.vector.y = w_odom.getY();
//	vio.angular_velocity.vector.z = w_odom.getZ();
//	vio.angular_velocity.header.frame_id = vio.odom_frame;
//	vio.angular_velocity.header.stamp = lastMsg.header.stamp;
//	*/
//
//
//	//ROS_DEBUG_STREAM("dTheta: " << dTheta.getX() << ", " << dTheta.getY() << ", " << dTheta.getZ());
//	//ROS_DEBUG_STREAM("time diff " << currentFrame.timeImageCreated.toSec() - this->imuMessageBuffer.at(endingIMUIndex).header.stamp.toSec());
//
//	/*
//	 * at this point we have a dTheta from inside the IMU's frame and correct accelerations from inside the camera's frame
//	 * now we must transform the dTheta into the camera frame
//	 * we must also integrate the accels inside the camera frame
//	 */
//
//	angleChange = imu2camera * dTheta - imu2camera * tf::Vector3(0.0, 0.0, 0.0); //set the angle change vector
//
//	//INTEGRATE the cameraAccels
//
//	positionChange = tf::Vector3(0.0, 0.0, 0.0); //0,0,0 initial pos
//	velocityChange = fromVelocity; // initial velocity (subtract out fromVelocity at end!!!)
//
//	if(cameraAccels.size() == 0) // if there are not accelerations
//	{
//		double dt = toTime.toSec() - fromTime.toSec();
//		positionChange += velocityChange * dt;
//	}
//	else // integrate all time segments
//	{
//		double dt = toTime.toSec() - cameraAccelTimes.at(0).toSec();
//		positionChange += velocityChange * dt;
//		for(int i = 0; i < cameraAccels.size(); i++)
//		{
//			if(i != cameraAccels.size() - 1) // if this is not the last element
//			{
//				double dt = cameraAccelTimes.at(i + 1).toSec() - cameraAccelTimes.at(i).toSec();
//				positionChange += 0.5 * cameraAccels.at(i) * (dt * dt) + velocityChange * dt;
//				velocityChange += cameraAccels.at(i) * dt;
//			}
//			else
//			{
//				double dt = toTime.toSec() - cameraAccelTimes.at(i).toSec();
//				positionChange += 0.5 * cameraAccels.at(i) * (dt * dt) + velocityChange * dt;
//				velocityChange += cameraAccels.at(i) * dt;
//			}
//		}
//	}
//
//	//remove from velocity from the velo Change
//	velocityChange = velocityChange - fromVelocity;
//
//	//finally once everything has been estimated remove all IMU messages from the buffer that have been used and before that
//	//ROS_ASSERT(this->imuMessageBuffer.size() == startingIMUBufferSize);
//
//	std::vector<sensor_msgs::Imu> newIMUBuffer;
//	for(int i = endingIMUIndex + 1; i < startingIMUBufferSize; i++)
//	{
//		newIMUBuffer.push_back(this->imuMessageBuffer.at(i));
//	}
//
//	this->imuMessageBuffer = newIMUBuffer; //erase all old IMU messages
//
//
//	return endingIMUIndex - startingIMUIndex + 1;
//}


