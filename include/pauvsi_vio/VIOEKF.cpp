/*
 * VIOEKF.cpp
 *
 *  Created on: Oct 28, 2016
 *      Author: kevin
 */

#include <VIOEKF.h>

VIOEKF::VIOEKF() {
	this->gyroBiasX = 0;
	this->gyroBiasY = 0;
	this->gyroBiasZ = 0;
	this->scaleAccelerometer = 1.0;
	this->GRAVITY_MAG = 9.8;

	ros::param::param<bool>("~convert2rad", convert2rad, CONVERT_2_RAD_DEFAULT);

}

VIOEKF::~VIOEKF() {
	// TODO Auto-generated destructor stub
}

VIOState VIOEKF::update(VIOState in, VisualMeasurement z)
{
	VIOState out = in;

	Eigen::Matrix<double, 7, 16> H;
	H << 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0;

	Eigen::Matrix<double, 7, 1> y = z.z - H*in.vector;

	Eigen::Matrix<double, 7, 7> S = H * in.covariance * H.transpose() + z.covariance;

	Eigen::Matrix<double, 16, 7> K = in.covariance * H.transpose() * S.inverse();

	out.vector = in.vector + K * y;

	out.covariance = (Eigen::MatrixXd::Identity(16, 16) - K*H) * in.covariance;

	return out;
}

VIOState VIOEKF::predict(VIOState lastState, ros::Time predictionTime)
{
	VIOState state = lastState;
	std::vector<sensor_msgs::Imu> imuMsgs;
	int imuMsgsSize = this->getMessagesBetweenTimes(lastState.getTime(), predictionTime, imuMsgs);

	if(imuMsgsSize > 0)
	{
		for(int i = 0; i < imuMsgs.size(); i++)
		{
			//this runs once to set the starting time of the system
			// I had to do this because rosbag time was not working properly
			if(!state.timeSet)
			{
				state.setTime(imuMsgs.at(i).header.stamp);
			}

			state = this->transitionState(state, imuMsgs.at(i).header.stamp.toSec() - state.getTime().toSec()); // predict and propagate
			state.setIMU(imuMsgs.at(i)); // set the new alpha and omega
		}
		this->lastMessageUsed = imuMsgs.at(imuMsgs.size() - 1); //set the last message used to the final message in the buffer
	}

	//predict to the end
	state = this->transitionState(state, predictionTime.toSec() - state.getTime().toSec());
	// this sets the states omega and alpha to the last state's omega and alpha

	//compute the full covariance for the entire pred step

	Eigen::Matrix<double, 16, 16> F = this->stateJacobian(lastState, predictionTime.toSec() - lastState.getTime().toSec()); // compute the state transition jacobian
	Eigen::Matrix<double, 16, 16> predictionError = this->computePredictionError(predictionTime.toSec() - lastState.getTime().toSec());
	//propagate the error using the jacobian
	state.covariance = F * state.covariance * F.transpose() + predictionError;

	//ROS_DEBUG_STREAM("new cov: " << state.covariance);

	return state;
}

/*
 * this function will use the state's current imu measurment to predict dt seconds into the future
 * and return the updated state.
 * NOTE: this function will keep the same imu reading as the last state x
 */
VIOState VIOEKF::transitionState(VIOState x, double dt)
{
	//ROS_DEBUG_STREAM("transitioning state with dt = " << dt);
	//ROS_DEBUG_STREAM("state before: " << x.vector);
	// get the imu 2 com transform
	tf::StampedTransform imu2odom;
	try{
		tf_listener.lookupTransform(this->odom_frame, this->imu_frame, ros::Time(0), imu2odom);
	}
	catch(tf::TransformException e){
		ROS_WARN_STREAM(e.what());
	}

	//convert the imu readings to tf::Vectors and remove their biases
	tf::Vector3 alpha_tf(x.getAlpha()(0), x.getAlpha()(1), x.getAlpha()(2));
	alpha_tf = this->scaleAccelerometer * alpha_tf;
	tf::Vector3 omega_tf(x.getOmega()(0) - this->gyroBiasX, x.getOmega()(1)- this->gyroBiasY, x.getOmega()(2) - this->gyroBiasZ);

	//ROS_DEBUG_STREAM("original omega " << omega_tf.getX() << ", " << omega_tf.getY() << ", " << omega_tf.getZ());

	//transform the imu readings into the center of mass frame
	alpha_tf = imu2odom * alpha_tf - imu2odom * tf::Vector3(0.0, 0.0, 0.0);
	omega_tf = imu2odom * omega_tf - imu2odom * tf::Vector3(0.0, 0.0, 0.0);

	Eigen::Vector3d alpha, omega;
	alpha << alpha_tf.getX(), alpha_tf.getY(), alpha_tf.getZ();
	omega << omega_tf.getX(), omega_tf.getY(), omega_tf.getZ();

	//ROS_DEBUG_STREAM("alpha " << alpha << "\n omega " << omega);

	VIOState xNew;

	//create quaternion to rotate the alpha vector
	Eigen::Quaterniond q(x.q0(), x.q1(), x.q2(), x.q3());
	//ROS_DEBUG_STREAM("q: " << q.w() << ", " << q.x() << ", " << q.y() << ", " << q.z());
	alpha = q.toRotationMatrix() * alpha; // rotate alpha into world coordinate frame

	//experimental
	omega = q.toRotationMatrix() * omega; // rotate alpha into world coordinate frame

	// these equations are from matlab's quatrotate function
	double ax = alpha(0);
	double ay = alpha(1);
	double az = alpha(2);

	//ROS_DEBUG_STREAM("newAX: " << ax << " newAY: " << ay << " newAZ: " << az);

	// compute the delta quaternion
	double w_mag = sqrt(omega(0)*omega(0) + omega(1)*omega(1) + omega(2)*omega(2));

	//ROS_DEBUG_STREAM("w_mag: " << w_mag);

	double dq0 = 1.0;
	double dq1 = 0;
	double dq2 = 0;
	double dq3 = 0;

	if(w_mag != 0)
	{
		dq0 = cos(0.5 * w_mag * dt);
		dq1 = (omega(0) / w_mag) * sin(0.5 * w_mag * dt);
		dq2 = (omega(1) / w_mag) * sin(0.5 * w_mag * dt);
		dq3 = (omega(2) / w_mag) * sin(0.5 * w_mag * dt);
	}

	Eigen::Quaterniond dq(dq0, dq1, dq2, dq3); // the delta quaternion
	dq.normalize();
	//ROS_DEBUG_STREAM("dq: " << dq.w() << ", " << dq.x() << ", " << dq.y() << ", " << dq.z());

	Eigen::Quaterniond newQ = dq * q; // rotate the quaternions
	newQ.normalize(); // normalize the final
	//ROS_DEBUG_STREAM("dq * q: " << newQ.w() << ", " << newQ.x() << ", " << newQ.y() << ", " << newQ.z());




	//transition state
	xNew.vector(0, 0) = x.x() + x.dx()*dt + 0.5 * ax * dt*dt;
	//xNew.vector(0, 0) = x.x();
	xNew.vector(1, 0) = x.y() + x.dy()*dt + 0.5 * ay * dt*dt;
	//xNew.vector(1, 0) = x.y();
	xNew.vector(2, 0) = x.z() + x.dz()*dt + 0.5 * (az - this->GRAVITY_MAG) * dt*dt;
	//xNew.vector(2, 0) = x.z();
	xNew.vector(3, 0) = x.dx() + ax * dt;
	//xNew.vector(3, 0) = x.dx();
	xNew.vector(4, 0) = x.dy() + ay * dt;
	//xNew.vector(4, 0) = x.dy();
	xNew.vector(5, 0) = x.dz() + (az - this->GRAVITY_MAG) * dt;
	//xNew.vector(5, 0) = x.dz();

	xNew.vector(6, 0) = newQ.w();
	//xNew.vector(6, 0) = dq.w();
	xNew.vector(7, 0) = newQ.x();
	//xNew.vector(7, 0) = dq.x();
	xNew.vector(8, 0) = newQ.y();
	//xNew.vector(8, 0) = dq.y();
	xNew.vector(9, 0) = newQ.z();
	//xNew.vector(9, 0) = dq.z();

	//ROS_DEBUG_STREAM("state after: " << xNew.vector);

	//set the same imu reading
	xNew.setAlpha(x.getAlpha());
	xNew.setOmega(x.getOmega());

	//update the new state time
	xNew.setTime(ros::Time(x.getTime().toSec() + dt));


	return xNew;
}

/*
 * this constructs the jacobain of the state transition function
 */
Eigen::Matrix<double, 16, 16> VIOEKF::stateJacobian(VIOState state, double dt){

	Eigen::Matrix<double, 16, 16> F;

	double x = state.x();
	double y = state.y();
	double z = state.z();
	double dx = state.dx();
	double dy = state.dy();
	double dz = state.dz();
	double q0 = state.q0();
	double q1 = state.q1();
	double q2 = state.q2();
	double q3 = state.q3();
	double ax = state.getAlpha()(0);
	double ay = state.getAlpha()(1);
	double az = state.getAlpha()(2);
	double wx = state.getOmega()(0);
	double wy = state.getOmega()(1);
	double wz = state.getOmega()(2);
	double omegaMag = state.getOmega().norm();

	//handle the two cases for omega
	if(omegaMag > 0.0)
	{
		double s1 = (dt*omegaMag)/2;
		double s2 = omegaMag;
		double cs1 = cos(s1);
		double ss1 = sin(s1);
		double s3 = 4*q3*wx*wx*cs1*cs1*cs1 - 4*q3*wy*wy*cs1 - 4*q3*wz*wz*cs1 - 4*q3*wx*wx*cs1 +
				4*q3*wy*wy*cs1*cs1*cs1 + 4*q3*wz*wz*cs1*cs1*cs1 - dt*q2*wy*wy*wy*cs1 + dt*q1*wy*wy*wy*cs1 -
				dt*q0*wy*wy*wy*cs1 + dt*q1*wx*wx*wy*cs1 - dt*q2*wx*wy*wy*cs1 - dt*q0*wx*wx*wz*cs1 -
				dt*q2*wx*wz*wz*cs1 - dt*q0*wy*wy*wz*cs1 + dt*q1*wy*wz*wz*cs1 + 2*dt*q3*s2*wx*wx*ss1 +
				2*dt*q3*s2*wy*wy*ss1 + 2*dt*q3*s2*wz*wz*ss1 + 2*q2*s2*wx*cs1*cs1*ss1 - 2*q1*s2*wy*cs1*cs1*ss1 + 2*q0*s2*wz*cs1*cs1*ss1;
		double s4 = 4*q2*wx*wx*cs1*cs1*cs1 - 4*q2*wy*wy*cs1 - 4*q2*wz*wz*cs1 - 4*q2*wx*wx*cs1 +
				4*q2*wy*wy*cs1*cs1*cs1 + 4*q2*wz*wz*cs1*cs1*cs1 + dt*q3*wx*wx*wx*cs1 - dt*q0*wy*wy*wy*cs1 -
				dt*q1*wz*wz*wz*cs1 - dt*q0*wx*wx*wy*cs1 + dt*q3*wx*wy*wy*cs1 - dt*q1*wx*wx*wz*cs1 +
				dt*q3*wx*wz*wz*cs1 - dt*q0*wy*wz*wz*cs1 - dt*q1*wy*wy*wz*cs1 +
				2*dt*q2*s2*wx*wx*ss1 + 2*dt*q2*s2*wy*wy*ss1 + 2*dt*q2*s2*wz*wz*ss1 -
				2*q3*s2*wx*cs1*cs1*ss1 + 2*q0*s2*wy*cs1*cs1*ss1 + 2*q1*s2*wz*cs1*cs1*ss1;
		double s5 = 4*q1*wx*wx*cs1*cs1*cs1 - 4*q1*wy*wy*cs1 - 4*q1*wz*wz*cs1 - 4*q1*wx*wx*cs1 +
				4*q1*wy*wy*cs1*cs1*cs1 + 4*q1*wz*wz*cs1*cs1*cs1 - dt*q0*wx*wx*wx*cs1 - dt*q3*wy*wy*wy*cs1 +
				dt*q2*wz*wz*wz*cs1 - dt*q0*wx*wy*wy*cs1 - dt*q3*wx*wx*wy*cs1 - dt*q0*wx*wz*wz*cs1 +
				dt*q2*wx*wx*wz*cs1 + dt*q2*wy*wy*wz*cs1 - dt*q3*wy*wz*wz*cs1 + 2*dt*q1*s2*wx*wx*ss1 +
				2*dt*q1*s2*wy*wy*ss1 + 2*dt*q1*s2*wz*wz*ss1 + 2*q0*s2*wx*cs1*cs1*ss1 +
				2*q3*s2*wy*cs1*cs1*ss1 - 2*q2*s2*wz*cs1*cs1*ss1;
		double s6 = 4*q0*wx*wx*cs1*cs1*cs1 - 4*q0*wy*wy*cs1 - 4*q0*wz*wz*cs1 - 4*q0*wx*wx*cs1 +
				4*q0*wy*wy*cs1*cs1*cs1 + 4*q0*wz*wz*cs1*cs1*cs1 + dt*q1*wx*wx*wx*cs1 + dt*q2*wy*wy*wy*cs1 +
				dt*q3*wz*wz*wz*cs1 + dt*q1*wx*wy*wy*cs1 + dt*q2*wx*wx*wy*cs1 + dt*q1*wx*wz*wz*cs1 +
				dt*q3*wx*wx*wz*cs1 + dt*q2*wy*wz*wz*cs1 + dt*q3*wy*wy*wz*cs1 +
				2*dt*q0*s2*wx*wx*ss1 + 2*dt*q0*s2*wy*wy*ss1 + 2*dt*q0*s2*wz*wz*ss1 -
				2*q1*s2*wx*cs1*cs1*ss1 - 2*q2*s2*wy*cs1*cs1*ss1 - 2*q3*s2*wz*cs1*cs1*ss1;
		double s7 = 5 - 3*cos(dt*s2);
		double s8 = sqrt(2);
		double s9 = 1/sqrt(s7);
		double s10 = dt*dt;
		double s11 = 1/(s2*s2*s2*s2);
		double s12 = s12 = 1/s2;
		double s13 = 2*s8*s9*s12*wx*ss1;
		double s14 = 2*s8*s9*s12*wy*ss1;
		double s15 = s8*s9*cs1;
		double s16 = s9*s9;
		double s17 = q0*q1 - q2*q3;
		double s18 = q0*q3 - q1*q2;
		double s19 = 2*s8*s9*s12*wz*ss1;
		double s20 = az*q1;
		double s21 = 2*ax*q2;
		double s22 = s20 - ax*q3;
		double s23 = 2*ay*q3;
		double s24 = q2*q2;
		double s25 = q1*q1;
		double s26 = q3*q3;
		double s27 = q0*q2;
		double s28 = 2*ay*q2;
		double s29 = 2*ax*q1;
		double s30 = 2*ax*q0;
		double s31 = 2*ay*q0;
		double s32 = 2*az*q3;
		double s33 = 2*az*q0;
		double s34 = 2*s25;
		double s35 = 2*s24;
		double s36 = 2*s22;
		double s37 = 2*s26;
		double s38 = ay*q2;
		double s39 = ax*q1;
		double s40 = ay*q3;
		double s41 = ax*q0;
		double s42 = ay*q0;
		double s43 = ax*q2;


		F << 1, 0, 0, dt,  0,  0,  s10*(s40 - az*q2),           s10*(s38 + az*q3),  -s10*(s21 - ay*q1 + az*q0),     s10*(s22 + s42 - ax*q3),                      0,                      0,                      0, -s10*(s24 + s26 - 1/2),    s10*(q0*q3 + q1*q2),     -s10*(s27 - q1*q3),
				0, 1, 0,  0, dt,  0,            s10*s22, s10*(s43 - 2*ay*q1 + az*q0),           s10*(s39 + az*q3),    -s10*(s23 + s41 - az*q2),                      0,                      0,                      0,               -s10*s18, -s10*(s25 + s26 - 1/2),    s10*(q0*q1 + q2*q3),
				0, 0, 1,  0,  0, dt,  s10*(s43 - ay*q1),      -s10*(s20 + s22 + s42),   s10*(s40 + s41 - 2*az*q2),             s10*(s38 + s39),                      0,                      0,                      0,      s10*(s27 + q1*q3),               -s10*s17, -s10*(s24 + s25 - 1/2),
				0, 0, 0,  1,  0,  0, dt*(s23 - 2*az*q2),              dt*(s28 + s32), -dt*(s33 + 4*s43 - 2*ay*q1),    dt*(s31 + s36 - 2*ax*q3),                      0,                      0,                      0,    -dt*(s35 + s37 - 1), dt*(2*q0*q3 + 2*q1*q2),    -2*dt*(s27 - q1*q3),
				0, 0, 0,  0,  1,  0,             dt*s36,    dt*(s21 + s33 - 4*ay*q1),              dt*(s29 + s32), -dt*(s30 + 4*s40 - 2*az*q2),                      0,                      0,                      0,              -2*dt*s18,    -dt*(s34 + s37 - 1), dt*(2*q0*q1 + 2*q2*q3),
				0, 0, 0,  0,  0,  1, dt*(s21 - 2*ay*q1),     -dt*(2*s20 + s31 + s36),    dt*(s23 + s30 - 4*az*q2),              dt*(s28 + s29),                      0,                      0,                      0,   dt*(2*s27 + 2*q1*q3),              -2*dt*s17,    -dt*(s34 + s35 - 1),
				0, 0, 0,  0,  0,  0,                s15,                        -s13,                        -s14,                        -s19, -2*s6*s8*s9*s11*s16*wx, -2*s6*s8*s9*s11*s16*wy, -2*s6*s8*s9*s11*s16*wz,                      0,                      0,                      0,
				0, 0, 0,  0,  0,  0,                s13,                         s15,                        -s19,                         s14, -2*s5*s8*s9*s11*s16*wx, -2*s5*s8*s9*s11*s16*wy, -2*s5*s8*s9*s11*s16*wz,                      0,                      0,                      0,
				0, 0, 0,  0,  0,  0,                s14,                         s19,                         s15,                        -s13, -2*s4*s8*s9*s11*s16*wx, -2*s4*s8*s9*s11*s16*wy, -2*s4*s8*s9*s11*s16*wz,                      0,                      0,                      0,
				0, 0, 0,  0,  0,  0,                s19,                        -s14,                         s13,                         s15, -2*s3*s8*s9*s11*s16*wx, -2*s3*s8*s9*s11*s16*wy, -2*s3*s8*s9*s11*s16*wz,                      0,                      0,                      0,
				0, 0, 0,  0,  0,  0,                  0,                           0,                           0,                           0,                      1,                      0,                      0,                      0,                      0,                      0,
				0, 0, 0,  0,  0,  0,                  0,                           0,                           0,                           0,                      0,                      1,                      0,                      0,                      0,                      0,
				0, 0, 0,  0,  0,  0,                  0,                           0,                           0,                           0,                      0,                      0,                      1,                      0,                      0,                      0,
				0, 0, 0,  0,  0,  0,                  0,                           0,                           0,                           0,                      0,                      0,                      0,                      1,                      0,                      0,
				0, 0, 0,  0,  0,  0,                  0,                           0,                           0,                           0,                      0,                      0,                      0,                      0,                      1,                      0,
				0, 0, 0,  0,  0,  0,                  0,                           0,                           0,                           0,                      0,                      0,                      0,                      0,                      0,                      1;
	}
	else
	{
		F = Eigen::MatrixXd::Identity(16, 16); // for now just set Identity in this case. TODO
	}

	//ROS_DEBUG_STREAM("F = " << F);
	return F;
}

/*
 * this function tells the ekf how prediction error is related to time
 */
Eigen::Matrix<double, 16, 16> VIOEKF::computePredictionError(double dt)
{
	Eigen::Matrix<double, 16, 16> PE = Eigen::MatrixXd::Identity(16, 16);
	double dts = dt*dt;
	PE(0, 0) = dts;
	PE(1, 1) = dts;
	PE(2, 2) = dts;
	PE(3, 3) = 0.01*dts;
	PE(4, 4) = 0.01*dts;
	PE(5, 5) = 0.01*dts;
	PE(6, 6) = dts;
	PE(7, 7) = dts;
	PE(8, 8) = dts;
	PE(9, 9) = dts;
	PE(10, 10) = 0.03*dts;
	PE(11, 11) = 0.03*dts;
	PE(12, 12) = 0.03*dts;
	PE(13, 13) = 0.03*dts;
	PE(14, 14) = 0.03*dts;
	PE(15, 15) = 0.03*dts;

	return PE;
}

/*
 * gets all imu messages between the two times and places them in the returnBuffer
 * removes all imu messages before the final time
 * returns size of return buffer
 */
int VIOEKF::getMessagesBetweenTimes(ros::Time t0, ros::Time t1, std::vector<sensor_msgs::Imu>& returnBuffer)
{
	int originalIMUBufferSize = this->imuMessageBuffer.size();
	int returnBufferSize = 0;

	std::vector<sensor_msgs::Imu> newImuBuffer;

	double startTime = t0.toSec();
	double endTime = t1.toSec();

	// if there are messages in the imu buffer
	if(originalIMUBufferSize > 0)
	{
		for(int i = 0; i < originalIMUBufferSize; i++)
		{
			double msgTime = this->imuMessageBuffer.at(i).header.stamp.toSec();

			//if this message is useful
			if(msgTime > startTime && msgTime < endTime)
			{
				returnBuffer.push_back(this->imuMessageBuffer.at(i));
				returnBufferSize++;
			}
			if(msgTime > endTime)
			{
				newImuBuffer.push_back(this->imuMessageBuffer.at(i));
			}
		}
	}

	return returnBufferSize;
}


