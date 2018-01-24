/*
 * TightlyCoupledEKF.cpp
 *
 *  Created on: Nov 25, 2017
 *      Author: kevin
 */

#include <TightlyCoupledEKF.h>

TightlyCoupledEKF::TightlyCoupledEKF() {

	this->Sigma.resize(BASE_STATE_SIZE, BASE_STATE_SIZE);

	//set the time to unknown, it is based of the stamp of the first message recieved
	this->t = ros::Time(0);

	//setup initial variances and values of the base state
	this->initializeBaseState();

}


void TightlyCoupledEKF::initializeBaseState()
{
	this->base_mu.setZero();
	this->Sigma.block<BASE_STATE_SIZE, BASE_STATE_SIZE>(0, 0).setZero(); //wipe the base state sigmas

	this->Sigma(0, 0) = 0;
	this->Sigma(1, 1) = 0;
	this->Sigma(2, 2) = 0;
	this->Sigma(3, 3) = 0;
	this->Sigma(4, 4) = 0;
	this->Sigma(5, 5) = 0;
	this->Sigma(6, 6) = 0;

	this->base_mu(3) = 1.0; // no rotation

	this->Sigma(7, 7) = 30;
	this->Sigma(8, 8) = 30;
	this->Sigma(9, 9) = 30;
	this->Sigma(10, 10) = 30;
	this->Sigma(11, 11) = 30;
	this->Sigma(12, 12) = 30;
	this->Sigma(13, 13) = 30;
	this->Sigma(14, 14) = 30;
	this->Sigma(15, 15) = 30;

	this->Sigma(15, 15) = 0.5; // somewhat certain about the biases
	this->Sigma(16, 16) = 0.5;
	this->Sigma(17, 17) = 0.5;
	this->Sigma(18, 18) = 0.5;
	this->Sigma(19, 19) = 0.5;
	this->Sigma(20, 20) = 0.5;

}

void TightlyCoupledEKF::addNewFeatures(std::vector<Eigen::Vector2f> new_homogenous_features){
	if(!new_homogenous_features.size()){return;}

	//resize the covariance matrix without changing other values
	int new_size = BASE_STATE_SIZE + this->features.size() * 3 + new_homogenous_features.size() * 3; // each feature has 3 degrees of freedom
	this->Sigma.conservativeResize(new_size, new_size);

	// set the new rows and columns zero
	// this ensures that the base state never has an initial correlation to these new features
	// instead the correlations should be introduced naturally through the process
	this->Sigma.block(Sigma.rows() - new_homogenous_features.size()*3, 0, new_homogenous_features.size()*3, Sigma.cols()).setZero();
	this->Sigma.block(0, Sigma.cols() - new_homogenous_features.size()*3, Sigma.rows() - new_homogenous_features.size()*3, new_homogenous_features.size()*3).setZero();

	//check the zeroing process
	//ROS_ASSERT(this->Sigma(BASE_STATE_SIZE-1, 0) == 1);
	//ROS_ASSERT(this->Sigma(0, BASE_STATE_SIZE-1) == 1);
	//ROS_ASSERT(this->Sigma(BASE_STATE_SIZE-1, BASE_STATE_SIZE-1) == 1);
	//ROS_ASSERT(this->Sigma(BASE_STATE_SIZE-1, this->Sigma.cols()-1) == 0);

	//TODO compute the average depth in the scene
	float average_scene_depth = DEFAULT_POINT_DEPTH;

	// add all new features to the state and adjust the current Sigma
	int starting_index = BASE_STATE_SIZE + this->features.size() * 3;
	for(auto e : new_homogenous_features){
		this->features.push_back(Feature(e, average_scene_depth)); // add a point with an estimated depth

		//initialize this point's uncertainties
		this->Sigma(starting_index, starting_index) = DEFAULT_POINT_HOMOGENOUS_VARIANCE;
		starting_index++;
		this->Sigma(starting_index, starting_index) = DEFAULT_POINT_HOMOGENOUS_VARIANCE;
		starting_index++;
		this->Sigma(starting_index, starting_index) = DEFAULT_POINT_DEPTH_VARIANCE; // high uncertainty for the depth of the feature
		starting_index++;
	}
}

void TightlyCoupledEKF::process(double dt){
	Eigen::SparseMatrix<float> F = this->numericallyLinearizeProcess(this->base_mu, this->features, dt); // compute the jacobian of the process numerically

	// process and update the feature vector because it depends on the base mu
	for(auto& e : this->features){
		e.setMu(this->convolveFeature(this->base_mu, e, dt));
	}

	// process the base mu
	this->base_mu = this->convolveBaseState(this->base_mu, dt);

	// update the Sigma
	this->Sigma = F * this->Sigma * F.transpose() + this->generateProcessNoise(dt);
}

Eigen::SparseMatrix<float> TightlyCoupledEKF::generateProcessNoise(float dt){
	int dim = BASE_STATE_SIZE + this->features.size.()*3;

	float low_noise = 0.0001 * dt;
	float velocity_noise = 0.01*dt;
	float omega_noise = 5*dt;
	float accel_noise = 5*dt;
	float bias_noise = 0.001*dt;

	Eigen::SparseMatrix<float> Q(dim, dim);
	Q.reserve(dim); // efficient method for setting up this matrix

	// set up base part
	Q.insert(0, 0) = low_noise;
	Q.insert(1, 1) = low_noise;
	Q.insert(2, 2) = low_noise;
	Q.insert(3, 3) = low_noise;
	Q.insert(4, 4) = low_noise;
	Q.insert(5, 5) = low_noise;
	Q.insert(6, 6) = low_noise;

	Q.insert(7, 7) = velocity_noise;
	Q.insert(8, 8) = velocity_noise;
	Q.insert(9, 9) = velocity_noise;
	Q.insert(10, 10) = omega_noise;
	Q.insert(11, 11) = omega_noise;
	Q.insert(12, 12) = omega_noise;

	Q.insert(13, 13) = accel_noise;
	Q.insert(14, 14) = accel_noise;
	Q.insert(15, 15) = accel_noise;

	Q.insert(16, 16) = bias_noise;
	Q.insert(17, 17) = bias_noise;
	Q.insert(18, 18) = bias_noise;
	Q.insert(19, 19) = bias_noise;
	Q.insert(20, 20) = bias_noise;
	Q.insert(21, 21) = bias_noise;

	// add feature noises
	int index = BASE_STATE_SIZE;
	for(auto& e : this->features){
		Q.insert(index, index) = low_noise;
		index++;
		Q.insert(index, index) = low_noise;
		index++;
		Q.insert(index, index) = low_noise;
		index++;
	}
}

Eigen::SparseMatrix<float> TightlyCoupledEKF::numericallyLinearizeProcess(Eigen::Matrix<float, BASE_STATE_SIZE, 1>& base_mu, std::list<Feature>& features, float dt){
	int dim = BASE_STATE_SIZE + features.size() * 3;
	Eigen::SparseMatrix<float> F(dim, dim);

#define QZ_INDEX 6
#define AZ_INDEX 15
#define DELTA_SHIFT 1e-3

	//TODO reserve each column

	Eigen::Matrix<float, BASE_STATE_SIZE, 1> test_mu = base_mu;


	// this is the least sparse part of the matrix
	for(int j = 0; j < BASE_STATE_SIZE; j++){
		if(j <= QZ_INDEX){ // pos and quat have no correlation to feature poritions

			test_mu(j) += DELTA_SHIFT; // shift the mu to the high test point
			Eigen::Matrix<float, BASE_STATE_SIZE, 1> derivatives = this->convolveBaseState(test_mu, dt); // temporarily store the high test point
			test_mu(j) -= 2*DELTA_SHIFT; // test_mu with negative shift
			derivatives -= this->convolveBaseState(test_mu, dt); // calculate the difference
			test_mu(j) = base_mu(j); // bring the test mu back to base mu
			derivatives /= 2*DELTA_SHIFT; // /d{var}

			// write the derivatives to the base state rows
			for(int i = 0; i < BASE_STATE_SIZE; i++){
				F.insert(i, j) = derivatives(i);
			}
			//ROS_DEBUG_STREAM("test_mu: " << test_mu.transpose());
			//ROS_DEBUG_STREAM("set derivative: " << derivatives.transpose());
		}
		else if(j <= AZ_INDEX){ // these columns correspond to the velocities and accelerations
			//first compute the base state part
			test_mu(j) += DELTA_SHIFT; // shift the mu to the high test point
			Eigen::Matrix<float, BASE_STATE_SIZE, 1> base_derivatives = this->convolveBaseState(test_mu, dt); // temporarily store the high test point
			test_mu(j) -= 2*DELTA_SHIFT; // test_mu with negative shift
			base_derivatives -= this->convolveBaseState(test_mu, dt); // calculate the difference
			test_mu(j) = base_mu(j); // bring the test mu back to base mu
			base_derivatives /= 2*DELTA_SHIFT; // /d{var}

			// write the derivatives to the base state rows
			for(int i = 0; i < BASE_STATE_SIZE; i++){
				F.insert(i, j) = base_derivatives(i);
			}

			//ROS_DEBUG_STREAM("set base derivative: " << base_derivatives.transpose());

			//compute the feature derivatives as efficiently as possible
			int row = 0;
			Eigen::VectorXf feature_derivatives(features.size()*3);

			test_mu(j) += DELTA_SHIFT; // shift the mu to the high test point

			for(auto e : features){
				feature_derivatives.segment(row, 3) = this->convolveFeature(test_mu, e.getMu(), dt);
				row += 3;
			}

			row = 0;
			test_mu(j) -= 2*DELTA_SHIFT; // test_mu with negative shift

			for(auto e : features){
				feature_derivatives.segment(row, 3) -= this->convolveFeature(test_mu, e.getMu(), dt);
				row += 3;
			}

			test_mu(j) = base_mu(j); // bring the test mu back to base mu
			feature_derivatives /= 2*DELTA_SHIFT; // /d{var}

			//ROS_DEBUG_STREAM("column: " << feature_derivatives.transpose());

			// set this column
			int matrix_row = BASE_STATE_SIZE;
			for(int i = 0; i < feature_derivatives.rows(); i++)
			{
				F.insert(matrix_row, j) = feature_derivatives(i);
				matrix_row++;
			}
		}
		else{ // biases have no correlation to feature positions
			F.insert(j, j) = 1; // biases do not change during the process
		}
	}

	//ROS_DEBUG("computing sub matrices");

	int col = BASE_STATE_SIZE;

	//compute each of the sub-jacobians for the feature
	for(auto& e : features){
		Eigen::Vector3f& base_feature_pos = e.getMu();
		Eigen::Vector3f test_feature_pos = base_feature_pos;
		Eigen::Vector3f derivative;

		//col 1
		test_feature_pos(0) += DELTA_SHIFT;
		derivative = this->convolveFeature(base_mu, test_feature_pos, dt);
		test_feature_pos(0) -= 2*DELTA_SHIFT;
		derivative -= this->convolveFeature(base_mu, test_feature_pos, dt);
		test_feature_pos(0) = base_feature_pos(0);

		derivative /= 2*DELTA_SHIFT;

		//set col 1
		int row = col;
		F.insert(row, col) = derivative(0);
		F.insert(row+1, col) = derivative(1);
		F.insert(row+2, col) = derivative(2);

		col++;

		//col 2
		test_feature_pos(1) += DELTA_SHIFT;
		derivative = this->convolveFeature(base_mu, test_feature_pos, dt);
		test_feature_pos(1) -= 2*DELTA_SHIFT;
		derivative -= this->convolveFeature(base_mu, test_feature_pos, dt);
		test_feature_pos(1) = base_feature_pos(1);

		derivative /= 2*DELTA_SHIFT;

		//set col 2
		//int row = col;
		F.insert(row, col) = derivative(0);
		F.insert(row+1, col) = derivative(1);
		F.insert(row+2, col) = derivative(2);

		col++;

		//col 3
		test_feature_pos(2) += DELTA_SHIFT;
		derivative = this->convolveFeature(base_mu, test_feature_pos, dt);
		test_feature_pos(2) -= 2*DELTA_SHIFT;
		derivative -= this->convolveFeature(base_mu, test_feature_pos, dt);
		test_feature_pos(2) = base_feature_pos(2);

		derivative /= 2*DELTA_SHIFT;

		//set col 3
		//int row = col;
		F.insert(row, col) = derivative(0);
		F.insert(row+1, col) = derivative(1);
		F.insert(row+2, col) = derivative(2);

		col++;

	}

	F.finalize();
	return F;
}


Eigen::Matrix<float, BASE_STATE_SIZE, 1> TightlyCoupledEKF::convolveBaseState(Eigen::Matrix<float, BASE_STATE_SIZE, 1>& last, float dt){
	Eigen::Vector3f pos, vel, accel, omega;
	Eigen::Quaternionf quat;

	pos = Eigen::Vector3f(last(0), last(1), last(2));
	quat = Eigen::Quaternionf(last(3), last(4), last(5), last(6));
	vel = Eigen::Vector3f(last(7), last(8), last(9));
	omega = Eigen::Vector3f(last(10), last(11), last(12));
	accel = Eigen::Vector3f(last(13), last(14), last(15));

	pos += quat*(dt*vel + 0.5*dt*dt*accel);

	float omega_norm = omega.norm();

	Eigen::Quaternionf dq;

	if(omega_norm < 1e-10){
		//small angle approximation
		dq = Eigen::Quaternionf(1.0, omega.x()*dt, omega.y()*dt, omega.z()*dt);
		dq.normalize();
	}
	else{
		float theta = dt*omega_norm;
		Eigen::Vector3f omega_hat = omega / omega_norm;
		float st2 = sin(theta/2);

		dq = Eigen::Quaternionf(cos(theta/2), omega_hat.x() * st2, omega_hat.y() * st2, omega_hat.z() * st2);
	}

	Eigen::Quaternionf dq_inv = dq.inverse();

	// body frame
	vel = dq_inv * (vel + dt*accel);
	accel = dq_inv * accel;
	quat *= dq;

	// form the mu_out;
	Eigen::Matrix<float, BASE_STATE_SIZE, 1> mu_out;

	mu_out(0) = pos.x();
	mu_out(1) = pos.y();
	mu_out(2) = pos.z();
	mu_out(3) = quat.w();
	mu_out(4) = quat.x();
	mu_out(5) = quat.y();
	mu_out(6) = quat.z();
	mu_out(7) = vel.x();
	mu_out(8) = vel.y();
	mu_out(9) = vel.z();
	//omega is the same
	mu_out(10) = last(10);
	mu_out(11) = last(11);
	mu_out(12) = last(12);

	mu_out(13) = accel.x();
	mu_out(14) = accel.y();
	mu_out(15) = accel.z();
	// biases are the same
	mu_out(16) = last(16);
	mu_out(17) = last(17);
	mu_out(18) = last(18);
	mu_out(19) = last(19);
	mu_out(20) = last(20);
	mu_out(21) = last(21);

	return mu_out;

}

Eigen::Vector3f TightlyCoupledEKF::convolveFeature(Eigen::Matrix<float, BASE_STATE_SIZE, 1>& base_state, Eigen::Vector3f& feature_state, float dt)
{

	static float last_omegax = 0;
	static float last_omegay = 0;
	static float last_omegaz = 0;
	static Eigen::Quaternionf dq_inv = Eigen::Quaternionf::Identity();

	Eigen::Vector3f vel, accel;

	//pos = Eigen::Vector3f(base_state(0), base_state(1), base_state(2));
	vel = Eigen::Vector3f(base_state(7), base_state(8), base_state(9));
	accel = Eigen::Vector3f(base_state(13), base_state(14), base_state(15));

	//convert feature to position in camera's coordinate frame
	Eigen::Vector3f feature_pos = feature_state;

	//ROS_DEBUG_STREAM("feature pos to convolve: " << feature_pos);

	feature_pos(0) = feature_pos(0)*feature_pos(2);
	feature_pos(1) = feature_pos(1)*feature_pos(2);


	Eigen::Vector3f translation = dt*vel + 0.5*dt*dt*accel;

	if(last_omegax != base_state(10) || last_omegay != base_state(11) || last_omegaz != base_state(12))
	{
		//ROS_DEBUG("omega has changed");
		Eigen::Vector3f omega = Eigen::Vector3f(base_state(10), base_state(11), base_state(12));

		float omega_norm = omega.norm();

		if(omega_norm < 1e-10){
			//small angle approximation
			dq_inv = Eigen::Quaternionf(1.0, -omega.x()*dt, -omega.y()*dt, -omega.z()*dt);
			dq_inv.normalize();
		}
		else{
			float theta = dt*omega_norm;
			Eigen::Vector3f omega_hat = omega / omega_norm;
			float st2 = sin(theta/2);

			dq_inv = Eigen::Quaternionf(cos(theta/2), -omega_hat.x() * st2, -omega_hat.y() * st2, -omega_hat.z() * st2);
		}

		last_omegax = base_state(10);
		last_omegay = base_state(11);
		last_omegaz = base_state(12);

	}

	// this transforms the point into the next camera frame
	feature_pos = dq_inv*feature_pos;
	feature_pos.noalias() += -(dq_inv*translation);

	//bring the point back to homogenous coordinates
	feature_pos(0) /= feature_pos(2);
	feature_pos(1) /= feature_pos(2);

	//ROS_DEBUG_STREAM("convolved feature: " << feature_pos);

	return feature_pos;
}

std::vector<Eigen::Vector2f> TightlyCoupledEKF::previousFeaturePositionVector(){
	std::vector<Eigen::Vector2f> output;
	//output.reserve(this->features.size());
	for(auto e : this->features){
		output.push_back(e.getLastResultFromKLTTracker());
	}

	return output;
}

/*
 * expects all measurements and uncertainties in metric coordinates
 */
void TightlyCoupledEKF::updateWithFeaturePositions(std::vector<Eigen::Vector2f> measured_positions, std::vector<Eigen::Matrix2f> estimated_covariance, std::vector<bool> pass)
{
	//ROS_DEBUG_STREAM(measured_positions.size() <<" , "<< estimated_covariance.size() <<" , "<< pass.size() <<" , "<< this->features.size());
	ROS_ASSERT(measured_positions.size() == estimated_covariance.size() && pass.size() == this->features.size() && estimated_covariance.size() == pass.size()); // make sure that there are enough features

	if(!pass.size()){
		ROS_ERROR("no measurements to update state with!");
	}

	Eigen::SparseMatrix<float> H = this->formFeatureMeasurementMap(pass); // create the mapping between the state and measurement dynamically

	ROS_DEBUG_STREAM("H rows: " << H.rows() << " H cols: " << H.cols());

	Eigen::SparseMatrix<float> R(H.rows(), H.rows()); // this will store the measurement uncertainty
	Eigen::VectorXf z(H.rows()); // this stores the measured metric feature positions
	Eigen::VectorXf mu(BASE_STATE_SIZE + this->features.size() * 3); // due to the dynamic nature of this ekf we need to create this mu vector

	//reserve the R matrix memory
	R.reserve(H.rows() * 2);

	ROS_DEBUG("start loading vectors");

	//setup the base_mu
	for(int i = 0; i < base_mu.size(); i++){mu(i) = base_mu(i);}

	//update the whole state
	int mu_index = BASE_STATE_SIZE;
	int i = 0; // track the index of the input
	int j = 0; // track the index of the new vector and matrix
	for(auto& e : this->features){

		if(pass.at(i))
		{
			//ROS_DEBUG_STREAM(measured_positions.at(i));
			e.setLastResultFromKLTTracker(measured_positions.at(i)); // used for klt tracking
			z(j) = measured_positions[i].x();
			R.insert(j, j) = estimated_covariance[i](0, 0);
			j++;

			z(j) = measured_positions[i].y();
			R.insert(j, j) = estimated_covariance[i](1, 1);
			R.insert(j-1, j) = estimated_covariance[i](0, 1);
			R.insert(j, j-1) = estimated_covariance[i](1, 0);
			j++;

			// at this point the covariance and measurement should be loaded in the right place

		}
		else
		{
			//ROS_DEBUG("fail");
			e.setDeleteFlag(true);
		}

		Eigen::Vector2f h = e.getNormalizedPixel();
		mu(mu_index) = h.x();
		mu_index++;
		mu(mu_index) = h.y();
		mu_index++;
		mu(mu_index) = e.getDepth();
		mu_index++;

		//increment
		i++;
	}

	ROS_DEBUG("start update");

	//finally update using this procedure (ensures no issues due to rounding errors)
	//y = z - H*mu
	//S = R + H*Sigma*H'
	// using x*A=b ==> A'*x_T=b'
	//K = Sigma*H'*inv(S) ==> K*S = Sigma*H' ==> S'*K' = (Sigma*H')' ==> K' = inv(S') * (Sigma*H')'
	//mu = mu + K*y
	// I_KH = I - K*H
	//Sigma = I_KH*Sigma*I_KH' + K*R*K'

	Eigen::VectorXf y = z;
	y.noalias() -= H*mu;

	ROS_DEBUG("computed residual");

	Eigen::MatrixXf S_dense(H.rows(), H.rows());
	S_dense = H * Sigma * H.transpose();
	S_dense += R;

	ROS_DEBUG("created S");

	//ROS_DEBUG_STREAM("S_dense: " << S_dense);

	// now we must solve for K
	Eigen::MatrixXf K(Sigma.rows(), H.rows());

	// sparsify S
	Eigen::SparseMatrix<float> S_sparse = S_dense.sparseView();

	ROS_DEBUG("sparsified S");
	//ROS_DEBUG_STREAM("Sparsified: " << S_sparse);

	//Eigen::LDLT<Eigen::MatrixXf> solver;
	//solver.compute(S_dense.transpose());
	//K = solver.solve((Sigma * H.transpose()).transpose()).transpose();

	Eigen::SimplicialLDLT<Eigen::SparseMatrix<float>> solver;
	solver.compute(S_sparse.transpose());
	ROS_ERROR_COND(solver.info() == Eigen::NumericalIssue, "there was a problem decomposing S... maybe it was not positive semi definite");
	K = solver.solve((Sigma * H.transpose()).transpose()).transpose();


	ROS_DEBUG_STREAM("solved for K");
	//ROS_DEBUG_STREAM("K: " << K);

	Eigen::SparseMatrix<float> K_sparse = K.sparseView();

	Eigen::SparseMatrix<float> I_KH(Sigma.rows(), Sigma.rows());
	I_KH.setIdentity();
	I_KH -= K_sparse*H;

	this->Sigma = I_KH * Sigma * I_KH; // update the covariance matrix
	this->Sigma.noalias() += K*R*K.transpose();

	ROS_DEBUG("updated sigma");

	mu += K_sparse*y; // shift the mu with the kalman gain and residual

	ROS_DEBUG("updated mu");

	//go through and update the state
	for(int i = 0; i < base_mu.size(); i++){base_mu(i) = mu(i);}

	mu_index = BASE_STATE_SIZE;
	for(auto& e : features){
		e.setNormalizedPixel(Eigen::Vector2f(mu(mu_index), mu(mu_index+1)));
		mu_index += 2;
		e.setDepth(mu(mu_index));
		mu_index++;
	}

	ROS_DEBUG("set mu");

}

/*
 * form the mapping between our measurement of feature positions (homogenous) and the state
 * the measured vector is used to tell this function which features were not observed in this measurement
 */
Eigen::SparseMatrix<float> TightlyCoupledEKF::formFeatureMeasurementMap(std::vector<bool> measured){

	ROS_ASSERT(measured.size() == this->features.size()); // sanity check

	std::vector<int> indexes;
	for(size_t i = 0; i < measured.size(); i++){
		if(measured.at(i)){
			indexes.push_back(i*3+BASE_STATE_SIZE); //this is the index of each measured feature's u in the state
		}
	}

	// the mapping from the state to the feature measurement vector
	Eigen::SparseMatrix<float> H((indexes.size()*2), (BASE_STATE_SIZE + this->features.size() * 3));

	H.reserve(indexes.size() * 2); // reserve memory for the elements

	//int index = BASE_STATE_SIZE; // index of the first feature

	int measurement_index = 0;
	for(auto e : indexes){
		H.insert(measurement_index, e) = 1.0;
		measurement_index++;
		H.insert(measurement_index, e+1) = 1.0;
		measurement_index++;
	}

	return H;
}

Eigen::Matrix2f TightlyCoupledEKF::getFeatureHomogenousCovariance(int index){
	int start = BASE_STATE_SIZE + index * 3;
	return this->Sigma.block<2, 2>(start, start);
}

void TightlyCoupledEKF::setFeatureHomogenousCovariance(int index, Eigen::Matrix2f cov)
{
	int start = BASE_STATE_SIZE + index * 3;
	this->Sigma.block<2, 2>(start, start) = cov;
}

float TightlyCoupledEKF::getFeatureDepthVariance(int index){
	int start = BASE_STATE_SIZE + index * 3 + 2;
	return this->Sigma(start, start);
}

Eigen::SparseMatrix<float> TightlyCoupledEKF::getMetric2PixelMap(Eigen::Matrix3f& K){
	Eigen::SparseMatrix<float> J(2, 2);
	//J.setIdentity();
	J.insert(0, 0) = K(0, 0);
	J.insert(1, 1) = K(1, 1);
	return J;
}

Eigen::SparseMatrix<float> TightlyCoupledEKF::getPixel2MetricMap(Eigen::Matrix3f& K){
	Eigen::SparseMatrix<float> J(2, 2);
	//J.setIdentity();
	J.insert(0, 0) = 1.0f/K(0, 0);
	J.insert(1, 1) = 1.0f/K(1, 1);
	return J;
}

