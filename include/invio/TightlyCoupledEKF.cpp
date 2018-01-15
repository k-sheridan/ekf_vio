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

TightlyCoupledEKF::~TightlyCoupledEKF() {
	// TODO Auto-generated destructor stub
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

	this->Sigma(7, 7) = 100000;
	this->Sigma(8, 8) = 100000;
	this->Sigma(9, 9) = 100000;
	this->Sigma(10, 10) = 100000;
	this->Sigma(11, 11) = 100000;
	this->Sigma(12, 12) = 100000;
	this->Sigma(13, 13) = 100000;
	this->Sigma(14, 14) = 100000;
	this->Sigma(15, 15) = 100000;

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
void TightlyCoupledEKF::updateWithFeaturePositions(Frame& cf, std::vector<Eigen::Vector2f> measured_positions, std::vector<Eigen::Matrix2f> estimated_covariance, std::vector<bool> pass)
{
	ROS_DEBUG_STREAM(measured_positions.size() <<" , "<< estimated_covariance.size() <<" , "<< pass.size() <<" , "<< this->features.size());
	ROS_ASSERT(measured_positions.size() == estimated_covariance.size() && pass.size() == this->features.size() && estimated_covariance.size() == pass.size()); // make sure that there are enough features

	if(!pass.size()){
		ROS_ERROR("no measurements to update state with!");
	}

	Eigen::SparseMatrix<float> H = this->formFeatureMeasurementMap(pass); // create the mapping between the state and measurement dynamically


	Eigen::SparseMatrix<float> R(H.rows(), H.rows()); // this will store the measurement uncertainty
	Eigen::VectorXf z(H.rows()); // this stores the measured metric feature positions
	Eigen::VectorXf mu(BASE_STATE_SIZE + this->features.size() * 3); // due to the dynamic nature of this ekf we need to create this mu vector

	//update the whole state
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

		//increment
		i++;
	}

	//finally update using this procedure (ensures no issues due to rounding errors)
	//y = z - H*mu
	//S = R + H*Sigma*H'
	// using x*A=b ==> A'*x_T=b'
	//K = Sigma*H'*inv(S) ==> K*S = Sigma*H' ==> S'*K' = (Sigma*H')' ==> K' = inv(S') * (Sigma*H')' ==> K' = inv(S') * (Sigma'*H)
	//mu = mu + K*y
	// I_KH = I - K*H
	//Sigma = I_KH*Sigma*I_KH' + K*R*K'

	Eigen::VectorXf y = z;
	y -= H*mu;

	Eigen::MatrixXf S;
	S.noalias() = H*Sigma*H.transpose();
	S += R;


}

/*
 * form the mapping between our measurement of feature positions (homogenous) and the state
 * the measured vector is used to tell this function which features were not observed in this measurement
 */
Eigen::SparseMatrix<float> TightlyCoupledEKF::formFeatureMeasurementMap(std::vector<bool> measured){

	ROS_ASSERT(measured.size() == this->features.size()); // sanity check

	std::vector<int> indexes;
	for(int i = 0; i < measured.size(); i++){
		if(measured.at(i)){
			indexes.push_back(i*3+BASE_STATE_SIZE); //this is the index of each measured feature's u in the state
		}
	}

	// the mapping from the state to the feature measurement vector
	Eigen::SparseMatrix<float> H((indexes.size()*2), (BASE_STATE_SIZE + this->features.size() * 3));

	int index = BASE_STATE_SIZE; // index of the first feature

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

