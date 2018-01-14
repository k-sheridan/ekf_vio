/*
 * KLTTracker.cpp
 *
 *  Created on: Jan 3, 2018
 *      Author: kevin
 */

#include <KLTTracker.h>

KLTTracker::KLTTracker()
{
	// TODO Auto-generated constructor stub

}

KLTTracker::~KLTTracker()
{
	// TODO Auto-generated destructor stub
}

/*
 * the main klt tracker function
 * iteratively determines the new sub-pixel accurate feature positions in the new frame and estimates its covariance matrix
 *
 * the initial version of this code is not vectorized by me and ideally gets auto-vectorized by the compiler
 *
 * all inputs and outputs are in homogenous metric coordinates
 */
void KLTTracker::findNewFeaturePositions(const Frame& lf, const Frame& cf, const std::vector<Eigen::Vector2f>& previous_feature_positions,
		const std::list<Feature>& estimated_new_feature_positions, std::vector<Eigen::Vector2f>& measured_positions,
		std::vector<Eigen::Matrix2f>& estimated_uncertainty, std::vector<bool>& passed)
{
	// for now I use the opencv built in klt tracker with custom uncertainty estimation
	this->findNewFeaturePositionsOpenCV(lf, cf, previous_feature_positions, estimated_new_feature_positions, measured_positions, estimated_uncertainty, passed);
}

/*
 * uses the built in opencv klt tracker and estimates the uncertainty of the results
 */
void KLTTracker::findNewFeaturePositionsOpenCV(const Frame& lf, const Frame& cf, const std::vector<Eigen::Vector2f>& previous_feature_positions,
		const std::list<Feature>& estimated_new_feature_positions, std::vector<Eigen::Vector2f>& measured_positions,
		std::vector<Eigen::Matrix2f>& estimated_uncertainty, std::vector<bool>& passed)
{

	std::vector<cv::Point2f> prev_fts, new_fts;
	std::vector<uchar> status; // status vector for each point
	cv::Mat error; // error vector for each point


	ROS_DEBUG_STREAM(previous_feature_positions.size() << " , " << estimated_new_feature_positions.size());
	ROS_ASSERT(previous_feature_positions.size() == estimated_new_feature_positions.size());
	//load the vectors
	for(auto e : previous_feature_positions){
		prev_fts.push_back(Feature::metric2Pixel(lf, e));

	}
	for(auto e : estimated_new_feature_positions){
		new_fts.push_back(e.getPixel(cf));
	}

	cv::calcOpticalFlowPyrLK(lf.img, cf.img, prev_fts, new_fts,
			status, error, cv::Size(WINDOW_SIZE, WINDOW_SIZE), MAX_PYRAMID_LEVEL,
			cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS,
					30, 0.01), cv::OPTFLOW_USE_INITIAL_FLOW, KLT_MIN_EIGEN);

	//estimate the covariance of each measurement
	//transform and write the results
	passed.resize(new_fts.size());
	estimated_uncertainty.resize(new_fts.size());
	measured_positions.resize(new_fts.size());

	for(int i = 0; i < new_fts.size(); i++){
		if(status.at(i) == 1 && !(new_fts[i].x < KILL_PAD || new_fts[i].y < KILL_PAD || cf.img.cols - new_fts[i].x < KILL_PAD || cf.img.rows - new_fts[i].y < KILL_PAD)){
			passed[i] = (true);
			estimated_uncertainty[i] = (this->estimateUncertainty(cf, new_fts.at(i)));
			measured_positions[i] = Feature::pixel2Metric(cf, new_fts.at(i));
		}
		else
		{
			passed[i] = (false);
			estimated_uncertainty[i] = (Eigen::Matrix2f::Zero());
		}
	}


}

/*
 * sum the 2x2 image hessian matrices over around the feature and invert it
 * this approximates the uncertainty in the feature position
 */
Eigen::Matrix2f KLTTracker::estimateUncertainty(const Frame& cf, cv::Point2f mu){
	Eigen::Matrix2f A;

	int mu_u = round(mu.x);
	int mu_v = round(mu.y);

	int half_length = (WINDOW_SIZE - 1)/2 + 1; // need one extra pixel for initial gradient calculation

	int lower_u_bound = std::max(mu_u - half_length, 2); // 1 should be min so that the gradient kernel will work
	int lower_v_bound = std::max(mu_v - half_length, 2);
	int upper_u_bound = std::min(mu_u + half_length, cf.img.cols-3);
	int upper_v_bound = std::min(mu_v + half_length, cf.img.rows-3);

	//ROS_DEBUG_STREAM("u: " << mu_u << " v: " << mu_v);
	//ROS_DEBUG_STREAM("image size: " << cf.img.size);
	//ROS_DEBUG_STREAM(lower_u_bound <<" , "<< upper_u_bound<<" , "<<lower_v_bound<<" , "<<upper_v_bound);
	//ROS_ASSERT(lower_u_bound < upper_u_bound && lower_v_bound < upper_v_bound);

	int length = WINDOW_SIZE + 2;

	float gradx[length][length];
	float grady[length][length];

	int i = 0;
	for(int u = lower_u_bound; u <= upper_u_bound; u++)
	{
		int j = 0;
		for(int v = lower_v_bound; v <= upper_v_bound; v++)
		{
			//ROS_ASSERT(i < length);
			//ROS_ASSERT(j < length);

			gradx[i][j] =  0.5f * (cf.img.at<uchar>(u+1, v)-cf.img.at<uchar>(u-1, v)); // gradu
			grady[i][j] =  0.5f * (cf.img.at<uchar>(u, v+1)-cf.img.at<uchar>(u, v-1)); // gradv

			j++;
		}
		i++;
	}

	//use a smaller window for final hessian calc
	int upper_bound = length - 2;

	float hxx = 0;
	float hxy = 0;
	float hyy = 0;

	// compute and add the hessians
	for(int i = 1; i <= upper_bound; i++)
	{
		for(int j = 1; j <= upper_bound; j++)
		{

			float hx_temp = 0.5f * (gradx[i+1][j]-gradx[i-1][j]);
			float hy_temp = 0.5f * (grady[i][j+1]-grady[i][j-1]);

			hxx += hx_temp * hx_temp;
			hyy += hy_temp * hy_temp;
			hxy += hx_temp * hy_temp;

		}
	}

	A << hxx, hxy, hxy, hyy;

	A *= 1.0/((float)(length-2)*255.0);

	ROS_DEBUG_STREAM("A: " << A.inverse());

	return A.inverse();
}
