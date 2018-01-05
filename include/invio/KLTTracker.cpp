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
		if(status.at(i) == 1){
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

Eigen::Matrix2f KLTTracker::estimateUncertainty(const Frame& cf, cv::Point2f mu){

}
