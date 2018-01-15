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
			estimated_uncertainty[i] = (this->estimateUncertainty(lf, prev_fts.at(i), cf, new_fts.at(i)));
			measured_positions[i] = Feature::pixel2Metric(cf, new_fts.at(i));
		}
		else
		{
			passed[i] = (false);
			estimated_uncertainty[i] = (Eigen::Matrix2f::Zero());
		}
	}


}


Eigen::Matrix2f KLTTracker::estimateUncertainty(const Frame& lf, cv::Point2f mu_ref, const Frame& cf, cv::Point2f mu)
{
	Eigen::Matrix2f A;
	A << 20, 0, 0, 20;
	return A;
}

/*
 * samples a small region around the feature with its reference to estimate the covariance matrix
 */
Eigen::Matrix2f KLTTracker::estimateUncertaintySampleBased(const Frame& lf, cv::Point2f mu_ref, const Frame& cf, cv::Point2f mu){
	Eigen::Matrix2f A;

	cv::Mat ref;

	//TODO check if feature is too close to the boundaries

	float window_size = 5;

	cv::getRectSubPix(lf.img, cv::Size(window_size, window_size), mu_ref, ref, CV_32F); // subpixel reference patch

	float window_area = window_size*window_size;
	float k = 0.01;

	float sum_rd=0;
	float sum_rd_xx=0;
	float sum_rd_yy=0;
	float sum_rd_xy=0; // = yx

	//compute the gaussian with a 5x5 sample
	for(float du = -10; du <= 10; du+=5)
	{
		for(float dv = -10; dv <= 10; dv+=5)
		{
			cv::Point2f sample_mu = mu + cv::Point2f(du, dv);

			//compute the subpixel image patch for this sample
			cv::Mat sample;

			cv::getRectSubPix(cf.img, cv::Size(window_size, window_size), sample_mu, sample, CV_32F); // subpixel test patch

			//compute the ssd for this patch
			float ssd = 0;
			for(int i = 0; i < window_size; i++){
				for(int j = 0; j < window_size; j++){
					ssd += pow(ref.at<float>(i, j) - sample.at<float>(i, j), 2);
				}
			}

			ssd /= window_area; // normalize

			//ROS_DEBUG_STREAM("ssd: " << ssd);

			float rd = exp(-k*ssd);

			sum_rd += rd;
			sum_rd_xx += rd*du*du;
			sum_rd_yy += rd*dv*dv;
			sum_rd_xy += rd*du*dv;
		}
	}

	//finally construct the covariance matrix

	//ROS_DEBUG_STREAM("sum_rd: " << sum_rd);

	A(0, 0) = sum_rd_xx/sum_rd;
	A(1, 1) = sum_rd_yy/sum_rd;
	A(0, 1) = sum_rd_xy/sum_rd;
	A(1, 0) = A(0, 1);

	//ROS_DEBUG_STREAM("cov: " << A);

	return A;
}
