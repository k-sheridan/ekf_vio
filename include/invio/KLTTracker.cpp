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
 */
void KLTTracker::findNewFeaturePositions(Frame& lf, Frame& cf, std::vector<Eigen::Vector2d>& previous_feature_positions,
			std::list<Feature>& estimated_new_feature_positions, std::vector<Eigen::Vector2d>& measured_positions,
			std::vector<Eigen::Matrix2d>& estimated_uncertainty, std::vector<bool>& passed)
{

}
