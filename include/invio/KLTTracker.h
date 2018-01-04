/*
 * KLTTracker.h
 *
 *  Created on: Jan 3, 2018
 *      Author: kevin
 */

#ifndef INVIO_INCLUDE_INVIO_KLTTRACKER_H_
#define INVIO_INCLUDE_INVIO_KLTTRACKER_H_

#include <Frame.h>
#include <Feature.h>

#include <Eigen/Core>

class KLTTracker
{
public:



	KLTTracker();
	virtual ~KLTTracker();

	void findNewFeaturePositions(const Frame& lf, const Frame& cf, const std::vector<Eigen::Vector2f>& previous_feature_positions,
			const std::list<Feature>& estimated_new_feature_positions, std::vector<Eigen::Vector2f>& measured_positions,
			std::vector<Eigen::Matrix2f>& estimated_uncertainty, std::vector<bool>& passed);
};

#endif /* INVIO_INCLUDE_INVIO_KLTTRACKER_H_ */
