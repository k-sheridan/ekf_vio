/*
 * DepthSolver.h
 *
 *  Created on: Sep 17, 2017
 *      Author: kevin
 */

#ifndef INVIO_INCLUDE_INVIO_DEPTHSOLVER_H_
#define INVIO_INCLUDE_INVIO_DEPTHSOLVER_H_

#include <Point.h>
#include <vioParams.h>
#include <Frame.h>
#include <Feature.h>

class DepthSolver {
public:
	DepthSolver();
	virtual ~DepthSolver();

	void updatePointDepths(Frame& f);

	bool solveAndUpdatePointDepth(Point* pt, Sophus::SE3d cf_2_rf, Eigen::Vector3d curr_ft);

};

#endif /* INVIO_INCLUDE_INVIO_DEPTHSOLVER_H_ */
