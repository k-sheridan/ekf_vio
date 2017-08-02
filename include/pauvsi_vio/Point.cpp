/*
 * Point.cpp
 *
 *  Created on: Jan 30, 2017
 *      Author: pauvsi
 */


#include "Point.h"

Point::Point()
{
	deleted = false;
	this->sigma = DEFAULT_POINT_STARTING_ERROR;
	this->theMap = NULL;
	this->immature = true;
	//this->thisPoint = 0;

}

Point::Point(Feature* ft){
	_observations.push_front(ft); // add this observation to the deque
	deleted = false;
	this->sigma = DEFAULT_POINT_STARTING_ERROR;
	this->theMap = NULL;
	this->immature = true;
	//this->thisPoint = 0;
}

Point::Point(Feature* ft, std::list<Point>::iterator _thisPoint, std::list<Point>* _map)
{
	_observations.push_front(ft); // add this observation to the deque
	deleted = false;

	ROS_ASSERT(_map != NULL);

	this->immature = true;

	this->sigma = DEFAULT_POINT_STARTING_ERROR;
	this->theMap = _map;
	this->thisPoint = _thisPoint;
}

void Point::addObservation(Feature* ft)
{
	/*
	if(observations.size() >= 1)
	{
		ROS_DEBUG_STREAM("this feature frame " << ft->frame << " last frame: " << observations.at(0)->frame);
	}*/

	_observations.push_front(ft);

	/*
	if(observations.size() > 1)
	{
		ROS_DEBUG_STREAM("*most recent obs frame: " << observations.front()->frame << " last obs frame: " << observations.back()->frame);
	}*/
}


void Point::safelyDeletePoint()
{
	ROS_DEBUG_STREAM("deleting point with " << this->observations().size() << " obs");
	ROS_ASSERT(this->observations().size() <= 100);


	for(auto& e : this->observations())
	{
		ROS_DEBUG_STREAM("check feature pos for deleted memory" << e->px);
		e->setPoint(NULL); // null the point reference

		/*
		if(e->frame->finalFrame) // this is the last frame in the buffer do not search past this point due to potential dangling pointer
		{
			ROS_DEBUG("breaking out of point deletion dereferencer because the final frame has been passed");
			break;
		}*/
	}

	ROS_DEBUG_STREAM("point deleting itself");

	deleted = true;

	//peace out delete my self
	// we had a good run
	this->getMap()->erase(this->getPointIterator());

	ROS_DEBUG("point deleted");
}




/*
 * adapted from SVO, Forster et al
 * this performs structure only bundle adjustment on this point using all keyframes still in the buffer
 *
 * This function uses the gauss newton optimization method to find the optimal 3d point for a set of keyframes
 */
void Point::SBA(int iterations)
{
	Eigen::Vector3d old_point = this->pos;
	Eigen::Vector3d original = this->pos;
	double chi2 = 0.0;
	Eigen::Matrix3d A;
	Eigen::Vector3d b;

	bool failed = false;
	int failIter = 0;

	//this bock assumes that there are no observations which point to an out of buffer frame
	std::vector<Feature*> vertices;

	for(auto e : this->_observations)
	{
		if(e->getParentFrame()->isKeyframe())
		{
			vertices.push_back(e);
			ROS_DEBUG_STREAM("added a keyframe to the SBA problem");
		}
		/*if(e->frame->finalFrame) // this is the last frame in the buffer do not search past this point due to potential dangling pointer
		{
			ROS_DEBUG("breaking out of keyframe search because the final frame has been passed");
			break;
		}*/
	}

	if(vertices.size() < 2){
		ROS_DEBUG_STREAM("NOT ENOUGH KEYFRAMES FOR SBA PROBLEM TO SOLVE");
		return;
	}

	ROS_DEBUG_STREAM("point before: " << this->pos);


	for(size_t i = 0; i < iterations; i++)
	{
		A.setZero();
		b.setZero();
		double new_chi2 = 0.0;

		// compute residuals
		for(auto it = vertices.begin(); it != vertices.end(); ++it)
		{
			Matrix23d J;
			const Eigen::Vector3d p_in_f((*it)->getParentFrame()->getPose_inv() * this->pos); // the 3d point projected into this keyframe's coordinate system

			Point::jacobian_xyz2uv(p_in_f, (*it)->getParentFrame()->getPose_inv().rotationMatrix(), J); // this gets the jacobian of the projection function

			const Eigen::Vector2d e((*it)->getMetricPixel() - Point::toMetricPixel(p_in_f)); // this is the metric pixel error of this point

			new_chi2 += e.squaredNorm(); // add the pixel error to chi

#if SUPER_DEBUG
			ROS_DEBUG_STREAM("error at vertex " << e.squaredNorm());
#endif

			A.noalias() += J.transpose() * J; //add this observation to this iteration's problem
			b.noalias() -= J.transpose() * e;
		}


		// solve linear system
		const Eigen::Vector3d dp(A.ldlt().solve(b));

		// check if error increased
		if((i > 0 && new_chi2 > chi2) || (bool) std::isnan((double)dp[0]))
		{
			failed = true;
			failIter = i;
			ROS_DEBUG_STREAM("it " << i << " FAILURE  new_chi2 = " << new_chi2);
			this->pos = old_point; // roll-back
			break;
		}

		// update the model
		Eigen::Vector3d new_point = this->pos + dp;
		old_point = this->pos;
		this->pos = new_point;
		chi2 = new_chi2;

		//ROS_DEBUG_STREAM("point between " << new_point);

		ROS_DEBUG_STREAM("it " << i << " Success  new_chi2 = " << new_chi2 << " norm(b) = " << b.norm());

		// stop when converged
		if(dp.norm() <= EPS_SBA)
			break;
	}

	//compute the depth to the current frame

	double depth = (vertices.back()->getParentFrame()->getPose_inv() * this->pos).z();

	if(!(failed && failIter < 2) && depth > MIN_POINT_Z && depth < MAX_POINT_Z && chi2 != 0.0)
	{
		ROS_DEBUG_STREAM("GOOD POINT DEPTH: "  << depth);
		this->sigma = chi2 / vertices.size();
		ROS_DEBUG_STREAM("new sigma: " << this->sigma);
	}
	else
	{
		ROS_WARN_STREAM("Point failed to converge in a valid position. depth: " << depth);
		this->pos = original;
	}


	ROS_DEBUG_STREAM("point after: " << this->pos);

}



