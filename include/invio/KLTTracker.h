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
#include <vioParams.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/video.hpp"

#include <Eigen/Core>

class KLTTracker
{
public:

	struct Pyramid{
		struct PyramidLevel{
			struct Pixel{
				bool set;
				float value;
			};

			Pixel* image = NULL;
			int rows, cols;

			PyramidLevel(){

			}

			PyramidLevel(const int _rows, const int _cols){
				rows = _rows;
				cols = _cols;

				image = new Pixel[rows*cols];
			}

		};

		cv::Mat base_img;
		int level_count = 0;
		std::vector<PyramidLevel> levels;

		Pyramid(const int num_levels, cv::Mat image_ptr){
			base_img = image_ptr;
			level_count = num_levels;
			//levels = new PyramidLevel[level_count];
			levels.resize(num_levels);

			ROS_INFO_STREAM(base_img.rows);

			for(int i = 1; i <= level_count; i++)
			{
				if(i == 1){
					ROS_ASSERT(base_img.rows % 2 == 0 && base_img.cols % 2 == 0); // assert that the base image is of a divisible size
					levels[i-1] = PyramidLevel(base_img.rows/2, base_img.cols/2);
				}
				else{
					ROS_ASSERT(levels[i-2].rows % 2 == 0 && levels[i-2].cols % 2 == 0); // assert that the last image is of a divisible size
					levels[i-1] = PyramidLevel(levels[i-2].rows/2, levels[i-2].cols/2);
				}
			}
		}

		~Pyramid(){
			for(auto e : levels){
				delete[] e.image;
			}
		}

	};

	KLTTracker();
	virtual ~KLTTracker();

	void findNewFeaturePositions(const Frame& lf, const Frame& cf, const std::vector<Eigen::Vector2f>& previous_feature_positions,
			const std::list<Feature>& estimated_new_feature_positions, std::vector<Eigen::Vector2f>& measured_positions,
			std::vector<Eigen::Matrix2f>& estimated_uncertainty, std::vector<bool>& passed);

	void findNewFeaturePositionsOpenCV(const Frame& lf, const Frame& cf, const std::vector<Eigen::Vector2f>& previous_feature_positions,
				const std::list<Feature>& estimated_new_feature_positions, std::vector<Eigen::Vector2f>& measured_positions,
				std::vector<Eigen::Matrix2f>& estimated_uncertainty, std::vector<bool>& passed);
};

#endif /* INVIO_INCLUDE_INVIO_KLTTRACKER_H_ */
