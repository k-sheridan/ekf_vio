/*
 * vioDraw.cpp
 *
 *  Created on: Jan 16, 2017
 *      Author: kevin
 */


#include "vio.h"

cv::Mat drawEpiLines(cv::Matx33f F, cv::Point2f pt, cv::Matx33f tK, cv::Mat img)
{
	cv::Matx31f u;

	u(0) = pt.x;
	u(1) = pt.y;
	//u(2) = 1.0;
	/*u = tK * u;
			u(0) /= u(2);
			u(1) /= u(2);*/

	// Draw the epipolar lines
	std::vector<cv::Vec3f> lines1;
	std::vector<cv::Point2f> ptt;
	ptt.push_back(cv::Point2f(u(0), u(1)));
	cv::computeCorrespondEpilines(ptt, 2, F, lines1);

	//ROS_DEBUG_STREAM("abc: " << lines1[0]);

	cv::Matx31f pt1, pt2;
	pt1(0) = -2;
	pt1(1) = -(lines1[0][2] + lines1[0][0]*-2)/lines1[0][1];
	pt1(2) = 1.0;
	pt1 = tK * pt1;

	pt2(0) = 2;
	pt2(1) = -(lines1[0][2]+lines1[0][0]*2)/lines1[0][1];
	pt2(2) = 1.0;
	pt2 = tK * pt2;

	cv::line(img,
			cv::Point(pt1(0), pt1(1)),
			cv::Point(pt2(0), pt2(1)),
			cv::Scalar(rand()%255, rand()%255, rand()%255));

	return img;
}

/*cv::Vec3b HSVtoBGR(const cv::Vec3f& hsv)
{
    cv::Mat_<cv::Vec3f> hsvMat(hsv);
    cv::Mat_<cv::Vec3f> bgrMat;

    //ROS_DEBUG_STREAM("hue: " << hsv(0));

    cv::cvtColor(hsvMat, bgrMat, CV_HSV2BGR);

    //bgrMat *= 255; // Upscale after conversion

    // Conversion to Vec3b is handled by OpenCV, no need to static_cast
    return bgrMat(0);
}*/


/*! \brief Convert HSV to RGB color space

  Converts a given set of HSV values `h', `s', `v' into RGB
  coordinates. The output RGB values are in the range [0, 1], and
  the input HSV values are in the ranges h = [0, 360], and s, v =
  [0, 1], respectively.

  \param fR Red component, used as output, range: [0, 1]
  \param fG Green component, used as output, range: [0, 1]
  \param fB Blue component, used as output, range: [0, 1]
  \param fH Hue component, used as input, range: [0, 360]
  \param fS Hue component, used as input, range: [0, 1]
  \param fV Hue component, used as input, range: [0, 1]

 */
cv::Scalar HSVtoBGR(float fH, float fS, float fV) {
	float fR, fG, fB;

	float fC = fV * fS; // Chroma
	float fHPrime = fmod(fH / 60.0, 6);
	float fX = fC * (1 - fabs(fmod(fHPrime, 2) - 1));
	float fM = fV - fC;

	if(0 <= fHPrime && fHPrime < 1) {
		fR = fC;
		fG = fX;
		fB = 0;
	} else if(1 <= fHPrime && fHPrime < 2) {
		fR = fX;
		fG = fC;
		fB = 0;
	} else if(2 <= fHPrime && fHPrime < 3) {
		fR = 0;
		fG = fC;
		fB = fX;
	} else if(3 <= fHPrime && fHPrime < 4) {
		fR = 0;
		fG = fX;
		fB = fC;
	} else if(4 <= fHPrime && fHPrime < 5) {
		fR = fX;
		fG = 0;
		fB = fC;
	} else if(5 <= fHPrime && fHPrime < 6) {
		fR = fC;
		fG = 0;
		fB = fX;
	} else {
		fR = 0;
		fG = 0;
		fB = 0;
	}

	fR += fM;
	fG += fM;
	fB += fM;

	return cv::Scalar(fB*255, fG*255, fR*255);
}

void VIO::drawKeyFrames()
{
	cv::Mat img1, img2;

	img1 = currentFrame().image;

	if(frameBuffer.at(1).isFrameSet())
	{
		img2 = frameBuffer.at(1).image;
	}

	cv::cvtColor(img1, img1, CV_GRAY2BGR);
	cv::cvtColor(img2, img2, CV_GRAY2BGR);

	//ROS_DEBUG_STREAM("test: " << currentFrame().features.at(0).point->observations.back()->frame);

	for(auto& e : currentFrame().features)
	{
		cv::drawMarker(img1, e.original_pxl, cv::Scalar(0, 255, 0), cv::MARKER_SQUARE);
		//ROS_DEBUG_STREAM("this feature's point info: obs count: " << e.point->observations.size() << " status: " << e.point->status);

		if(frameBuffer.at(1).isFrameSet())
		{
			if(e.point->observations.size() > 1)
			{
				//ROS_DEBUG_STREAM("plotting: " << e.point->observations.at(1)->original_pxl);
				cv::drawMarker(img2, e.point->observations.at(1)->original_pxl, cv::Scalar(0, 255, 0), cv::MARKER_SQUARE);
				cv::drawMarker(img1, e.original_pxl, cv::Scalar(255, 255, 0), cv::MARKER_SQUARE);

				//ROS_DEBUG_STREAM("frame link: " << e.point->observations.at(0)->frame);
				//ROS_DEBUG_STREAM("test2: " << currentFrame().features.at(0).frame);
			}
			else
			{
				//ROS_DEBUG_STREAM("frame link outside: " << e.point->observations.at(0)->frame);
			}
		}
	}

	//ROS_DEBUG_STREAM("test3: " << currentFrame().features.at(0).frame);

	if(frameBuffer.at(1).isFrameSet())
	{
		cv::Mat final;
		cv::vconcat(img1, img2, final);
		cv::imshow("debug", final);
		cv::waitKey(1);
	}
	else{
		cv::imshow("debug", img1);
		cv::waitKey(1);
	}
}


// LEGACY - VERSION 2

template <typename T1, typename T2>
void VIO::drawEpipolarLines(const std::string& title, const cv::Matx<T1,3,3> F,
		const cv::Mat& img1, const cv::Mat& img2,
		const std::vector<cv::Point_<T2> > points1,
		const std::vector<cv::Point_<T2> > points2,
		const float inlierDistance)
{
	CV_Assert(img1.size() == img2.size() && img1.type() == img2.type());
	cv::Mat outImg(img1.rows, img1.cols*2, CV_8UC3);
	cv::Rect rect1(0,0, img1.cols, img1.rows);
	cv::Rect rect2(img1.cols, 0, img1.cols, img1.rows);
	/*
	 * Allow color drawing
	 */
	if (img1.type() == CV_8U)
	{
		cv::cvtColor(img1, outImg(rect1), CV_GRAY2BGR);
		cv::cvtColor(img2, outImg(rect2), CV_GRAY2BGR);
	}
	else
	{
		img1.copyTo(outImg(rect1));
		img2.copyTo(outImg(rect2));
	}
	std::vector<cv::Vec<T2,3> > epilines1, epilines2;
	cv::computeCorrespondEpilines(points1, 1, F, epilines1); //Index starts with 1
	cv::computeCorrespondEpilines(points2, 2, F, epilines2);

	CV_Assert(points1.size() == points2.size() &&
			points2.size() == epilines1.size() &&
			epilines1.size() == epilines2.size());

	cv::RNG rng(0);
	for(size_t i=0; i<points1.size(); i++)
	{
		if(inlierDistance > 0)
		{
			if(distancePointLine(points1[i], epilines2[i]) > inlierDistance ||
					distancePointLine(points2[i], epilines1[i]) > inlierDistance)
			{
				//The point match is no inlier
				continue;
			}
		}
		/*
		 * Epipolar lines of the 1st point set are drawn in the 2nd image and vice-versa
		 */
		cv::Scalar color(rng(256),rng(256),rng(256));

		cv::line(outImg(rect2),
				cv::Point(0,-epilines1[i][2]/epilines1[i][1]),
				cv::Point(img1.cols,-(epilines1[i][2]+epilines1[i][0]*img1.cols)/epilines1[i][1]),
				color);
		cv::circle(outImg(rect1), points1[i], 3, color, -1, CV_AA);

		cv::line(outImg(rect1),
				cv::Point(0,-epilines2[i][2]/epilines2[i][1]),
				cv::Point(img2.cols,-(epilines2[i][2]+epilines2[i][0]*img2.cols)/epilines2[i][1]),
				color);
		cv::circle(outImg(rect2), points2[i], 3, color, -1, CV_AA);
	}
	cv::imshow(title, outImg);
	cv::waitKey(1);
}

template <typename T>
float VIO::distancePointLine(const cv::Point_<T> point, const cv::Vec<T,3>& line)
{
	//Line is given as a*x + b*y + c = 0
	return abs(line(0)*point.x + line(1)*point.y + line(2)) / sqrt(line(0)*line(0)+line(1)*line(1));
}

/*
void VIO::viewMatches(std::vector<VIOFeature2D> ft1, std::vector<VIOFeature2D> ft2, Frame f1, Frame f2, std::vector<cv::Point2f> pt1_new, std::vector<cv::Point2f> pt2_new)
{
	cv::Mat img1 = f1.image;
	cv::Mat img2 = f2.image;

	cv::cvtColor(img1, img1, CV_GRAY2BGR);
	cv::cvtColor(img2, img2, CV_GRAY2BGR);

	cv::Matx33f tK = currentFrame().K;

	for(int i = 0; i < f1.features.size(); i++)
	{
		cv::Matx31f u;
		u(0) = f1.features.at(i).getUndistorted(true).x;
		u(1) = f1.features.at(i).getUndistorted(true).y;
		u(2) = 1.0;
		u = tK * u;

		cv::drawMarker(img1, cv::Point2f(u(0) / u(2), u(1) / u(2)), cv::Scalar(255, 0, 0), cv::MARKER_DIAMOND, 4);
	}

	for(int i = 0; i < f2.features.size(); i++)
	{
		cv::Matx31f u;
		u(0) = f2.features.at(i).getUndistorted(true).x;
		u(1) = f2.features.at(i).getUndistorted(true).y;
		u(2) = 1.0;
		u = tK * u;

		cv::drawMarker(img2, cv::Point2f(u(0) / u(2), u(1) / u(2)), cv::Scalar(255, 0, 0), cv::MARKER_DIAMOND, 4);
	}

	for(int i = 0; i < ft1.size(); i++)
	{
		cv::drawMarker(img1, ft1.at(i).getFeaturePosition(), cv::Scalar(0, 255, 0), cv::MARKER_SQUARE, 10);
	}

	for(int i = 0; i < ft2.size(); i++)
	{
		cv::drawMarker(img2, ft2.at(i).getFeaturePosition(), cv::Scalar(0, 255, 0), cv::MARKER_SQUARE, 10);
	}

	for(int i = 0; i < pt1_new.size(); i++)
	{
		cv::Matx31f u;
		u(0) = pt1_new.at(i).x;
		u(1) = pt1_new.at(i).y;
		u(2) = 1.0;
		u = tK * u;

		cv::drawMarker(img1, cv::Point2f(u(0) / u(2), u(1) / u(2)), cv::Scalar(0, 0, 255), cv::MARKER_TRIANGLE_UP, 8);
	}

	for(int i = 0; i < pt2_new.size(); i++)
	{
		cv::Matx31f u;
		u(0) = pt2_new.at(i).x;
		u(1) = pt2_new.at(i).y;
		u(2) = 1.0;
		u = tK * u;

		cv::drawMarker(img2, cv::Point2f(u(0) / u(2), u(1) / u(2)), cv::Scalar(0, 0, 255), cv::MARKER_TRIANGLE_UP, 8);
	}

	//img1 = this->reproject3dPoints(img1, f1.state);
	//img2 = this->reproject3dPoints(img2, f2.state);

	cv::Mat img;
	cv::vconcat(img2, img1, img);

	cv::imshow("matches", img);
	cv::waitKey(30);
}



/*
 * shows cv::Mat
 */
void VIO::viewImage(cv::Mat img){
	cv::imshow("test", img);
	cv::waitKey(30);
}

/*
 * draws frame with its features
 */
/*
void VIO::viewImage(Frame frame){
	cv::Mat img;
	cv::drawKeypoints(frame.image, frame.getKeyPointVectorFromFeatures(), img, cv::Scalar(0, 0, 255));
	cv::drawKeypoints(img, frame.getUndistortedKeyPointVectorFromFeatures(), img, cv::Scalar(255, 0, 0));
	this->viewImage(img);

}
 */

