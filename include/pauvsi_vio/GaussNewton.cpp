/*
 * vioGaussNewton.cpp
 *
 *  Created on: Jan 21, 2017
 *      Author: kevin
 */

#include "vio.h"

void exponential_map(const cv::Mat &v, cv::Mat dt, cv::Mat dR)
{
  double vx = v.at<double>(0,0);
  double vy = v.at<double>(1,0);
  double vz = v.at<double>(2,0);
  double vtux = v.at<double>(3,0);
  double vtuy = v.at<double>(4,0);
  double vtuz = v.at<double>(5,0);
  cv::Mat tu = (cv::Mat_<double>(3,1) << vtux, vtuy, vtuz); // theta u
  cv::Rodrigues(tu, dR);

  double theta = sqrt(tu.dot(tu));
  double sinc = (fabs(theta) < 1.0e-8) ? 1.0 : sin(theta) / theta;
  double mcosc = (fabs(theta) < 2.5e-4) ? 0.5 : (1.-cos(theta)) / theta / theta;
  double msinc = (fabs(theta) < 2.5e-4) ? (1./6.) : (1.-sin(theta)/theta) / theta / theta;

  dt.at<double>(0,0) = vx*(sinc + vtux*vtux*msinc)
        + vy*(vtux*vtuy*msinc - vtuz*mcosc)
        + vz*(vtux*vtuz*msinc + vtuy*mcosc);

  dt.at<double>(1,0) = vx*(vtux*vtuy*msinc + vtuz*mcosc)
        + vy*(sinc + vtuy*vtuy*msinc)
        + vz*(vtuy*vtuz*msinc - vtux*mcosc);

  dt.at<double>(2,0) = vx*(vtux*vtuz*msinc - vtuy*mcosc)
        + vy*(vtuy*vtuz*msinc + vtux*mcosc)
        + vz*(sinc + vtuz*vtuz*msinc);
}

//! [Estimation function]
void VIO::pose_gauss_newton(const std::vector< cv::Point3d > &wX,
                       const std::vector< cv::Point2d > &x,
                       cv::Mat &ctw, cv::Mat &cRw)
//! [Estimation function]
{
  //! [Gauss-Newton]
  int npoints = (int)wX.size();
  cv::Mat J(2*npoints, 6, CV_64F);
  cv::Mat cX;
  double lambda = 0.25;
  cv::Mat err, sd(2*npoints, 1, CV_64F), s(2*npoints, 1, CV_64F);
  cv::Mat xq(npoints*2, 1, CV_64F);
  // From input vector x = (x, y) we create a column vector xn = (x, y)^T to ease computation of e_q
  cv::Mat xn(npoints*2, 1, CV_64F);
  //vpHomogeneousMatrix cTw_ = cTw;
  double residual=0, residual_prev;
  cv::Mat Jp;

  // From input vector x = (x, y, 1)^T we create a new one xn = (x, y)^T to ease computation of e_q
  for (int i = 0; i < x.size(); i ++) {
    xn.at<double>(i*2,0)   = x[i].x; // x
    xn.at<double>(i*2+1,0) = x[i].y; // y
  }

  int iteration = 0;
  // Iterative Gauss-Newton minimization loop
  do {
	iteration++;

    for (int i = 0; i < npoints; i++) {
      cX = cRw * cv::Mat(wX[i]) + ctw;                      // Update cX, cY, cZ
      // Update x(q)
      xq.at<double>(i*2,0)   = cX.at<double>(0,0) / cX.at<double>(2,0); // x(q) = cX/cZ
      xq.at<double>(i*2+1,0) = cX.at<double>(1,0) / cX.at<double>(2,0); // y(q) = cY/cZ

      // Update J using equation (11)
      J.at<double>(i*2,0) = -1/cX.at<double>(2,0);          // -1/cZ
      J.at<double>(i*2,1) = 0;
      J.at<double>(i*2,2) = x[i].x / cX.at<double>(2,0);    // x/cZ
      J.at<double>(i*2,3) = x[i].x * x[i].y;                // xy
      J.at<double>(i*2,4) = -(1 + x[i].x * x[i].x);         // -(1+x^2)
      J.at<double>(i*2,5) = x[i].y;                         // y

      J.at<double>(i*2+1,0) = 0;
      J.at<double>(i*2+1,1) = -1/cX.at<double>(2,0);        // -1/cZ
      J.at<double>(i*2+1,2) = x[i].y / cX.at<double>(2,0);  // y/cZ
      J.at<double>(i*2+1,3) = 1 + x[i].y * x[i].y;          // 1+y^2
      J.at<double>(i*2+1,4) = -x[i].x * x[i].y;             // -xy
      J.at<double>(i*2+1,5) = -x[i].y;                      // -x
    }

    cv::Mat e_q = xq - xn;                                  // Equation (7)

    cv::Mat Jp = J.inv(cv::DECOMP_SVD);                     // Compute pseudo inverse of the Jacobian
    cv::Mat dq = -lambda * Jp * e_q;                        // Equation (10)

    cv::Mat dctw(3,1,CV_64F), dcRw(3,3,CV_64F);
    exponential_map(dq, dctw, dcRw);

    cRw = dcRw.t() * cRw;                                   // Update the pose
    ctw = dcRw.t() * (ctw - dctw);

    residual_prev = residual;                               // Memorize previous residual
    residual = e_q.dot(e_q);                                // Compute the actual residual

  } while (iteration <= MAX_GN_ITERS && fabs(residual - residual_prev) > 0);
  //! [Gauss-Newton]
}


