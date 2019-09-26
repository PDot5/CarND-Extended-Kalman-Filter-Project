#include "kalman_filter.h"
#include <iostream>
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in)
{
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict()
{
  /**
   * TODO: predict the state
   */
  x_ = F_ * x_;                       // predict the new mean
  P_ = F_ * P_ * F_.transpose() + Q_; // predict new covariance
}

void KalmanFilter::Update(const VectorXd &z)
{
  /**
   * TODO: update the state by using Kalman Filter equations
   */

  cout << "KalmanFilter::Update()" << endl;
  VectorXd y = z - (H_ * x_);
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd K = (P_ * Ht) * S.inverse();

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z)
{
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
  cout << "KalmanFilter::UpdateEKF()" << endl;
  // VectorXd h_ (x) = h(mu) + Jacobian * (x - mu)
  cout << "H_: " << H_ << endl;

  VectorXd z_pred = hx_;
  VectorXd y = z - z_pred;
  // if phi is not in the range (-pi, pi), put it in that range

  bool in_pi = false;
  while (in_pi == false)
  {
    if (y(1) > 3.14159)
    {
      cout << "phi > pi" << endl;
      y(1) = y(1) - 6.2831;
    }
    else if (y(1) < -3.14159)
    {
      cout << "phi < -pi" << endl;
      y(1) = y(1) + 6.2831;
    }
    else
    {
      in_pi = true;
    }
  }
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

// References:
// https://www.haidynmcleod.com/extended-kalman-filter
// https://github.com/jessicayung/self-driving-car-nd
// https://medium.com/@serrano_223/extended-kalman-filters-for-dummies-4168c68e2117
// https://github.com/Heych88/udacity-sdcnd-extended-kalman-filter
// https://cmake.org/install/
// https://tuannguyen68.gitbooks.io/learning-cmake-a-beginner-s-guide/content/chap1/chap1.html
// https://www.youtube.com/watch?v=TpQv0k2ZQjo
// https://www.youtube.com/watch?v=TpQv0k2ZQjo