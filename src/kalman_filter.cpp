#include <iostream>
#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

//void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
//                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
//  x_ = x_in;
//  P_ = P_in;
//  F_ = F_in;
//  H_ = H_in;
//  R_ = R_in;
//  Q_ = Q_in;
//}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_; // Q is Process Covariance Matrix, no u needed in x_ = F * x_ + u.
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  VectorXd z_pred = H_ * x_; // z_pred is the location we predict
  VectorXd y = z - z_pred;  // z is the measurement vector, we want to compare our predict
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_; // R represents the uncertainty in sensor measurements.
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  // convert current state into polar coordinate
  float ro = sqrt(x_[0] * x_[0] + x_[1] * x_[1]);
  float phi = 0.0; // this will be default case if px is 0
  if (fabs(x_[0]) > 0.0001) {
    phi = atan2(x_[1], x_[0]);
  }
  float ro_dot = 0.0; // make ro_dot 0.0 if too small
  if (fabs(ro) > 0.0001) {
    ro_dot = (x_[0] * x_[2] + x_[1] * x_[3]) / ro;
  }

  VectorXd hx(3);
  hx << ro, phi, ro_dot;

  // prediction error
  VectorXd y = z - hx; // need normalize y(1) to (-pi pi) range ?

  std::cout << "!z_ = " << z(1) << "!!!" << std::endl;
  std::cout << "!!hx_ = " << hx(1) << "!!!" << std::endl;
  std::cout << "!!!y_ = " << y(1) << "!!!" << std::endl;


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
