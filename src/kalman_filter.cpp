#include "kalman_filter.h"

#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() = default;

KalmanFilter::~KalmanFilter() = default;

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &R_radar_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  R_radar_ = R_radar_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  // Predict the state
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ *  Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  // Update the state by using Kalman Filter equations
  VectorXd y = z - H_ * x_;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = P_ * Ht * Si;
  x_ = x_ + K * y;
  MatrixXd I = MatrixXd::Identity(P_.rows(), P_.cols());
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateRadar(const VectorXd &z, const MatrixXd &Hj) {
  // update the state by using Extended Kalman Filter equations
  VectorXd y = CalculateRadarPredictionError(z);
  MatrixXd Ht = Hj.transpose();
  MatrixXd S = Hj * P_ * Ht + R_radar_;
  MatrixXd Si = S.inverse();
  MatrixXd K = P_ * Ht * Si;
  x_ = x_ + K * y;
  MatrixXd I = MatrixXd::Identity(P_.rows(), P_.cols());
  P_ = (I - K * Hj) * P_;
}

Eigen::MatrixXd KalmanFilter::CalculateRadarPredictionError(const Eigen::VectorXd &z) {
  double px = x_(0);
  double py = x_(1);
  double vx = x_(2);
  double vy = x_(3);
  // Compute intermediate terms
  double rho = sqrt(px*px + py*py);
  // Skip update if rho = 0 (prevent divide by 0)
  if (rho == 0)
    throw std::invalid_argument("Cannot `compute Radar Prediction Error because rho = 0");

  VectorXd hx(3);
  hx << rho, atan2(py, px), (px*vx + py*vy) / rho;
  VectorXd y = z - hx;
  // Ensure phi, y(1), value is [-PI, +PI)
  if (!(y(1) >= -M_PI && y(1) < M_PI)) {
    y(1) = fmod(y(1) + M_PI, 2*M_PI) - M_PI;
  }
  return y;
}