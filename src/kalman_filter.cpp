#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(
                        VectorXd &x_in,
                        MatrixXd &P_in,
                        MatrixXd &F_in,
                        MatrixXd &H_in,
                        MatrixXd &R_laser_in,
                        MatrixXd &R_radar_in,
                        MatrixXd &Q_in)
{
  x_ = x_in;            // State vector
  P_ = P_in;            // State covariance matrix (error in belief)
  F_ = F_in;            // State transition matrix
  H_ = H_in;            // Measurement mapping matrix
  R_laser = R_laser_in; // Measurement covariance matrix (error in measurements) for laser
  R_radar = R_radar_in; // Measurement covariance matrix (error in measurements) for radar
  Q_ = Q_in;            // Process covariance matrix (e.g. random accelerations)
}

void KalmanFilter::Predict()
{
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z)
{
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);

  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();

  MatrixXd S = H_ * P_ * Ht + R_laser;
  MatrixXd Si = S.inverse();

  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  x_ = x_ + (K * y);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z)
{
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);

  // x, y, x_dot, y_dot
  VectorXd cartesian = VectorXd(4);
  cartesian << x_[0], x_[1], x_[2], x_[3];

  // rho, phi, rho_dot
  VectorXd polar = tools_.CartesianToPolar(cartesian);

  VectorXd z_pred = VectorXd(3);
  z_pred << polar[0], polar[1], polar[2];
  VectorXd y = z - z_pred;

  y[1] = tools_.NormalizedAngle(y[1]);

  MatrixXd Hj = tools_.CalculateJacobian(x_);
  MatrixXd Ht = Hj.transpose();

  MatrixXd S = Hj * P_ * Ht + R_radar;
  MatrixXd Si = S.inverse();

  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  x_ = x_ + (K * y);
  P_ = (I - K * Hj) * P_;
}
