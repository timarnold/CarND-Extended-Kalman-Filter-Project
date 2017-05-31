#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF()
{
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // measurement covariance matrix - laser
  R_laser_ = MatrixXd(2, 2);
  R_laser_ <<
  0.0225, 0,
  0, 0.0225;

  // measurement mapping matrix - laser
  H_laser_ = MatrixXd(2, 4);
  H_laser_ <<
  1, 0, 0, 0,
  0, 1, 0, 0;

  // measurement covariance matrix - radar
  R_radar_ = MatrixXd(3, 3);
  R_radar_ <<
  0.09, 0, 0,
  0, 0.0009, 0,
  0, 0, 0.09;

  Hj_ = MatrixXd(3, 4); // used for RADAR

  // Process noise to be used in Q matrix (stochastic-based motion noise)
  noise_ax = 9;
  noise_ay = 9;

  // F, the transition matrix
  ekf_.F_ = MatrixXd(4, 4);

  // H, the measurement matrix
  ekf_.H_ = H_laser_;

  // R, the measurement covariance matrix
  ekf_.R_laser = R_laser_;
  ekf_.R_radar = R_radar_;

  // P, the initial state covariance matrix
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_ <<
  1, 0, 0, 0,
  0, 1, 0, 0,
  0, 0, 1000, 0,
  0, 0, 0, 1000;
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack)
{

  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_)
  {
    // first measurement
    cout << "EKF: " << endl;

    ekf_.x_ = VectorXd(4);

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
    {
      VectorXd polar = VectorXd(3);
      polar <<
      measurement_pack.raw_measurements_[0],
      measurement_pack.raw_measurements_[1],
      measurement_pack.raw_measurements_[2];
      ekf_.x_ = tools.PolarToCartesian(polar);
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER)
    {
      ekf_.x_ <<
      measurement_pack.raw_measurements_[0],
      measurement_pack.raw_measurements_[1],
      0,
      0;
    }

    previous_timestamp_ = measurement_pack.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

  ekf_.F_ <<
  1, 0, dt, 0,
  0, 1, 0, dt,
  0, 0, 1, 0,
  0, 0, 0, 1;

  double dt4 = pow(dt, 4) / 4;
  double dt3 = pow(dt, 3) / 2;
  double dt2 = pow(dt, 2);
  double s_ax = noise_ax;
  double s_ay = noise_ay;

  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ <<
  dt4 * s_ax, 0, dt3 * s_ax, 0,
  0, dt4 * s_ay, 0, dt3 * s_ay,
  dt3 * s_ax, 0, dt2 * s_ax, 0,
  0, dt3 * s_ay, 0, dt2 * s_ay;

  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
  {
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  }
  else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER)
  {
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  //  cout << "x_ = " << ekf_.x_ << endl;
  //  cout << "P_ = " << ekf_.P_ << endl;
}
