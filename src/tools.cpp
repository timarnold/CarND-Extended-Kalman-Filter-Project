#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

double Tools::NormalizedAngle(const double angle)
{
  double normalized_angle = angle;
  while (normalized_angle > M_PI)
  {
    normalized_angle -= 2 * M_PI;
  }
  while (angle < -M_PI)
  {
    normalized_angle += 2 * M_PI;
  }
  if (normalized_angle > M_PI || normalized_angle < -M_PI)
  {
    std::cout << "Found phi outside of expected range: " << angle << std::endl;
  }
  return normalized_angle;
}

VectorXd Tools::CartesianToPolar(const VectorXd &cartesian)
{
  double x = cartesian(0);
  double y = cartesian(1);
  double x_dot = cartesian(2);
  double y_dot = cartesian(3);

  double rho = sqrtf(pow(x, 2.0) + pow(y, 2.0));
  double phi = atan2(y, x);
  double rho_dot = (x * x_dot + y * y_dot) / rho;

  VectorXd polar = VectorXd(3);
  polar << rho, phi, rho_dot;
  return polar;
}

VectorXd Tools::PolarToCartesian(const VectorXd &polar)
{
  double rho = polar(0);
  double phi = polar(1);
  double rho_dot = polar(2);

  double x = rho * cos(phi);
  double y = rho * sin(phi);
  double x_dot = rho_dot * cos(phi);
  double y_dot = rho_dot * sin(phi);

  VectorXd cartesian(4);
  cartesian << x, y, x_dot, y_dot;
  return cartesian;
}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth)
{
  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;

  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  if (estimations.size() != ground_truth.size() || estimations.size() == 0)
  {
    std::cout << "Invalid estimation or ground_truth data" << std::endl;
    return rmse;
  }

  // accumulate squared residuals
  for (unsigned int i = 0; i < estimations.size(); ++i)
  {
    VectorXd residual = estimations[i] - ground_truth[i];

    // coefficient-wise multiplication
    residual = residual.array() * residual.array();
    rmse += residual;
  }

  // calculate the mean
  rmse = rmse / estimations.size();

  // calculate the squared root
  rmse = rmse.array().sqrt();

  // return the result
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd &x_state)
{
  MatrixXd Hj(3, 4);

  // recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // pre-compute a set of terms to avoid repeated calculation
  float c1 = px * px + py * py;
  float c2 = sqrt(c1);
  float c3 = (c1 * c2);

  // check division by zero
  if (fabs(c1) < 0.0001)
  {
    std::cout << "CalculateJacobian () - Error - Division by Zero" << std::endl;
    return Hj;
  }

  //compute the Jacobian matrix
  Hj << (px / c2), (py / c2), 0, 0,
  -(py / c1), (px / c1), 0, 0,
  py * (vx * py - vy * px) / c3, px * (px * vy - py * vx) / c3, px / c2, py / c2;

  return Hj;
}
