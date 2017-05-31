#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

class Tools
{
public:
  /**
  * Constructor.
  */
  Tools();

  /**
  * Destructor.
  */
  virtual ~Tools();

  /**
  * A helper method to calculate RMSE.
  */
  VectorXd CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth);

  /**
   * Convert cartesian state from cartesian to polar coordinates.
  */
  MatrixXd CalculateJacobian(const VectorXd &x_state);

  /**
  * Convert radar measurement from polar to cartesian coordinates.
  */
  VectorXd CartesianToPolar(const VectorXd &cartesian);

  VectorXd PolarToCartesian(const VectorXd &polar);

  double NormalizedAngle(const double angle);


};

#endif /* TOOLS_H_ */
