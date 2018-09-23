#include <iostream>
#include <stdexcept>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() = default;

Tools::~Tools() = default;

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  VectorXd rmse(4);
  // Initialize RMSE to 0
  rmse << 0, 0, 0, 0;

  // Validate input vector sizes
  size_t n = estimations.size();
  if (n == 0 || ground_truth.size() != n) {
    cout << "Invalid input vector sizes" << endl;
    return rmse;
  }

  // Calculate the RMSE here.
  // Accumulate residuals
  for (int i = 0; i < n; ++i) {
    VectorXd residual = estimations[i] - ground_truth[i];
    residual = residual.array() * residual.array();
    rmse += residual;
  }

  // Calculate the mean
  rmse /= n;

  // Calculate the square root
  rmse = rmse.array().sqrt();

  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  // Get state variables
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);
  // Compute intermediate terms
  float rho2 = px*px + py*py;
  // Skip update if rho = 0 (prevent divide by 0)
  if (rho2 == 0)
    throw std::invalid_argument("Cannot compute Jacobian because rho = 0");
  float rho = sqrt(rho2);
  float rho3 = rho2 * rho;

  MatrixXd Hj(3, 4);
  Hj << px / rho, py / rho, 0, 0,
        -py / rho2, px / rho2, 0, 0,
        py*(vx*py - vy*px) / rho3, px*(vy*px - vx*py) / rho3, px/rho, py/rho;
  return Hj;
}
