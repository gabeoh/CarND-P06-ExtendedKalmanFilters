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
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  // Measurement noise (covariance matrix) - laser
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  // Measurement noise (covariance matrix) - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

  // Measurement matrix - laser
  H_laser_ << 1, 0, 0, 0,
      0, 1, 0, 0;

  // Measurement matrix - radar (Jacobian matrix)
  Hj_ << 0, 0, 0, 0,
      0, 0, 0, 0,
      0, 0, 0, 0;

  // Set the process noises - Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
  noise_ax = 9.0;
  noise_ay = 9.0;


  // Initialize EKF model
  VectorXd x_init(4);
  x_init << 0, 0, 0, 0;
  // State covariance matrix
  MatrixXd P_init(4, 4);
  P_init << 1, 0, 0, 0,
      0, 1, 0, 0,
      0, 0, 1000, 0,
      0, 0, 0, 1000;
  // State transition matrix - unit time transition
  MatrixXd F_init(4, 4);
  F_init << 1, 0, 1, 0,
      0, 1, 0, 1,
      0, 0, 1, 0,
      0, 0, 0, 1;
  // Process covariance matrix
  MatrixXd Q_init(4, 4);
  Q_init << 0, 0, 0, 0,
      0, 0, 0, 0,
      0, 0, 0, 0,
      0, 0, 0, 0;
  // Initialize EKF model with radar measurement models
  ekf_.Init(x_init, P_init, F_init, H_laser_, R_laser_, Q_init);
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() = default;

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
     * Initialize the state ekf_.x_ with the first measurement.
     * Create the covariance matrix.
     */
    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 0, 0, 0, 0;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // Convert radar from polar to cartesian coordinates and initialize state.
      double rho = measurement_pack.raw_measurements_(0);
      double phi = measurement_pack.raw_measurements_(1);
      double rho_dot = measurement_pack.raw_measurements_(2);
      ekf_.x_ << rho * cos(phi),
                 rho * sin(phi),
                 rho_dot * cos(phi),
                 rho_dot * sin(phi);
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // Initialize state with a laser measurement
      ekf_.x_(0) = measurement_pack.raw_measurements_(0);
      ekf_.x_(1) = measurement_pack.raw_measurements_(1);
    }

    // Set measurement timestamp as a previous timestamp
    previous_timestamp_ = measurement_pack.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/
  // Compute elapsed time since the previous measurment, then update the previous timestamp tracking
  double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

  // Compute F and Q based on dt
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;
  double dt2 = dt * dt;
  double dt3 = dt2 * dt;
  double dt4 = dt3 * dt;
  ekf_.Q_ << dt4/4 * noise_ax, 0, dt3/2 * noise_ax, 0,
      0, dt4/4 * noise_ay, 0, dt3/2 * noise_ay,
      dt3/2 * noise_ax, 0, dt2 * noise_ax, 0,
      0, dt3/2 * noise_ay, 0, dt2 * noise_ay;
  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/
  // Use the sensor type to perform the update step.
  // Update the state and covariance matrices.
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Update H_ and R_ as Radar measurements
    try {
      Hj_ = tools.CalculateJacobian(ekf_.x_);
    } catch(exception &e) {
      cout << e.what() << endl;
      return;
    }
    ekf_.H_ = Hj_;
    ekf_.R_ = R_radar_;
    ekf_.Update(measurement_pack.raw_measurements_);
  } else {
    // Update R_ and H_ as Ladar measurements
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
