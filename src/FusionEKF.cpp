#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
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

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  /**
   * TODO: Finish initializing the FusionEKF.
   * TODO: Set the process and measurement noises
   */
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;
  
  Hj_ << 0, 0, 0, 0,
          0, 0, 0, 0,
          0, 0, 0, 0;

  // measurement noises
  noise_ax_ = 9.0;
  noise_ay_ = 9.0;
}


/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  if (!is_initialized_) {
    /**
     * TODO: Initialize the state ekf_.x_ with the first measurement.
     * TODO: Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */
    
    // state covariance matrix (initial velocity is unknown, hence the level of uncertainty is high)
    MatrixXd P = MatrixXd(4, 4);
    P << 1, 0,    0,    0,
         0, 1,    0,    0,
         0, 0, 1000,    0,
         0, 0,    0, 1000;
    
    MatrixXd Q = MatrixXd(4,4);
    MatrixXd F = MatrixXd(4,4);
    
    P << 1, 0, 0, 0,
         0, 1, 0, 0,
         0, 0, 1, 0,
         0, 0, 0, 1;    

    // first measurement
    cout << "EKF: " << endl;
    VectorXd x = VectorXd(4);
    x << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // TODO: Convert radar from polar to cartesian coordinates 
      //         and initialize state.
      double rho     = measurement_pack.raw_measurements_[0];
      double phi     = measurement_pack.raw_measurements_[1];
      double px      = rho * cos(phi);
      double py      = rho * sin(phi);
      
      x << px, py, 0, 0;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // TODO: Initialize state.
      x << measurement_pack.raw_measurements_[0],
           measurement_pack.raw_measurements_[1],
                                             0.0,
                                             0.0;
    }
    
    previous_timestamp_ = measurement_pack.timestamp_;
    cout << "initializing" << endl;
    ekf_.Init(x, P, F, H_laser_, R_laser_, R_radar_, Q);
    cout << "done: initialized!" << endl;
    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /**
   * Prediction
   */

  /**
   * TODO: Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * TODO: Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
  previous_timestamp_ = measurement_pack.timestamp_;

  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 0, dt, 0,
       0, 1, 0, dt,
       0, 0, 1, 0,
       0, 0, 0, 1;  
  
  double dt2 = dt * dt;
  double dt3 = dt2 * dt;
  double dt4 = dt3 * dt;
  
  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ << dt4/4*noise_ax_, 0, dt3/2*noise_ax_, 0,
       0, dt4/4*noise_ay_, 0, dt3/2*noise_ay_,
       dt3/2*noise_ax_, 0, dt2*noise_ax_, 0,
       0, dt3/2*noise_ay_, 0, dt2*noise_ay_;

  ekf_.Predict();

  /**
   * Update
   */

  /**
   * TODO:
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // TODO: Radar updates
    ekf_.Hj_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);

  } else {
    // TODO: Laser updates
    ekf_.Update(measurement_pack.raw_measurements_);
  }
  cout<<"print"<<endl;
  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
