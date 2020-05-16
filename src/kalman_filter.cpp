#include "kalman_filter.h"
#include "iostream"

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
                        MatrixXd &H_in, MatrixXd &R_lidar_in, MatrixXd &R_radar_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_lidar_ = R_lidar_in;
  R_radar_ = R_radar_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
   * TODO: predict the state
   */
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  cout<<"update"<<endl;
  VectorXd z_pred = H_ * x_;
  cout<<"h = "<<H_<<endl;
  cout<< "z = " << z <<endl;
  cout<< "z_pred = " << z_pred <<endl;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  cout<<R_lidar_;
  MatrixXd S = (H_ * P_ * Ht) + R_lidar_;
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
   * TODO: update the state by using Extended Kalman Filter equations
   */
cout<<"updateEKF"<<endl;
  double px = x_(0);
  double py = x_(1);
  double vx = x_(2);
  double vy = x_(3);
  
  double rho = sqrt(px * px + py * py);
  double phi = atan2(py, px);

  // Check that denominator is not zero.
  if (rho < 0.000001) {
    rho = 0.000001;
  }
  
  double rho_dot = (px * vx + py * vy) / rho;
  
  VectorXd x_polar = VectorXd(3);
  x_polar << rho, phi, rho_dot;
  VectorXd y = z - x_polar;
  
  // normalize the angle between -pi and pi
  while(y(1) > M_PI){
    y(1) -= 2 * M_PI;
  }

  while(y(1) < -M_PI){
    y(1) += 2 * M_PI;
  }
  
  MatrixXd Ht = Hj_.transpose();
  MatrixXd S = Hj_ * P_ * Ht + R_radar_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;
  
  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * Hj_) * P_;
}
