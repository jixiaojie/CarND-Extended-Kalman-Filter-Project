#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}




/**
* Convert radar from polar to cartesian coordinates.
*/
VectorXd KalmanFilter::ConvertPolarToCartesian(const VectorXd& polarvalue){

  VectorXd cartesian(4);

  float polar_rho = polarvalue(0);
  float polar_phi = polarvalue(1) * 180.0 /3.1415;
  float polar_rhodot = polarvalue(2);
  
  float px = polar_rho * cos(polar_phi);
  float py = polar_rho * sin(polar_phi);
  float vx = polar_rhodot * cos(polar_phi);
  float vy = polar_rhodot * sin(polar_phi);

  cartesian << px, py, vx, vy;

  return cartesian;


}


/**
* Convert radar from cartesian to polar coordinates.
*/
VectorXd KalmanFilter::ConvertCartesianToPolar(const VectorXd& cartesianvalue){

  VectorXd polar(3);
  //polar = VectorXd(3);

  polar << 0.0, 0.0, 0.0;

  float px = cartesianvalue(0);
  float py = cartesianvalue(1);
  float vx = cartesianvalue(2);
  float vy = cartesianvalue(3);

  float polar_rho = sqrt(px * px + py * py);
  float polar_phi = 0.0;
  float polar_rhodot = 0.0;

  if ((polar_rho != 0) && (px != 0)) {

    polar_phi = atan2(py, px);
    polar_rhodot = (px * vx + py * vy) / polar_rho;

    polar << polar_rho, polar_phi, polar_rhodot;

  }

  return polar;
}



void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
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

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  if ((abs(x_[0]) > 0.8) && (abs(x_[1]) > 0.8)) {

    VectorXd z_pred = KalmanFilter::ConvertCartesianToPolar(x_);
    VectorXd y = z - z_pred;
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
}
