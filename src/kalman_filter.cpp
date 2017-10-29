#include "kalman_filter.h"
#include <math.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;

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
    /* Code Reviewer:
    I suggest moving this variable definition up a few lines and using it instead of repeating the calculation (P_ * Ht).
    */
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
    VectorXd z_pred(3);
    double px = x_[0];
    double py = x_[1];
    double vx = x_[2];
    double vy = x_[3];
    
    
    double z1 = sqrt((px*px) + (py*py));
    
    if (py ==0 && px == 0){
        px = 0.000001;
        py = 0.000001;
    }
    
    double z2 = atan2(py, px);
    double z3 = (px*vx + py*vy)/z1;
    z_pred << z1, z2, z3;

    VectorXd y = z - z_pred;
    /* Code reviewer:
     This method of normalising angles is perfectly fine if the angles are never outside -3pi and 3pi. Alternative approaches could use fmod or atan2. For atan2 the approach is to execute atan2(sin(y(1)), cos(y(1))).
     */
    if (y[1] < -M_PI/2) {
        y[1] = y[1] + 2*M_PI;
    }
    if (y[1] > M_PI/2) {
        y[1] = y[1] - 2*M_PI;
    }
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd PHt = P_ * Ht;
    MatrixXd K = PHt * Si;
    /* Code Reviewer:
     I suggest moving this variable definition up a few lines and using it instead of repeating the calculation (P_ * Ht).
     */
    
    //new estimate
    x_ = x_ + (K * y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    // Code Reveiwer: I suggest you try refactoring your implementation to keep it DRY as there is a lot of repeated code in your two update methods.
    P_ = (I - K * H_) * P_;

}
