#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

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
   * TODO: predict the state
   */
  x_=F_*x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_*P_*Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
    VectorXd y=z-H_*x_;
  	MatrixXd Ht=H_.transpose();
  	MatrixXd S= H_*P_*Ht +R_;
    MatrixXd Si = S.inverse();
  	MatrixXd K= P_*Ht*Si;
  	x_=x_+K*y;
  	MatrixXd I= MatrixXd::Identity(x_.size(), x_.size());
  	P_=(I-K*H_)*P_;
  
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
  VectorXd hx(3);//Vector to store polar coordinates after transformation
  float rho= sqrt(x_(0)*x_(0)+x_(1)*x_(1));
  float theta=atan2(x_(1),x_(0));
  float rhodot;
  if (fabs(rho) < 0.0001){
    rhodot =0;
  }
  else{
    rhodot = (x_(0)*x_(2) + x_(1)*x_(3))/rho;
  }
  hx<<rho,theta,rhodot;
  VectorXd y=z-hx;
  y[1]=atan2(sin(y[1]),cos(y[1]));
  MatrixXd Ht=H_.transpose();
  MatrixXd S= H_*P_*Ht +R_;
  MatrixXd K= P_*Ht*(S.inverse());
  x_=x_+K*y;
  MatrixXd I= MatrixXd::Identity(x_.size(), x_.size());
  P_=(I-K*H_)*P_;    
}
