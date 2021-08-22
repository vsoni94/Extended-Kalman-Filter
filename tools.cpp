#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */
  VectorXd rmse(4);
  rmse<<0,0,0,0;
  if(estimations.size()==0||ground_truth.size()==0||estimations.size()!=ground_truth.size())
  {
    std::cout<<"CalculateRMSE(): Invalid Size of Estimates or Ground Truth"<<std::endl;
    return rmse;
	}
	for (int i=0; i < estimations.size(); ++i) {
    // ... your code here
    VectorXd residual = estimations[i] - ground_truth[i];
    residual = residual.array()*residual.array();
    rmse+=residual; 
    
  }
  rmse = rmse/estimations.size();
  rmse = rmse.array().sqrt();
  return rmse;  
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
  MatrixXd Hj(3,4);
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);
  float d1 = px*px + py*py;
  float d2 = sqrt(d1);
  float d3 = d1*d2;
  if(fabs(d1) < 0.0001){
    std::cout << "CalculateJacobian(): Divison by zero." << std::endl;
    return Hj;
  }

  // Compute Jacobian matrix
  Hj << px/d2, py/d2, 0, 0,
        -py/d1, px/d1, 0, 0,
        py*(vx*py - vy*px)/d3, px*(vy*px - vx*py)/d3, px/d2, py/d2;

  return Hj;
}
