#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
  VectorXd rmse(4);
  rmse << 0,0,0,0;
  if(estimations.size() == 0 || estimations.size() != ground_truth.size()){
    std::cout << "Invalid estimation or ground truth data" << std::endl;
    return rmse;
  }
  //accumulate squared residuals
  for(int i=0; i < estimations.size(); ++i){
    // ... your code here
    VectorXd residual = estimations[i] - ground_truth[i];

    //coefficient-wise multiplication
    residual = residual.array()*residual.array();
    rmse += residual;

  }
  //calculate the mean
  rmse = rmse/estimations.size();
  //calculate the squared root
  rmse = rmse.array().sqrt();
  //return result
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
  MatrixXd Hj(3,4);
  //recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  //TODO: YOUR CODE HERE
  float preg = px*px + py*py;
  float g = sqrt (preg);
  float postg = preg * g;

  //check division by zero
  if (fabs(preg) < 0.0001) {
    std::cout << "CalculateJacobinan() - Error - Division by Zero" << std::endl;
//    return Hj;    // this would return an uninitialized Hj
    preg = 0.001;
  }


  //compute the Jacobian matrix
  Hj << px/g, py/g, 0, 0,
      -py/preg, px/preg, 0, 0,
      py*(vx*py-vy*px)/postg, px*(vy*px-vx*py)/postg, px/g, py/g;

  return Hj;
}
