#include <iostream>
#include "tools.h"
#include <math.h>

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

  int estimations_size = estimations.size();
  int ground_truth_size = ground_truth.size();

  VectorXd rmse(4);
  rmse << 0,0,0,0;

  if((estimations_size == 0) or (ground_truth_size == 0))
  {
     return rmse;
  }

  if(estimations_size != ground_truth_size)
  {
     return rmse;
  }

  for(int k = 0; k < estimations_size; k++)
  {
    VectorXd residual = estimations[k] - ground_truth[k];

    //coefficient-wise multiplication
    residual = residual.array()*residual.array();
    rmse += residual;

  }

  rmse = rmse.array() / estimations_size;
  rmse = rmse.array().sqrt();

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
  
  Hj <<   0, 0, 0, 0,
          0, 0, 0, 0,
          0, 0, 0, 0;

  //TODO: YOUR CODE HERE 
  
  //check division by zero
  if((px == 0) && (py == 0))
  {
  	return Hj;
  }
  	
  //compute the Jacobian matrix
  float px2_py2 = pow(px, 2) + pow(py, 2);
  float px2_py2_sqrt = sqrt(px2_py2);
  float px2_py2_1dot5 = sqrt(pow(px2_py2, 3));
  	
  	
  Hj <<   px / px2_py2_sqrt, py / px2_py2_sqrt, 0, 0,
  	    - py / px2_py2, px / px2_py2, 0, 0,
  	    py * (vx*py - vy*px) / px2_py2_1dot5, px * (vy*px - vx*py) / px2_py2_1dot5, px / px2_py2_sqrt, py / px2_py2_sqrt;
  	
  
  return Hj;

}





