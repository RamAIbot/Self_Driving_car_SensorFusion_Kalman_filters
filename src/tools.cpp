#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

using namespace std;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */
   VectorXd rmse(4);
	rmse << 0,0,0,0;

   
   
  // TODO: accumulate squared residuals
  if((estimations.size()!=ground_truth.size())||(estimations.size()==0))
  {
      cout << "Invalid estimation or ground_truth data" << endl;
    return rmse;
  }
   
	for (int i=0; i < estimations.size(); ++i) {
    
    VectorXd diff = estimations[i] - ground_truth[i];
    diff = diff.array()*diff.array();
    rmse += diff;
  }

  
  rmse = rmse / estimations.size();
  
  rmse = rmse.array().sqrt();
	
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
   MatrixXd Jac(3,4);
  // recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // TODO: YOUR CODE HERE 

  // check division by zero
  float mag = sqrt(pow(px,2)+pow(py,2));
  float sub = (vx*py) - (vy*px);
  float sub1 = (vy*px) - (vx*py);
  
  if(fabs(mag)<0.0001)
  {
      cout << "CalculateJacobian () - Error - Division by Zero" << endl;
    return Jac;
  }
  // compute the Jacobian matrix
  Jac<< (px/mag), (py/mag), 0, 0,
       (-py/(pow(mag,2))), (px/(pow(mag,2))), 0, 0,
       ((py*sub)/(pow(mag,3))), ((px*sub1)/(pow(mag,3))) , (px/mag) , (py/mag);
  return Jac;
  
}
