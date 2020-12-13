#include "kalman_filter.h"
#include <iostream>
#define PI 3.14
using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;
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
   x_ = F_ * x_;
   //cout<<"x:"<<x_<<endl;
   P_ = (F_ * P_ * (F_.transpose())) + Q_;
   //cout<<"P:"<<P_<<endl;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
   	VectorXd y = z - (H_ * x_);  //dont use Z transpose here instead of z
   	//cout<<"Y"<<y<<endl;
	MatrixXd S = (H_ * P_ * H_.transpose()) + R_;
	//cout<<"S"<<S<<endl;
	MatrixXd K = P_ * H_.transpose() * S.inverse();
	//cout<<"K"<<K<<endl;
	
	MatrixXd I = MatrixXd::Identity(4, 4);
	// new state
	x_ = x_ + (K * y);
	P_ = (I - (K * H_)) * P_;
	//cout<<"New P"<<P_<<endl;
	
    
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
   
   //Converting x values to polar coordinates (Here we don't use y = z - (H_ * x_); because we convert the measured values to rho phi and theta
    VectorXd hx(3);
    float px = x_(0);
  	float py = x_(1);
  	float vx = x_(2);
  	float vy = x_(3);
  	
  	float rho = sqrt((px*px)+(py*py));
  	float phi = atan2(py,px);  //arctan
  	float rho_dot = ((px*vx) + (py*vy))/rho;
  	
  	hx << rho,phi,rho_dot;
  	
	VectorXd y = z - hx;
	//Angle normalization (this prevents abnormal behaviour at one point of curve where the predict deviates
	if(y(1) > PI){
		y(1) -= (2*PI);
	}
	
	if(y(1) < -PI)
	{
		y(1) += (2*PI);
	}
    //cout<<"Y"<<y<<endl;
	MatrixXd S = (H_ * P_ * H_.transpose()) + R_;
	//cout<<"S"<<S<<endl;
	MatrixXd K = P_ * H_.transpose() * S.inverse();
	//cout<<"K"<<K<<endl;
	
	MatrixXd I = MatrixXd::Identity(4, 4);
	// new state
	x_ = x_ + (K * y);
	P_ = (I - (K * H_)) * P_;
	//cout<<"New P"<<P_<<endl;
}
