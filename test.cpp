#include <iostream>
#include "Eigen/Dense"
#include <vector>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;


void UKF::UpdateRadarState(VectorXd* x_out, MatrixXd* P_out, MeasurementPackage meas_package) {

	//create vector for predicted state
	VectorXd x = VectorXd(n_x_);
	x = x_;

	//create covariance matrix for prediction
	MatrixXd P = MatrixXd(n_x_, n_x_);
	P = P_;

	//create matrix with sigma points in measurement space
	MatrixXd Zsig = MatrixXd(n_z_rad_, 2 * n_aug_ + 1);
	Zsig = Zsig_meas_rad_;

	//create  vector for mean predicted measurement
	VectorXd z_pred = VectorXd(n_z_rad_);
	z_pred = z_pred_rad_;

	//create  matrix for predicted measurement covariance
	MatrixXd S = MatrixXd(n_z_rad_,n_z_rad_);
	S = S_rad_;

	//create  vector for incoming radar measurement
	VectorXd z = VectorXd(n_z_rad_);
	float ro = meas_package.raw_measurements_(0);
	float phi = meas_package.raw_measurements_(1);
	float ro_dot = meas_package.raw_measurements_(2);
	z << ro, phi, ro_dot;

	//create matrix for cross correlation Tc
	MatrixXd Tc = MatrixXd(n_x_, n_z_rad_);

	//calculate cross correlation matrix
	Tc.fill(0.0);
	for (int i = 0; i != 2 * n_aug_ + 1; i++) {  //2n+1 simga points

		//residual
		VectorXd z_diff = Zsig.col(i) - z_pred;
		//angle normalization
		while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
		while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

		// state difference
		VectorXd x_diff = Xsig_pred_.col(i) - x;
		//angle normalization
		while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
		while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

		Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
	}

	//Kalman gain K;
	MatrixXd K = Tc * S.inverse();

	//residual
	VectorXd z_diff = z - z_pred;

	//angle normalization
	while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
	while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

	//update state mean and covariance matrix
	x = x + K * z_diff;
	P = P - K*S*K.transpose();

	//calculate radar NIS
	NIS_radar_ = z_diff.transpose() * S.inverse() * z_diff;

	//print result
	/*  std::cout << "Updated state x: " << std::endl << x << std::endl;
  std::cout << "Updated state covariance P: " << std::endl << P << std::endl;*/

	//write result
	*x_out = x;
	*P_out = P;
}
