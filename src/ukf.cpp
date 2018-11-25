#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 0.2;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.2;

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;

  /**
  TODO:
  Complete the initialization. See ukf.h for other member properties.
  Hint: one or more values initialized above might be wildly off...
   */
  is_initialized_ = false;

  n_x_ = 5;

  ///* Augmented state dimension
  n_aug_ = 7;

  ///* Sigma point spreading parameter
  lambda_ = 3 - n_aug_;

  ///* set measurement dimension, 
  ///* lidar measure data : px and py
  ///* radar measure data: r, phi, and r_dot
  n_z_rad_ = 3;

  n_z_las_ = 2; 

  previous_timestamp_ = 0;

  //matrix for augmented sigma points
  Xsig_aug_ = MatrixXd(n_aug_, 2 * n_aug_ + 1);
  
  //matrix for predicted sigma points
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);

  //create vector for weights
  weights_ = VectorXd(2*n_aug_+1);
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:
  Complete this function! Make sure you switch between lidar and radar
  measurements.
   */
  if (!is_initialized_) {
    /**
        TODO:
     * Initialize the state ekf_.x_ with the first measurement.
     * Create the covariance matrix.
     * Remember: you'll need to convert radar from polar to cartesian coordinates.
     */
    // first measurement

    x_.fill(0);

    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      /**
          Convert radar from polar to cartesian coordinates and initialize state.
       */
      float ro = meas_package.raw_measurements_(0);
      float phi = meas_package.raw_measurements_(1);
      float ro_dot = meas_package.raw_measurements_(2);

      float px = ro * cos(phi);
      float py = ro * sin(phi);
      float vx = ro_dot * cos(phi);
      float vy = ro_dot * sin(phi);
      float v = sqrt(vx*vx+vy*vy);
      x_ << px, py, v, 0, 0;

    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      /**
          Initialize state.
       */
      x_ << meas_package.raw_measurements_(0), meas_package.raw_measurements_(1), 0, 0, 0;
    }
    //state covariance matrix P
    P_.setIdentity();

    /*ekf_.P_ << 1, 0, 0, 0,
          0, 1, 0, 0,
          0, 0, 1000, 0,
          0, 0, 0, 1000;*/
    previous_timestamp_ = meas_package.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  float dt = (meas_package.timestamp_ - previous_timestamp_) / 1000000.0; //dt - expressed in seconds
  previous_timestamp_ = meas_package.timestamp_;

  Prediction(dt);

  if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    UpdateRadar(meas_package);
  }
  else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
    UpdateLidar(meas_package);
  }

}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:
  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
   */

  //1, generate augmented sigma points
  AugmentedSigmaPoints(&Xsig_aug_);

  //2, predict sigma points
  SigmaPointPrediction(&Xsig_pred_, delta_t);

  //3, predict mean and covariance
  PredictMeanAndCovariance(&x_, &P_);
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:
  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.
  You'll also need to calculate the lidar NIS.
   */

  //1, predict laser measurement
  PredictLaserMeasurement(&z_pred_las_, &S_las_, &Zsig_meas_las_);

  //2, update state
  UpdateLaserState(&x_, &P_, meas_package);

}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:
  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.
  You'll also need to calculate the radar NIS.
   */

  //1, predict radar measurement
  PredictRadarMeasurement(&z_pred_rad_, &S_rad_, &Zsig_meas_rad_);

  //2, update state
  UpdateRadarState(&x_, &P_, meas_package);

}

//generate sigma points
void UKF::AugmentedSigmaPoints(MatrixXd* Xsig_out) {

  //define spreading parameter

  //create augmented mean vector
  VectorXd x_aug = VectorXd(n_aug_);

  //create augmented state covariance
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);

  //create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);

  x_aug.head(n_x_) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;

  P_aug.fill(0.0);
  P_aug.topLeftCorner(n_x_,n_x_) = P_;
  P_aug(5,5) = std_a_*std_a_;
  P_aug(6,6) = std_yawdd_*std_yawdd_;

  //calculate square root of P_aug
  MatrixXd A_aug = P_aug.llt().matrixL();

  // caclulate square root of lambda + n_aug
  float s_l_n_aug = sqrt(lambda_ + n_aug_);

  // first column of sigma matrix is xk
  Xsig_aug.col(0) = x_aug;

  for (int i = 1; i != n_aug_ + 1; ++i) {
    Xsig_aug.col(i) = x_aug + (s_l_n_aug * A_aug.col(i-1));
    Xsig_aug.col(i+n_aug_) = x_aug - (s_l_n_aug * A_aug.col(i-1));
  }

  //print result
  //cout << "Xsig_aug = " << endl << Xsig_aug << endl;

  *Xsig_out = Xsig_aug;
}

//predict sigma points
void UKF::SigmaPointPrediction(MatrixXd* Xsig_out, double delta_t) {

  //create example sigma point matrix
  //MatrixXd Xsig_aug = Xsig_out;

  //create matrix with predicted sigma points as columns
  MatrixXd Xsig_pred = MatrixXd(n_x_, 2 * n_aug_ + 1);

  //predict sigma points
  //avoid division by zero
  //write predicted sigma points into right column
  for (int i = 0; i != 2*n_aug_+1; i++)
  {
    //extract values for better readability
    double p_x = Xsig_aug_(0,i);
    double p_y = Xsig_aug_(1,i);
    double v = Xsig_aug_(2,i);
    double yaw = Xsig_aug_(3,i);
    double yawd = Xsig_aug_(4,i);
    double nu_a = Xsig_aug_(5,i);
    double nu_yawdd = Xsig_aug_(6,i);

    //predicted state values
    double px_p, py_p;

    //avoid division by zero
    if (yawd != 0) {
      px_p = p_x + v/yawd * ( sin (yaw + yawd*delta_t) - sin(yaw));
      py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );
    }
    else {
      px_p = p_x + v*delta_t*cos(yaw);
      py_p = p_y + v*delta_t*sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd*delta_t;
    double yawd_p = yawd;

    //add noise
    px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
    py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
    v_p = v_p + nu_a*delta_t;

    yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
    yawd_p = yawd_p + nu_yawdd*delta_t;

    //write predicted sigma point into right column
    Xsig_pred(0,i) = px_p;
    Xsig_pred(1,i) = py_p;
    Xsig_pred(2,i) = v_p;
    Xsig_pred(3,i) = yaw_p;
    Xsig_pred(4,i) = yawd_p;
  }

  *Xsig_out = Xsig_pred;

}

void UKF::PredictMeanAndCovariance(VectorXd* x_out, MatrixXd* P_out) {

  //create vector for predicted state
  VectorXd x = VectorXd(n_x_);
  x = x_;

  //create covariance matrix for prediction
  MatrixXd P = MatrixXd(n_x_, n_x_);
  P = P_;

  //set weights
  weights_.fill(1/(2*(lambda_ + n_aug_)));
  weights_(0) = lambda_/(lambda_ + n_aug_);

  //predict state mean
  x.fill(0.0);
  for (int i = 0; i != 2 * n_aug_ + 1; i++) {  //iterate over sigma points
    x = x + weights_(i) * Xsig_pred_.col(i);
  }

  //predict state covariance matrix
  P.fill(0.0);
  for (int i = 0; i != 2 * n_aug_ + 1; i++) {  //iterate over sigma points

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x;
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    P = P + weights_(i) * x_diff * x_diff.transpose() ;
  }

  //write result
  *x_out = x;
  *P_out = P;
}

void UKF::PredictRadarMeasurement(VectorXd* z_out, MatrixXd* S_out, MatrixXd* Zsig_out) {

  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z_rad_, 2 * n_aug_ + 1);

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z_rad_);

  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z_rad_,n_z_rad_);

  //transform sigma points into measurement space
  for (int i = 0; i != 2 * n_aug_ + 1; i++) {  //2n+1 simga points

    // extract values for better readibility
    double p_x = Xsig_pred_(0,i);
    double p_y = Xsig_pred_(1,i);
    double v  = Xsig_pred_(2,i);
    double yaw = Xsig_pred_(3,i);

    double v1 = cos(yaw)*v;
    double v2 = sin(yaw)*v;

    // measurement model
    Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);                        //r
    Zsig(1,i) = atan2(p_y,p_x);                                 //phi
    Zsig(2,i) = (p_x*v1 + p_y*v2 ) / sqrt(p_x*p_x + p_y*p_y);   //r_dot
  }

  //mean predicted measurement
  z_pred.fill(0.0);
  for (int i=0; i != 2*n_aug_+1; i++) {
    z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  //measurement covariance matrix S
  S.fill(0.0);
  for (int i = 0; i != 2 * n_aug_ + 1; i++) {  //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  //add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z_rad_,n_z_rad_);
  R <<    std_radr_*std_radr_, 0, 0,
      0, std_radphi_*std_radphi_, 0,
      0, 0,std_radrd_*std_radrd_;
  S = S + R;

  //write result
  *Zsig_out = Zsig;
  *z_out = z_pred;
  *S_out = S;
}

void UKF::PredictLaserMeasurement(VectorXd* z_out, MatrixXd* S_out, MatrixXd* Zsig_out) {

  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z_las_, 2 * n_aug_ + 1);

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z_las_);

  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z_las_,n_z_las_);

  //transform sigma points into measurement space
  for (int i = 0; i != 2 * n_aug_ + 1; i++) {  //2n+1 simga points

    // extract values for better readibility
    double p_x = Xsig_pred_(0,i);
    double p_y = Xsig_pred_(1,i);

    // measurement model
    Zsig(0,i) = p_x;                        //px
    Zsig(1,i) = p_y;                        //p_y
    
  }

  //mean predicted measurement
  z_pred.fill(0.0);
  for (int i=0; i != 2*n_aug_+1; i++) {
    z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  //measurement covariance matrix S
  S.fill(0.0);
  for (int i = 0; i != 2 * n_aug_ + 1; i++) {  //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  //add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z_las_,n_z_las_);
  R <<    std_laspx_*std_laspx_, 0,
      0, std_laspy_*std_laspy_;

  S = S + R;

  //write result
  *Zsig_out = Zsig;
  *z_out = z_pred;
  *S_out = S;
}

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

void UKF::UpdateLaserState(VectorXd* x_out, MatrixXd* P_out, MeasurementPackage meas_package) {

  //create vector for predicted state
  VectorXd x = VectorXd(n_x_);
  x = x_;

  //create covariance matrix for prediction
  MatrixXd P = MatrixXd(n_x_, n_x_);
  P = P_;

  //create example matrix with sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z_las_, 2 * n_aug_ + 1);
  Zsig = Zsig_meas_las_;

  //create example vector for mean predicted measurement
  VectorXd z_pred = VectorXd(n_z_las_);
  z_pred = z_pred_las_;

  //create example matrix for predicted measurement covariance
  MatrixXd S = MatrixXd(n_z_las_,n_z_las_);
  S = S_las_;

  //create example vector for incoming radar measurement
  VectorXd z = VectorXd(n_z_las_);

  z << meas_package.raw_measurements_(0), meas_package.raw_measurements_(1);

  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z_las_);

  //calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i != 2 * n_aug_ + 1; i++) {  //2n+1 simga points

    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x;

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  //Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  //residual
  VectorXd z_diff = z - z_pred;

  //update state mean and covariance matrix
  x = x + K * z_diff;
  P = P - K*S*K.transpose();

  //calculate laser NIS
  NIS_laser_ = z_diff.transpose() * S.inverse() * z_diff;

  //print result
  /*  std::cout << "Updated state x: " << std::endl << x << std::endl;
  std::cout << "Updated state covariance P: " << std::endl << P << std::endl;*/

  //write result
  *x_out = x;
  *P_out = P;
}