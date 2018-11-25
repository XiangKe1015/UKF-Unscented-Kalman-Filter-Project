#ifndef UKF_H
#define UKF_H

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class UKF {
public:

  ///* initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;

  ///* if this is false, laser measurements will be ignored (except for init)
  bool use_laser_;

  ///* if this is false, radar measurements will be ignored (except for init)
  bool use_radar_;

  ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  VectorXd x_;

  ///* state covariance matrix
  MatrixXd P_;

  ///* predicted sigma points matrix
  MatrixXd Xsig_pred_;

  ///* time when the state is true, in us
  long long time_us_;

  ///* Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a_;

  ///* Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd_;

  ///* Laser measurement noise standard deviation position1 in m
  double std_laspx_;

  ///* Laser measurement noise standard deviation position2 in m
  double std_laspy_;

  ///* Radar measurement noise standard deviation radius in m
  double std_radr_;

  ///* Radar measurement noise standard deviation angle in rad
  double std_radphi_;

  ///* Radar measurement noise standard deviation radius change in m/s
  double std_radrd_ ;

  ///* Weights of sigma points
  VectorXd weights_;

  ///* State dimension
  int n_x_;

  ///* Augmented state dimension
  int n_aug_;

  ///* Sigma point spreading parameter
  double lambda_;

  ///* timestamp
  long long previous_timestamp_;

  ///* Radar measurement data size
  int n_z_rad_;

  ///* Lidar meausurement data size
  int n_z_las_;

  ///* the current NIS for radar
  double NIS_radar_;

  ///* the current NIS for laser
  double NIS_laser_;

  ///* matrix for augmented sigmapoints
  MatrixXd Xsig_aug_;

  ///* predicted measurement radar
  MatrixXd Zsig_meas_rad_;

  ///* predicted measurement laser
  MatrixXd Zsig_meas_las_;

  ///* mean predicted measurement radar
  VectorXd z_pred_rad_;

  ///* mean predicted measurement laser
  VectorXd z_pred_las_;

  ///* matrix for predicted measurement covariance radar
  MatrixXd S_rad_;

  ///* matrix for predicted measurement covariance laser
  MatrixXd S_las_;


  /**
   * Constructor
   */
  UKF();

  /**
   * Destructor
   */
  virtual ~UKF();

  /**
   * ProcessMeasurement
   * @param meas_package The latest measurement data of either radar or laser
   */
  void ProcessMeasurement(MeasurementPackage meas_package);

  /**
   * Prediction Predicts sigma points, the state, and the state covariance
   * matrix
   * @param delta_t Time between k and k+1 in s
   */
  void Prediction(double delta_t);

  /**
   * Updates the state and the state covariance matrix using a laser measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateLidar(MeasurementPackage meas_package);

  /**
   * Updates the state and the state covariance matrix using a radar measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateRadar(MeasurementPackage meas_package);

  //Generates augmented sigma points
  void AugmentedSigmaPoints(MatrixXd* Xsig_out);

  //Predict sigma points
  void SigmaPointPrediction(MatrixXd* Xsig_out, double delta_t);

  //Predict mean and conariance
  void PredictMeanAndCovariance(VectorXd* x_pred, MatrixXd* P_pred);

  //Predict radar measurement
  void PredictRadarMeasurement(VectorXd* z_out, MatrixXd* S_out, MatrixXd* Zsig_out);

  //Predict laser measurement
  void PredictLaserMeasurement(VectorXd* z_out, MatrixXd* S_out, MatrixXd* Zsig_out);

  //Update state based on radar measurement
  void UpdateRadarState(VectorXd* x_out, MatrixXd* P_out, MeasurementPackage meas_package);

  //Update state based on laser measurement
  void UpdateLaserState(VectorXd* x_out, MatrixXd* P_out, MeasurementPackage meas_package);

};

#endif /* UKF_H */