#ifndef UKF_H
#define UKF_H

#include <iostream>
#include "Eigen/Dense"
#include "measurement_package.h"

class UKF {
 public:
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

  void generateAugmentedSigmaPoints(Eigen::MatrixXd& X_aug);
  void sigmaPointPrediction(Eigen::MatrixXd& Xsig_aug, double dt);
  void predictMeanAndCovariance();
  void updateStateFromRadar(MeasurementPackage meas_package);

  // initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;

  // if this is false, laser measurements will be ignored (except for init)
  bool use_laser_;

  // if this is false, radar measurements will be ignored (except for init)
  bool use_radar_;

  // final estimated state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  Eigen::VectorXd x_;

  // final estimated state covariance matrix
  Eigen::MatrixXd P_;

  // predicted sigma points matrix from motion update
  Eigen::MatrixXd Xsig_pred_;

  // predicted state vector from sigma points: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  Eigen::VectorXd x_pred_;

  // predicted state covariance matrix from sigma points
  Eigen::MatrixXd P_pred_;

  // predicted sigma points matrix from measurement update
  Eigen::MatrixXd Zsig_pred_;

  Eigen::MatrixXd S_pred_;

  // predicted measurement vector from sigma points: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  Eigen::VectorXd z_pred_;

  // time when the state is true, in us
  long long time_us_;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a_;

  // Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd_;

  // Laser measurement noise standard deviation position1 in m
  double std_laspx_;

  // Laser measurement noise standard deviation position2 in m
  double std_laspy_;

  // Radar measurement noise standard deviation radius in m
  double std_radr_;

  // Radar measurement noise standard deviation angle in rad
  double std_radphi_;

  // Radar measurement noise standard deviation radius change in m/s
  double std_radrd_ ;

  // Weights of sigma points
  Eigen::VectorXd weights_;

  int n_x_;         // State dimension
  int n_aug_;       // Augmented state dimension
  int n_zr_;        // number of radar measurements
  int n_zl_;        // number of lidar measurements
  double lambda_;   // Sigma point spreading parameter

  Eigen::MatrixXd Rr;   // radar measurement noise covariance matrix
  Eigen::MatrixXd Rl;   // lidar measurement noise covariance matrix
  Eigen::MatrixXd H;    // linear H matrix to transform predicted state to lidar measurement space

  long previous_timestamp_;
};

#endif  // UKF_H