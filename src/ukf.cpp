#include "ukf.h"
#include "Eigen/Dense"


/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  
  use_laser_ = true;    // if this is false, laser measurements will be ignored (except during init)
  use_radar_ = true;    // if this is false, radar measurements will be ignored (except during init)
  plot_nis = false;

  is_initialized_ = false;

  n_x_ = 5;                                           // number of states
  n_aug_ = 7;                                         // number of augmented states: n_x_ + number of acceleration noise variables
  n_zr_ = 3;                                          // number of radar measurements
  n_zl_ = 2;                                          // number of lidar measurements
  lambda_ = 3 - n_x_;                                 // covariance scaling factor

  x_ = Eigen::VectorXd(n_x_);                         // state vector used for tracking traffic car
  P_ = Eigen::MatrixXd(n_x_, n_x_);                   // covariance matrix used for tracking traffic car state estimate uncertainity

  x_pred_ = Eigen::VectorXd(n_x_);                    // mean predicted state from sigma points
  P_pred_ = Eigen::MatrixXd(n_x_, n_x_);              // predicted state covariance matrix P
  Xsig_pred_ = Eigen::MatrixXd(n_x_, 2*n_aug_ + 1);   // predicted sigma points matrix from motion update

  z_pred_ = Eigen::VectorXd(n_zr_);                   // mean predicted measurement from sigma points
  S_pred_ = Eigen::MatrixXd(n_zr_,n_zr_);             // predicted measurement covariance matrix S
  Zsig_pred_ = Eigen::MatrixXd(n_zr_,2*n_aug_ + 1);   // predicted sigma points matrix from measurement update

  weights_ = Eigen::VectorXd(2*n_aug_+1);             // sigma point weights

  double a_max = 6.0;             // maximum expected acceleration of car (estimation)
  std_a_ = a_max/2.0;             // Process noise standard deviation longitudinal acceleration in m/s^2
  double yawdd_max = 4.0;    // maximum expected acceleration of car (estimation)
  std_yawdd_ = yawdd_max/2.0;     // Process noise standard deviation yaw acceleration in rad/s^2

  nis_plotter = new pcl::visualization::PCLPlotter("NIS Plotter");
  
  /**
   * DO NOT MODIFY measurement noise values below.
   * These are provided by the sensor manufacturer.
   */

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
   * End DO NOT MODIFY section for measurement noise values 
   */
  
  /**
   * TODO: Complete the initialization. See ukf.h for other member properties.
   * Hint: one or more values initialized above might be wildly off...
   */
  Rr = Eigen::MatrixXd(n_zr_,n_zr_);
  Rr.fill(0);
  Rr(0,0) = pow(std_radr_,2);
  Rr(1,1) = pow(std_radphi_,2);
  Rr(2,2) = pow(std_radrd_,2);

  Rl = Eigen::MatrixXd(n_zl_, n_zl_);
  Rl.fill(0);
  Rl(0,0) = pow(std_laspx_,2);
  Rl(1,1) = pow(std_laspy_,2);

  H = Eigen::MatrixXd(n_zl_, n_x_);
  H.fill(0);
  H(0,0) = 1;
  H(1,1) = 1;
}

UKF::~UKF() {

  // delete nis_plotter;
}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Make sure you switch between lidar and radar
   * measurements.
   */
  if (!is_initialized_){

    std::cout << "--------- Initialized Unscented Kalman Filter ---------" << std::endl;
    is_initialized_ = true;
    P_ = Eigen::MatrixXd::Identity(n_x_,n_x_);

    if (meas_package.sensor_type_ == MeasurementPackage::LASER){
      x_ << meas_package.raw_measurements_[0],
          meas_package.raw_measurements_[1],
          0,
          0,
          0;
      P_(0,0) = pow(std_laspx_,2);
      P_(1,1) = pow(std_laspy_,2);
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::RADAR){
      x_ << meas_package.raw_measurements_[0]*cos(meas_package.raw_measurements_[1]),     // px = rho_x
          meas_package.raw_measurements_[0]*sin(meas_package.raw_measurements_[1]),       // py = rho_y
          meas_package.raw_measurements_[2],                                              // v = rho_dot   (not quite but good initial estimate)
          meas_package.raw_measurements_[1],                                              // psi = rho      (not quite but good initial estimate)
          0;
    }
    else{
      std::cout << "Unknown sensor type in measurement package!" <<std::endl;
    }

    previous_timestamp_ = meas_package.timestamp_;
    return;
  }

  double dt = (meas_package.timestamp_ - previous_timestamp_)/1000000.0;
  previous_timestamp_ = meas_package.timestamp_;

  // motion update
  // std::cout << "------- update state prediction -------" << std::endl;
  Prediction(dt);

  // measurement update
  if ((meas_package.sensor_type_ == MeasurementPackage::LASER) && use_laser_){
    // std::cout << "------- update LiDAR -------" << std::endl;
    UpdateLidar(meas_package);
  }
  else if ((meas_package.sensor_type_ == MeasurementPackage::RADAR) && use_radar_){
    // std::cout << "------- update radar -------" << std::endl;
    UpdateRadar(meas_package);
  }
}

void UKF::Prediction(double delta_t) {
  /**
   * TODO: Complete this function! Estimate the object's location. 
   * Modify the state vector, x_. Predict sigma points, the state, 
   * and the state covariance matrix.
   */

  // create sigma point matrix
  Eigen::MatrixXd Xsig_aug = Eigen::MatrixXd(n_aug_, 2*n_aug_ + 1);
  generateAugmentedSigmaPoints(Xsig_aug);
  sigmaPointPrediction(Xsig_aug, delta_t);
  predictMeanAndCovariance();   // updates x_pred_ and P_pred_

}

void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use lidar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */

  // create vector for incoming radar measurement
  Eigen::VectorXd z = Eigen::VectorXd(n_zl_);
  z <<meas_package.raw_measurements_[0],   // px in m
      meas_package.raw_measurements_[1];   // py in m

  Eigen::VectorXd z1 = H*x_pred_;
  Eigen::VectorXd y = z - z1;
  Eigen::MatrixXd S = H*P_pred_*H.transpose() + Rl;
  Eigen::MatrixXd K = P_pred_*H.transpose()*S.inverse();

  x_ = x_pred_ + K*y;
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(n_x_, n_x_);
  P_ = (I - K*H)*P_pred_;

  // print result
  // std::cout << "Updated state x: " << std::endl << x_ << std::endl;
  // std::cout << "Updated state covariance P: " << std::endl << P_ << std::endl;

  // calculate NIS 
    double nis_lidar = (z - z1).transpose() * S.inverse() * (z - z1);
    nis_lidar_values.push_back(nis_lidar);
    if (nis_lidar_values.size() > 2 && plot_nis){
      plotLidarNIS();
    }

}

void UKF::plotLidarNIS(){
  std::vector<double> xAxis(300);
  std::iota(xAxis.begin(), xAxis.end(), 0);
  std::vector<double> y_data_lidar(300);
  std::generate(y_data_lidar.begin(), y_data_lidar.end(), [this] () {return this->nis_lidar_threshold;});
  nis_plotter->addPlotData(xAxis, y_data_lidar);

  // Plot all threshold values calculated so far
  std::vector<double> xAxisLidar(nis_lidar_values.size());
  std::iota(xAxisLidar.begin(), xAxisLidar.end(), 0);
  nis_plotter->addPlotData(xAxisLidar, nis_lidar_values);
  nis_plotter->spinOnce(100);
  // nis_plotter->clearPlots();

}

void UKF::UpdateRadar(MeasurementPackage meas_package) {
    /**
     * TODO: Complete this function! Use radar data to update the belief 
     * about the object's position. Modify the state vector, x_, and 
     * covariance, P_.
     * You can also calculate the radar NIS, if desired.
     */

    // transform sigma points into measurement space
    for (int i=0; i < 2 * n_aug_ + 1; i++){

        double px = Xsig_pred_(0,i);
        double py = Xsig_pred_(1,i);
        double v = Xsig_pred_(2,i);
        double psi = Xsig_pred_(3,i);
        double psidot = Xsig_pred_(4,i);

        Zsig_pred_(0,i) = sqrt(pow(px,2) + pow(py,2));
        Zsig_pred_(1,i) = atan2(py,px);
        Zsig_pred_(2,i) = (px*v*cos(psi) + py*v*sin(psi)) / sqrt(pow(px,2) + pow(py,2));
    }
    
    // calculate mean predicted measurement
    z_pred_.fill(0.0);
    for (int i=0; i < 2*n_aug_+1; ++i) {
        z_pred_ = z_pred_ + weights_(i) * Zsig_pred_.col(i);
    }

    // calculate innovation covariance matrix S
    S_pred_.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; ++i) {  // 2n+1 simga points
        // residual
        Eigen::VectorXd z_diff = Zsig_pred_.col(i) - z_pred_;

        // angle normalization
        while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
        while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

        S_pred_ = S_pred_ + weights_(i) * z_diff * z_diff.transpose();
    }

    S_pred_ += Rr;   // add in Rr because we did not consider noise when transforming sigma points to radar measurement space

    // print result
    // std::cout << "z_pred: " << std::endl << z_pred_ << std::endl;
    // std::cout << "S: " << std::endl << S_pred_ << std::endl;

    // create vector for incoming radar measurement
    Eigen::VectorXd z = Eigen::VectorXd(n_zr_);
    z <<meas_package.raw_measurements_[0],   // rho in m
        meas_package.raw_measurements_[1],   // phi in rad
        meas_package.raw_measurements_[2];   // rho_dot in m/s

    // create matrix for cross correlation Tc
    Eigen::MatrixXd Tc = Eigen::MatrixXd(n_x_, n_zr_);

    Tc.fill(0);
    // calculate cross correlation matrix

    for (int i=0; i < 2 * n_aug_ + 1; ++i) {
      Eigen::VectorXd z_diff = Zsig_pred_.col(i) - z_pred_;
      // angle normalization
      while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
      while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

      // state difference
      Eigen::VectorXd x_diff = Xsig_pred_.col(i) - x_pred_;
      // angle normalization
      while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
      while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

      Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
    }

    // calculate Kalman gain K;
    Eigen::MatrixXd K = Eigen::MatrixXd(n_x_, n_x_);
    K = Tc * S_pred_.inverse();

    // update state mean and covariance matrix
    x_ = x_pred_ + K * (z - z_pred_);
    P_ = P_pred_ - K*S_pred_*K.transpose();

    // print result
    // std::cout << "Updated state x: " << std::endl << x_ << std::endl;
    // std::cout << "Updated state covariance P: " << std::endl << P_ << std::endl;

    // calculate NIS 
    double nis_radar = (z - z_pred_).transpose() * S_pred_.inverse() * (z - z_pred_);
    nis_radar_values.push_back(nis_radar);
    if (nis_radar_values.size() > 2 && plot_nis){
      plotRadarNIS();
    }
}

void UKF::plotRadarNIS(){

  // Plot threshold line
  std::vector<double> xAxis(300);
  std::iota(xAxis.begin(), xAxis.end(), 0);
  std::vector<double> y_data_radar(300);
  std::generate(y_data_radar.begin(), y_data_radar.end(), [this] () {return this->nis_radar_threshold;});
  nis_plotter->addPlotData(xAxis, y_data_radar);

  // Plot all threshold values calculated so far
  std::vector<double> xAxisRadar(nis_radar_values.size());
  std::iota(xAxisRadar.begin(), xAxisRadar.end(), 0);
  nis_plotter->addPlotData(xAxisRadar, nis_radar_values);
  nis_plotter->spinOnce(100);
  nis_plotter->clearPlots();
}

void UKF::generateAugmentedSigmaPoints(Eigen::MatrixXd& X_aug) {

    // create augmented mean vector
    Eigen::VectorXd x_aug = Eigen::VectorXd(n_aug_);

    // create augmented state covariance
    Eigen::MatrixXd P_aug = Eigen::MatrixXd(n_aug_, n_aug_);
    
    // create augmented mean state
    x_aug.head(5) = x_;
    x_aug(5) = 0;
    x_aug(6) = 0;

    // create augmented covariance matrix
    P_aug.fill(0.0);
    P_aug.topLeftCorner(5,5) = P_;
    P_aug(5,5) = std_a_*std_a_;
    P_aug(6,6) = std_yawdd_*std_yawdd_;

    // create square root matrix
    Eigen::MatrixXd L = P_aug.llt().matrixL();

    // create augmented sigma points
    X_aug.col(0) = x_aug;
    for (int i = 0; i < n_aug_; ++i) {
        X_aug.col(i+1) = x_aug + sqrt(lambda_+n_aug_) * L.col(i);
        X_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_+n_aug_) * L.col(i);
    }

    // print result
    // std::cout << "Xsig_aug = " << std::endl << X_aug << std::endl;
  
}

void UKF::sigmaPointPrediction(Eigen::MatrixXd& Xsig_aug, double dt){

    for (int i=0; i < 2*n_aug_+1; i++){

        double px = Xsig_aug(0,i);
        double py = Xsig_aug(1,i);
        double v = Xsig_aug(2,i);
        double psi = Xsig_aug(3,i);
        double psidot = Xsig_aug(4,i);
        double nu_lin_acc = Xsig_aug(5,i);
        double nu_ang_acc = Xsig_aug(6,i);

        Eigen::VectorXd F(n_x_);
        Eigen::VectorXd nu(n_x_);

        if (fabs(psidot) < 0.001){
            F << v*cos(psi)*dt,
                v*sin(psi)*dt,
                0,
                psidot*dt,
                0;
        }
        else{
            F << (v/psidot)*(sin(psi + psidot*dt) - sin(psi)),
                (v/psidot)*(-cos(psi + psidot*dt) + cos(psi)),
                0,
                psidot*dt,
                0;
        }
        
        nu << (1/2)*pow(dt,2)*cos(psi)*nu_lin_acc,
                (1/2)*pow(dt,2)*sin(psi)*nu_lin_acc,
                dt*nu_lin_acc,
                (1/2)*pow(dt,2)*nu_ang_acc,
                dt*nu_ang_acc;

        Xsig_pred_.col(i) = Xsig_aug.block(0,i,n_x_,1) + F + nu;
    }

    // print result
    // std::cout << "Xsig_pred = " << std::endl << Xsig_pred_ << std::endl;

}

void UKF::predictMeanAndCovariance(){

    // set weights
    // predict state mean
    weights_(0) = lambda_/(lambda_+n_aug_);
    x_pred_ = weights_(0) * Xsig_pred_.col(0);

    for (int i=1; i < 2*n_aug_+1; i++){
        weights_(i) = 1/(2*(lambda_+n_aug_));
        x_pred_ += weights_(i)*Xsig_pred_.col(i);
    }

    // predict state covariance matrix
    P_pred_.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; ++i) {  // iterate over sigma points
        // state difference
        Eigen::VectorXd x_diff = Xsig_pred_.col(i) - x_pred_;
        // angle normalization
        while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
        while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

        P_pred_ = P_pred_ + weights_(i) * x_diff * x_diff.transpose() ;
    }

    // print result
    // std::cout << "Predicted state" << std::endl;
    // std::cout << x_pred_ << std::endl;
    // std::cout << "Predicted covariance matrix" << std::endl;
    // std::cout << P_pred_ << std::endl;

}
