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
  //std_a_ = 0.23;
  std_a_ = 0.7;
  // Process noise standard deviation yaw acceleration in rad/s^2
  //std_yawdd_ = 0.23;
  std_yawdd_ = 0.75;
  
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
  P_ << 1, 0, 0, 0, 0,
        0, 1, 0, 0, 0,
        0, 0, 1, 0, 0,
        0, 0, 0, 1, 0,
        0, 0, 0, 0, 1;
  
  is_initialized_ = false;
  
  time_us_ = 0;
  
  n_x_ = 5;
  
  n_aug_ = 7;
  
  lambda_ = 3 - n_aug_;
  
  Xsig_pred_ = MatrixXd(n_x_, 2*n_aug_ + 1);
  
  weights_ = VectorXd(2*n_aug_ + 1);
  weights_(0) = lambda_ / (lambda_ + n_aug_);
  for (int i=2; i<=2*n_aug_+1; ++i) {
    weights_(i-1) = 1 / (2*(lambda_ + n_aug_));
  }
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(const MeasurementPackage &meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
  if (!is_initialized_) {
    cout << "UKF: " << endl;
    if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      x_ << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], 0, 0, 0;
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      float rho_ = meas_package.raw_measurements_[0];
      float theta_ = meas_package.raw_measurements_[1];
      float px_ = rho_*cos(theta_);
      float py_ = rho_*sin(theta_);
      x_ << px_, py_, 0, 0, 0;
    }
    time_us_ = meas_package.timestamp_;
    is_initialized_ = true;
    return;
  }
  
  if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_) {
    double dt_ = (meas_package.timestamp_ - time_us_) / 1000000.0;
    time_us_ = meas_package.timestamp_;
    Prediction(dt_);
    UpdateLidar(meas_package);
    cout << "x_ = " << x_ << endl;
    cout << "P_ = " << P_ << "\n" << endl;
  }
  
  else if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_) {
    double dt_ = (meas_package.timestamp_ - time_us_) / 1000000.0;
    time_us_ = meas_package.timestamp_;
    Prediction(dt_);
    UpdateRadar(meas_package);
    cout << "x_ = " << x_ << endl;
    cout << "P_ = " << P_ << "\n" << endl;
  }
  else {
    return;
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
  MatrixXd P_aug_ = MatrixXd(n_aug_, n_aug_);
  VectorXd x_aug_ = VectorXd(n_aug_);
  P_aug_.fill(0.);
  x_aug_.fill(0.);
  MatrixXd Xsig_aug_ = MatrixXd(n_aug_, 2*n_aug_ + 1);
  x_aug_.head(n_x_) = x_;
  P_aug_.topLeftCorner(n_x_, n_x_) = P_;
  P_aug_(5, 5) = std_a_*std_a_;
  P_aug_(6, 6) = std_yawdd_*std_yawdd_;
  MatrixXd P_aug_root_ = P_aug_.llt().matrixL();
  
  Xsig_aug_.col(0) = x_aug_;
  for (int i=0; i<n_aug_; ++i) {
    Xsig_aug_.col(i+1) = x_aug_ + sqrt(lambda_ + n_aug_)*P_aug_root_.col(i);
    Xsig_aug_.col(i+1+n_aug_) = x_aug_ - sqrt(lambda_ + n_aug_)*P_aug_root_.col(i);
  }
  
  for (int i=0; i< 2 * n_aug_ + 1; ++i) {
    double v = Xsig_aug_.col(i)[2];
    double psi = Xsig_aug_.col(i)[3];
    double psi_dot = Xsig_aug_.col(i)[4];
    double nu_a = Xsig_aug_.col(i)[5];
    double nu_psi = Xsig_aug_.col(i)[6];
    if (psi_dot != 0) {
      Xsig_pred_.col(i)[0] = v/psi_dot*(sin(psi+psi_dot*delta_t)-sin(psi));
      Xsig_pred_.col(i)[0] += 0.5*delta_t*delta_t*cos(psi)*nu_a + Xsig_aug_.col(i)[0];
      Xsig_pred_.col(i)[1] = v/psi_dot*(-cos(psi+psi_dot*delta_t)+cos(psi));
      Xsig_pred_.col(i)[1] += 0.5*delta_t*delta_t*sin(psi)*nu_a + Xsig_aug_.col(i)[1];
      Xsig_pred_.col(i)[2] = delta_t*nu_a + v;
      Xsig_pred_.col(i)[3] = psi_dot*delta_t + 0.5*delta_t*delta_t*nu_psi + psi;
      Xsig_pred_.col(i)[4] = delta_t*nu_psi + psi_dot;
    } else {
      Xsig_pred_.col(i)[0] = v*cos(psi)*delta_t;
      Xsig_pred_.col(i)[0] += 0.5*delta_t*delta_t*cos(psi)*nu_a + Xsig_aug_.col(i)[0];
      Xsig_pred_.col(i)[1] = v*sin(psi)*delta_t;;
      Xsig_pred_.col(i)[1] += 0.5*delta_t*delta_t*sin(psi)*nu_a + Xsig_aug_.col(i)[1];
      Xsig_pred_.col(i)[2] = delta_t*nu_a + v;
      Xsig_pred_.col(i)[3] = 0.5*delta_t*delta_t*nu_psi + psi;
      Xsig_pred_.col(i)[4] = delta_t*nu_psi + psi_dot;
    }
  }
  
  x_.fill(0.);
  for (int i=0; i<2*n_aug_+1; ++i) {
    x_ = x_ + weights_(i) * Xsig_pred_.col(i);
  }
  
  P_.fill(0.);
  VectorXd x_diff_ = VectorXd(n_x_);
  for (int i=0; i<2*n_aug_+1; ++i) {
    x_diff_ = Xsig_pred_.col(i) - x_;
    while (x_diff_(3) < -M_PI) x_diff_(3) += 2.*M_PI;
    while (x_diff_(3) > M_PI) x_diff_(3) -= 2.*M_PI;
    P_ += weights_[i] * x_diff_ * x_diff_.transpose();
  }
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
  MatrixXd Zsig_ = MatrixXd(2, 2*n_aug_+1);
  VectorXd z_pred_ = VectorXd(2);
  MatrixXd S_ = MatrixXd(2,2);
  MatrixXd Tc_ = MatrixXd(n_x_, 2);
  MatrixXd K_ = MatrixXd(n_x_, 2);
  
  MatrixXd R_ = MatrixXd(2,2);
  R_ << std_laspx_*std_laspx_, 0,
  0, std_laspy_*std_laspy_;
  
  VectorXd z_ = VectorXd(2);
  z_ = meas_package.raw_measurements_;
  
  Zsig_.row(0) = Xsig_pred_.row(0);
  Zsig_.row(1) = Xsig_pred_.row(1);
  
  z_pred_.fill(0.);
  for (int i=0; i<2*n_aug_+1; ++i) {
    z_pred_ += weights_(i) * Zsig_.col(i);
  }
  
  VectorXd z_diff_ = VectorXd(2);
  VectorXd X_diff_ = VectorXd(n_x_);
  S_.fill(0.);
  Tc_.fill(0.);
  for (int i=0; i<2*n_aug_+1; ++i) {
    z_diff_ = Zsig_.col(i) - z_pred_;
    X_diff_ = Xsig_pred_.col(i) - x_;
    while (X_diff_(3) < -M_PI) X_diff_(3) += 2*M_PI;
    while (X_diff_(3) > M_PI) X_diff_(3) -= 2*M_PI;
    
    S_ += weights_(i) * z_diff_ * z_diff_.transpose();
    Tc_ += weights_(i) * X_diff_ * z_diff_.transpose();
  }
  S_ += R_;
  
  K_ = Tc_ * S_.inverse();
  
  z_diff_ = z_ - z_pred_;
  x_ = x_ + K_*(z_diff_);
  P_ = P_ - K_*S_*K_.transpose();
  
  //Calculate Normalized Innovation Sqaured and write to file
  double NIS;
  NIS = z_diff_.transpose() * S_.inverse() * z_diff_;
  ofstream myFile;
  myFile.open ("NIS_LIDAR.txt", ios::out | ios::app | ios::binary);
  myFile << NIS << "\n";
  myFile.close();
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
  MatrixXd Zsig_ = MatrixXd(3, 2*n_aug_+1);
  VectorXd z_pred_ = VectorXd(3);
  MatrixXd S_ = MatrixXd(3,3);
  MatrixXd Tc_ = MatrixXd(n_x_, 3);
  MatrixXd K_ = MatrixXd(n_x_, 3);
  
  MatrixXd R_ = MatrixXd(3,3);
  R_ << std_radr_*std_radr_, 0, 0,
  0, std_radphi_*std_radphi_, 0,
  0, 0, std_radrd_*std_radrd_;
  
  VectorXd z_ = VectorXd(3);
  z_ = meas_package.raw_measurements_;
  
  for (int i=0; i<2*n_aug_+1; ++i) {
    Zsig_(0, i) = sqrt(Xsig_pred_(0,i)*Xsig_pred_(0,i) + Xsig_pred_(1,i)*Xsig_pred_(1,i));
    Zsig_(1, i) = atan2(Xsig_pred_(1,i),Xsig_pred_(0,i));
    Zsig_(2, i) = Xsig_pred_(0,i)*cos(Xsig_pred_(3,i))*Xsig_pred_(2,i);
    Zsig_(2, i) += Xsig_pred_(1,i)*sin(Xsig_pred_(3,i))*Xsig_pred_(2,i);
    Zsig_(2, i) /= sqrt(Xsig_pred_(0,i)*Xsig_pred_(0,i) + Xsig_pred_(1,i)*Xsig_pred_(1,i));
  }
  
  z_pred_.fill(0.);
  for (int i=0; i<2*n_aug_+1; ++i) {
    z_pred_ += weights_(i) * Zsig_.col(i);
  }
  
  VectorXd z_diff_ = VectorXd(3);
  VectorXd X_diff_ = VectorXd(5);
  S_.fill(0.);
  Tc_.fill(0.);
  for (int i=0; i<2*n_aug_+1; ++i) {
    z_diff_ = Zsig_.col(i) - z_pred_;
    while (z_diff_(1) < -M_PI) z_diff_(1) += 2*M_PI;
    while (z_diff_(1) > M_PI) z_diff_(1) -= 2*M_PI;
    
    X_diff_ = Xsig_pred_.col(i) - x_;
    while (X_diff_(3) < -M_PI) X_diff_(3) += 2*M_PI;
    while (X_diff_(3) > M_PI) X_diff_(3) -= 2*M_PI;
    
    S_ += weights_(i) * z_diff_ * z_diff_.transpose();
    Tc_ += weights_(i) * X_diff_ * z_diff_.transpose();
  }
  S_ += R_;
  
  K_ = Tc_ * S_.inverse();
  
  z_diff_ = z_ - z_pred_;
  while (z_diff_(1) < -M_PI) z_diff_(1) += 2*M_PI;
  while (z_diff_(1) > M_PI) z_diff_(1) -= 2*M_PI;
  x_ = x_ + K_*(z_diff_);
  P_ = P_ - K_*S_*K_.transpose();
  
  //Calculate Normalized Innovation Sqaured and write to file
  double NIS;
  NIS = z_diff_.transpose() * S_.inverse() * z_diff_;
  ofstream myFile;
  myFile.open ("NIS_RADAR.txt", ios::out | ios::app | ios::binary);
  myFile << NIS << "\n";
  myFile.close();
}
