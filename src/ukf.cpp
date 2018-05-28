#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
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
  std_a_ = 30;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 30;
  
  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
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
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.
  
  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
  
  // Set state dimension //
  this->n_x_ = 5;
  // Set augmented dimension //
  this->n_aug_ = 7;
  // Define spreading parameter //
  this->lambda_ = 3 - this->n_aug_;
  
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
}


void UKF::GenerateSigmaPoints(MatrixXd* Xsig_out) {
    
    // Create augmented mean vector //
    VectorXd x_aug = VectorXd(this->n_aug_);
    // Create augmented state covariance matrix //
    MatrixXd P_aug;
    P_aug.setZero(this->n_aug_, this->n_aug_);
    // Create sigma point matrix //
    MatrixXd Xsig_aug = MatrixXd(this->n_aug_, 2 * this->n_aug_ + 1);
    
    // Create augmented mean state //
    x_aug.head(5) = this->x_;
    x_aug(5) = 0;
    x_aug(6) = 0;
    
    // Create augmented covariance matrix //
    P_aug.topLeftCorner(this->n_x_, this->n_x_) = this->P_;
    P_aug(5,5) = this->std_a_ * this->std_a_;
    P_aug(6,6) = this->std_yawdd_ * this->std_yawdd_;
    
    //calculate square root of P_aug //
    MatrixXd A_aug = P_aug.llt().matrixL();
    
    // Set first column of sigma point matrix //
    Xsig_aug.col(0)  = x_aug;
    // Set remaining sigma points //
    for (int i = 0; i < n_aug; i++)
    {
	Xsig_aug.col(i+1)     = x_aug + sqrt(this->lambda_ + this->n_aug_) * A_aug.col(i);
	Xsig_aug.col(i+1+this->n_aug_) = x_aug - sqrt(this->lambda_ + this->n_aug_) * A_aug.col(i);
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
    
    // Generate sigma points //
    
    
    
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
}
