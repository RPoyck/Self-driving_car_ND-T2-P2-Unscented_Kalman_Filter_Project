#include "ukf.h"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;


// Constructor //
/**
 * Initialises Unscented Kalman filter
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

  Complete the initialisation. See ukf.h for other member properties.

  Hint: one or more values initialised above might be wildly off...
  */
  
  // Set state dimension //
  this->n_x_ = 5;
  // Set augmented dimension //
  this->n_aug_ = 7;
  // Define spreading parameter //
//   this->lambda_ = 3 - this->n_aug_;
  this->lambda_ = 3 - this->n_x_;
  
  SetWeights();
  
  this->is_initialized_ = false;
  
}


// Destructor //
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
	// first measurement initialisation //
	cout << "UKF: " << endl;
	this->x_ = VectorXd(5);

	if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
	    /**
	    Convert radar from polar to Cartesian coordinates and initialise state.
	    */
	    double rho = meas_package.raw_measurements_[0]; 
	    double phi = meas_package.raw_measurements_[1]; 
	    double rhoDot = meas_package.raw_measurements_[2]; 
	     
	    this->x_(0) = rho * cos(phi); 
	    this->x_(1) = rho * sin(phi); 
	    this->x_(2) = rhoDot * cos(phi); 
	    this->x_(3) = rhoDot * sin(phi); 
	    this->x_(4) = 0.0;
	} 
	else { 
	    if (meas_package.sensor_type_ == MeasurementPackage::LASER) { 
		/** 
		Initialise state. 
		*/ 
		//set the state with the initial location and zero velocity 
		this->x_ << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], 0.0, 0.0, 0.0; 
	    } 
	} 
 
	this->last_timestamp_ = meas_package.timestamp_; 
	 
	// Done initialising, no need to predict or update // 
	is_initialized_ = true; 
	return; 
    }
    
//     if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {return;}
    
    cout << "GenerateSigmaPoints: " << endl;
    GenerateSigmaPoints();
    
    this->delta_t_ = (meas_package.timestamp_ - this->last_timestamp_) / 1000000.0;
    this->last_timestamp_ = meas_package.timestamp_;
    
    cout << "SigmaPointPrediction: " << endl;
    SigmaPointPrediction();
    
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) 
    {
	// Measurement dimension, radar can measure r, phi, and r_dot //
	this->n_z_ = 3;
	cout << "set z rad: " << endl;
	cout << "1: " << meas_package.raw_measurements_[0] << endl;
	cout << "2: " << meas_package.raw_measurements_[1] << endl;
	cout << "3: " << meas_package.raw_measurements_[2] << endl;
	this->z_ = VectorXd(3);
	z_(0) = meas_package.raw_measurements_[0];
	z_(1) = meas_package.raw_measurements_[1];
	z_(2) = meas_package.raw_measurements_[2];
// 	this->z_ << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], meas_package.raw_measurements_[2];
	cout << "z set: " << endl;
    }
    else 
    { 
	if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
	    this->n_z_ = 2;
	    cout << "set z lid: " << endl;
	    this->z_ = VectorXd(2);
	    z_(0) = meas_package.raw_measurements_[0];
	    z_(1) = meas_package.raw_measurements_[1];
// 	    this->z_ << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1];
	    cout << "s set: " << endl;
	}
    }
    
    cout << "PredictMeanAndCovariance: " << endl;
    PredictMeanAndCovariance();
    
    cout << "PredictMeasurement: " << endl;
    PredictMeasurement(meas_package.sensor_type_);
    
    cout << "Update: " << endl;
    Update(meas_package);
    
}


void UKF::GenerateSigmaPoints() {
    
    cout << "Part 1: " << endl;
    // Create augmented mean vector //
    VectorXd x_aug = VectorXd(this->n_aug_);
    // Create augmented state covariance matrix //
    MatrixXd P_aug;
    P_aug.setZero(this->n_aug_, this->n_aug_);
    // Create sigma point matrix //
    MatrixXd Xsig_aug = MatrixXd(this->n_aug_, 2 * this->n_aug_ + 1);
    
    cout << "Part 2: " << endl;
    cout << this->x_ << endl;
    // Create augmented mean state //
    x_aug.head(5) = this->x_;
    x_aug(5) = 0;
    x_aug(6) = 0;
    
    cout << "Part 3: " << endl;
    // Create augmented covariance matrix //
    P_aug.topLeftCorner(this->n_x_, this->n_x_) = this->P_;
    P_aug(5,5) = this->std_a_ * this->std_a_;
    P_aug(6,6) = this->std_yawdd_ * this->std_yawdd_;
    
    cout << "Part 4: " << endl;
    //calculate square root of P_aug //
    MatrixXd A_aug = P_aug.llt().matrixL();
    
    cout << "Part 5: " << endl;
    // Set first column of sigma point matrix //
    Xsig_aug.col(0)  = x_aug;
    // Set remaining sigma points //
    for (int i = 0; i < (this->n_aug_); i++)
    {
	Xsig_aug.col(i+1)     = x_aug + sqrt(this->lambda_ + this->n_aug_) * A_aug.col(i);
	Xsig_aug.col(i+1+this->n_aug_) = x_aug - sqrt(this->lambda_ + this->n_aug_) * A_aug.col(i);
    }
    
    cout << "Part 6: " << endl;
    
    this->Xsig_aug_ = Xsig_aug;
    
}


void UKF::SigmaPointPrediction() {
    
    // Create matrix with predicted sigma points as columns //
    MatrixXd Xsig_pred = MatrixXd(this->n_x_, 2 * this->n_aug_ + 1);
    
    cout << "Part 1: " << endl;
    // Predict sigma points //
    for (int i = 0; i < 2*this->n_aug_+1; i++)
    {
	cout << "size" << Xsig_aug_.size() << endl;
	cout << i << endl;
	// Extract values for better readability //
	double p_x = this->Xsig_aug_(0,i);
	double p_y = this->Xsig_aug_(1,i);
	double v = this->Xsig_aug_(2,i);
	double yaw = this->Xsig_aug_(3,i);
	double yawd = this->Xsig_aug_(4,i);
	double nu_a = this->Xsig_aug_(5,i);
	double nu_yawdd = this->Xsig_aug_(6,i);
	
	cout << "Part 2: " << endl;
	//predicted state values
	double px_p, py_p;
	
	// Avoid division by zero //
	if (fabs(yawd) > 0.001) {
	    px_p = p_x + v/yawd * ( sin (yaw + yawd*this->delta_t_) - sin(yaw) );
	    py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*this->delta_t_) );
	}
	else {
	    px_p = p_x + v*this->delta_t_*cos(yaw);
	    py_p = p_y + v*this->delta_t_*sin(yaw);
	}
	
	cout << "Part 3: " << endl;
	double v_p = v;
	double yaw_p = yaw + yawd*this->delta_t_;
	double yawd_p = yawd;
	
	cout << "Part 4: " << endl;
	//add noise effect //
	px_p += 0.5 * nu_a * this->delta_t_*this->delta_t_ * cos(yaw);
	py_p += 0.5 * nu_a * this->delta_t_*this->delta_t_ * sin(yaw);
	v_p += nu_a * this->delta_t_;
	
	yaw_p += 0.5 * nu_yawdd * this->delta_t_*this->delta_t_;
	yawd_p += nu_yawdd * this->delta_t_;
	
	cout << "Part 5: " << endl;
	// Write predicted sigma point into the column of this sigma point number //
	Xsig_pred(0,i) = px_p;
	Xsig_pred(1,i) = py_p;
	Xsig_pred(2,i) = v_p;
	Xsig_pred(3,i) = yaw_p;
	Xsig_pred(4,i) = yawd_p;
    }
    
    cout << "Part 6: " << endl;
    this->Xsig_pred_ = Xsig_pred;
    
}


void UKF::PredictMeanAndCovariance() {
    
    // Create vector for predicted state //
    VectorXd x; // = VectorXd(this->n_x_);
    x.setZero(this->n_x_);
    
    // Create covariance matrix for prediction //
    MatrixXd P; // = MatrixXd(n_x, n_x);
    P.setZero(this->n_x_, this->n_x_);
    
    cout << "Part 1: " << endl;
    // Predict state mean //
    for (int i = 0; i < 2 * this->n_aug_ + 1; i++) {  // iterate over sigma points
        x += this->weights_(i) * this->Xsig_pred_.col(i);
    }
    
    cout << "Part 2: " << endl;
    // Predict state covariance matrix //
    for (int i = 0; i < 2 * this->n_aug_ + 1; i++) {  //iterate over sigma points

        // State difference //
        VectorXd x_diff = this->Xsig_pred_.col(i) - x;
	cout << "x_diff: " << x_diff << endl;
	cout << "this->Xsig_pred_.col(i): " << this->Xsig_pred_.col(i) << endl;
	cout << "x: " << x << endl;
        // Angle formalisation //
	cout << "x_diff(3): " << x_diff(3) << endl;
        while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
        while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;
    
        P += this->weights_(i) * x_diff * x_diff.transpose() ;
    }
    cout << "Part 3: " << endl;
    
    this->x_ = x;
    this->P_ = P;
    
}


void UKF::PredictMeasurement(MeasurementPackage::SensorType sensor_type) {
    
    // Create matrix for sigma points in measurement space //
    MatrixXd Zsig = MatrixXd(this->n_z_, 2 * this->n_aug_ + 1);
    
    // Mean predicted measurement //
    VectorXd z_pred; // = VectorXd(this->this->n_z_);
    z_pred.setZero(this->n_z_);
    
    // Measurement covariance matrix //
    MatrixXd S; // = MatrixXd(n_z,n_z);
    S.setZero(this->n_z_, this->n_z_);
    
    
    // Transform sigma points into measurement space //
    for (int i = 0; i < 2 * this->n_aug_ + 1; i++) // 2*n+1 sigma points //
    {  
	
	// Extract values for better readability //
	double p_x = this->Xsig_pred_(0,i);
	double p_y = this->Xsig_pred_(1,i);
	
	if (sensor_type == MeasurementPackage::RADAR) 
	{
	    double v  = this->Xsig_pred_(2,i);
	    double yaw = this->Xsig_pred_(3,i);
	    double v1 = cos(yaw)*v;
	    double v2 = sin(yaw)*v;

	    // Measurement model //
	    Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);                        // r  //
	    Zsig(1,i) = atan2(p_y,p_x);                                 // phi //
	    Zsig(2,i) = (p_x*v1 + p_y*v2 ) / sqrt(p_x*p_x + p_y*p_y);   // r_dot //
	}
	else 
	{ 
	    if (sensor_type == MeasurementPackage::LASER) 
	    {
		Zsig(0,i) = p_x;
		Zsig(1,i) = p_y;
	    }
	}
    }    
  
    // Mean predicted measurement //
    for (int i=0; i < 2*this->n_aug_+1; i++) {
	z_pred += this->weights_(i) * Zsig.col(i);
    }
    
    // Innovation covariance matrix S //
    for (int i = 0; i < 2 * this->n_aug_ + 1; i++) {  // 2*n+1 sigma points //
	// Residual //
	VectorXd z_diff = Zsig.col(i) - z_pred;

	// Angle normalisation //
	while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
	while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

	S += this->weights_(i) * z_diff * z_diff.transpose();
    }
    
    //add measurement noise covariance matrix
    MatrixXd R = MatrixXd(this->n_z_, this->n_z_);
    if (sensor_type == MeasurementPackage::RADAR) 
    {
	R <<    this->std_radr_*this->std_radr_, 	0, 					0,
		0, 					this->std_radphi_*this->std_radphi_, 	0,
		0, 					0,					this->std_radrd_*this->std_radrd_;
    }
    else 
    { 
	if (sensor_type == MeasurementPackage::LASER) 
	{
	    R <<    this->std_laspx_*this->std_laspx_, 	0,
		    0, 					this->std_laspy_*this->std_laspy_;
	}
    }
    
    S += R;
    
    // Output S and z_pred //
    this->Zsig_ = Zsig;
    this->z_pred_ = z_pred;
    this->S_ = S;
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
// void UKF::UpdateRadar(MeasurementPackage meas_package) {
void UKF::Update(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
    
    // Create matrix for cross correlation Tc //
    MatrixXd Tc; // = MatrixXd(n_x, n_z);
    Tc.setZero(this->n_x_, this->n_z_);
    
    // Calculate cross correlation matrix //
//     Tc.fill(0.0);
    for (int i = 0; i < 2 * this->n_aug_ + 1; i++) {  //2*n+1 sigma points

	// Residual //
	VectorXd z_diff = this->Zsig_.col(i) - this->z_pred_;
	// Angle normalisation //
	while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
	while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

	// state difference
	VectorXd x_diff = this->Xsig_pred_.col(i) - this->x_;
	//angle normalisation
	while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
	while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

	Tc = Tc + this->weights_(i) * x_diff * z_diff.transpose();
	
    }
    
    // Kalman gain K //
    MatrixXd K = Tc * this->S_.inverse();
    
    // Residual //
    VectorXd z_diff = this->z_ - this->z_pred_;
    
    // Angle normalisation //
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
    
    // Update state mean and covariance matrix //
    this->x_ += K * z_diff;
    this->P_ -= K*this->S_*K.transpose();
    
}

void UKF::SetWeights() 
{
    // Create vector for weights //
    VectorXd weights = VectorXd(2*this->n_aug_+1);
    // Set weights // 
    weights(0) = this->lambda_ / (this->lambda_ + this->n_aug_);
    for (int i = 1; i < (2 * this->n_aug_ + 1); i++)
    {
	weights(i) = 1 / (2 * (this->lambda_ + this->n_aug_));
    }
    this->weights_ = weights;
}
