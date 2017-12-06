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
    
  // initialize sigma point prediction matrix
  Xsig_pred_ = MatrixXd(5, 15);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  expected_laser_nis_=3;
  expected_radar_nis_=4;
  //std_a_ = 5; //original, now will change according to NIS
  //Value used for both was actually 0.2
  std_a_ = 2;
  // Process noise standard deviation yaw acceleration in rad/s^2
  //std_yawdd_ = 5;
  std_yawdd_ = 2;

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

  // Parameters above this line are scaffolding, do not modify
  time_us_ = 0; //TODO: update this to fill with timestamp of reading

  // Previous Zk+1, for calculation of NIS
  previous_zkp1_radar=VectorXd(3);
  previous_zkp1_laser=VectorXd(2);

  is_initialized_ = false;
  
  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
    
    //set example state
    x_ <<   0.0,
    0.0,
    0.0,
    0.0,
    0.0;

    previous_zkp1_laser << 0.0,
    0.0;

    previous_zkp1_radar << 0.0,
    0.0,
    0.0;
    
    
    P_ = MatrixXd::Identity(5, 5);
    //P_.fill(1.0);

}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
/*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    cout << "UKF: " << endl;
    //x_ << 1, 1, 1, 1, 1;

    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
        cout << "Initializing with Radar packet: " << endl;

        double rho = meas_package.raw_measurements_[0];
        double phi = meas_package.raw_measurements_[1];
        double rho_dot = meas_package.raw_measurements_[2];

        double px = rho * cos(phi);
        double py = rho * sin(phi);
        double vel_abs = rho_dot;//This is incorrect, since vel_abs is vehicles velocity and rho_dot is vehicles radial velocity with resect to us, still good approximation
        double yaw_angle = M_PI;
        double yaw_rate = 0;
        

        x_ << px, py, vel_abs, yaw_angle, yaw_rate;

    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
        /**
         Initialize state.
        */

        cout << "Initializing with Laser packet: " << endl;

        double px = meas_package.raw_measurements_[0];
        double py = meas_package.raw_measurements_[1];
        //double yaw_angle = atan2 (px,py);

        x_ << px, py, 0, 0, 0;
    }

    // done initializing, no need to predict or update
      
    time_us_ = meas_package.timestamp_;
    is_initialized_ = true;

    return;
      
  }

  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
  std::cout << "Measurement Arrived, Processing" << std::endl;

  if (meas_package.sensor_type_==MeasurementPackage::RADAR)
  {
    std::cout << "It is a Radar measurement" << std::endl;
    double delta_t = (meas_package.timestamp_-time_us_)/1e6;
    UKF::Prediction(delta_t);
    UKF::UpdateRadar(meas_package);
    time_us_=meas_package.timestamp_;

  }
  if (meas_package.sensor_type_==MeasurementPackage::LASER)
  {
    std::cout << "It is a Laser measurement" << std::endl;
    double delta_t = (meas_package.timestamp_-time_us_)/1e6;
    UKF::Prediction(delta_t);
    UKF::UpdateLidar(meas_package);
    time_us_=meas_package.timestamp_;
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
    
    
    /*****************
     Generate Sigma Points
     *****************/
    std::cout << "Generating Sigma Points" << std::endl;

    //set state dimension
    int n_x = 5;
    
    //define spreading parameter
    double lambda = 3 - n_x;
    
    //create sigma point matrix
    MatrixXd Xsig = MatrixXd(n_x, 2 * n_x + 1);
    
    //calculate square root of P
    MatrixXd A = P_.llt().matrixL();
    
    //set first column of sigma point matrix
    std::cout << "Previously predicted mean x_ " << x_ << std::endl;

    Xsig.col(0)  = x_;

    std::cout << "Previously predicted covariance matrix P " << P_ << std::endl;

    
    //set remaining sigma points
    for (int i = 0; i < n_x; i++)
    {
        Xsig.col(i+1)     = x_ + sqrt(lambda+n_x) * A.col(i);
        Xsig.col(i+1+n_x) = x_ - sqrt(lambda+n_x) * A.col(i);
    }

    std::cout << "Sigma points generated: " << std::endl << Xsig << std::endl;

    
    /*****************
     Augment vector with noise estimation
     *****************/
    std::cout << "Augmenting with noise" << std::endl;

    //set augmented dimension
    int n_aug = 7;
    
    //Process noise standard deviation longitudinal acceleration in m/s^2
    double std_a = std_a_;
    
    //Process noise standard deviation yaw acceleration in rad/s^2
    double std_yawdd = std_yawdd_;
    
    //define spreading parameter
    lambda = 3 - n_aug;
    
    //create augmented mean vector
    VectorXd x_aug = VectorXd(7);
    
    //create augmented state covariance
    MatrixXd P_aug = MatrixXd(7, 7);
    
    //create sigma point matrix
    MatrixXd Xsig_aug = MatrixXd(n_aug, 2 * n_aug + 1);
    
    //create augmented mean state
    x_aug.head(5) = x_;
    x_aug(5) = 0;
    x_aug(6) = 0;
    
    //create augmented covariance matrix
    P_aug.fill(0.0);
    P_aug.topLeftCorner(5,5) = P_;
    P_aug(5,5) = std_a*std_a;
    P_aug(6,6) = std_yawdd*std_yawdd;
    
    //create square root matrix
    MatrixXd L = P_aug.llt().matrixL();
    
    //create augmented sigma points
    Xsig_aug.col(0)  = x_aug;
    for (int i = 0; i< n_aug; i++)
    {
        Xsig_aug.col(i+1)       = x_aug + sqrt(lambda+n_aug) * L.col(i);
        Xsig_aug.col(i+1+n_aug) = x_aug - sqrt(lambda+n_aug) * L.col(i);
    }

    std::cout << "Finished augmenting with noise" << std::endl;
    std::cout << "Augmented mean state vector: " << std::endl;
    std::cout << x_aug << std::endl;
    
    std::cout << "Augmented covariance matrix: " << std::endl;
    std::cout << P_aug << std::endl;


    
    /*****************
     Predict new Sigma points
     *****************/
    std::cout << "Predicting New Sigma points" << std::endl;

    //create matrix with predicted sigma points as columns
    //No need to, it is already stored as a member variable
    
    //predict sigma points
    for (int i = 0; i< 2*n_aug+1; i++)
    {
        //extract values for better readability
        double p_x = Xsig_aug(0,i);
        double p_y = Xsig_aug(1,i);
        double v = Xsig_aug(2,i);
        double yaw = Xsig_aug(3,i);
        double yawd = Xsig_aug(4,i);
        double nu_a = Xsig_aug(5,i);
        double nu_yawdd = Xsig_aug(6,i);
        
        //predicted state values
        double px_p, py_p;
        
        //std::cout << "Finished one loop" << i  <<std::endl;

        
        //avoid division by zero
        if (fabs(yawd) > 0.001) {
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
        Xsig_pred_(0,i) = px_p;
        Xsig_pred_(1,i) = py_p;
        Xsig_pred_(2,i) = v_p;
        Xsig_pred_(3,i) = yaw_p;
        Xsig_pred_(4,i) = yawd_p;
        

    }
    std::cout << "Finished predicting new sigma points" << std::endl;
    std::cout << Xsig_pred_ << std::endl;

    
    // set weights
    //create vector for weights
    VectorXd weights = VectorXd(2*n_aug+1);
    
    double weight_0 = lambda/(lambda+n_aug);
    weights(0) = weight_0;
    for (int i=1; i<2*n_aug+1; i++) {  //2n+1 weights
        double weight = 0.5/(n_aug+lambda);
        weights(i) = weight;
    }
    std::cout << "Predicting New State Mean and covariance" << std::endl;

    //predicted state mean
    x_.fill(0.0);
    for (int i = 0; i < 2 * n_aug + 1; i++) {  //iterate over sigma points
        x_ = x_+ weights(i) * Xsig_pred_.col(i);
    }
    std::cout << "Finished predicting mean" << std::endl;
    std::cout << x_ << std::endl;

    //predicted state covariance matrix
    P_.fill(0.0);
    for (int i = 0; i < 2 * n_aug + 1; i++) {  //iterate over sigma points
        
        // state difference
        VectorXd x_diff = Xsig_pred_.col(i) - x_;
        //angle normalization

        while (x_diff(3)> M_PI) {
            x_diff(3)-=2.*M_PI;
            std::cout << "Need to normalize angle because it is > 2*Pi" << std::endl;
            std::cout << "x_diff(3)" << x_diff(3) << std::endl;

        }
        while (x_diff(3)<-M_PI){
            x_diff(3)+=2.*M_PI;
            std::cout << "Need to normalize angle because it is > 2*Pi" << std::endl;
            std::cout << "x_diff(3)" << x_diff(3) << std::endl;
        } 
        P_ = P_ + weights(i) * x_diff * x_diff.transpose() ;
    }
    std::cout << "Finished predicting covariance" << std::endl;
    std::cout << P_ << std::endl;

    
    
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

    //set state dimension
    std::cout << "Predicting Laser measurement" << std::endl;

    int n_x = 5;
    
    //set augmented dimension
    int n_aug = 7;
    
    //set measurement dimension, radar can measure px, py
    int n_z = 2;
    
    //define spreading parameter
    double lambda = 3 - n_aug;
    
    //set vector for weights
    VectorXd weights = VectorXd(2*n_aug+1);
    double weight_0 = lambda/(lambda+n_aug);
    weights(0) = weight_0;
    for (int i=1; i<2*n_aug+1; i++) {
        double weight = 0.5/(n_aug+lambda);
        weights(i) = weight;
    }
    
    //radar measurement noise standard deviation radius in m
    double std_laspx = std_laspx_;  
    
    //radar measurement noise standard deviation angle in rad
    double std_laspy = std_laspy_;
    

    //create matrix for sigma points in measurement space
    MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug + 1);
    
    std::cout << "Transforming Sigma Points into measurement space" << std::endl;

    //transform sigma points into measurement space
    for (int i = 0; i < 2 * n_aug + 1; i++) {  //2n+1 simga points
        
        // extract values for better readibility
        double p_x = Xsig_pred_(0,i);
        double p_y = Xsig_pred_(1,i);
        double v  = Xsig_pred_(2,i);
        double yaw = Xsig_pred_(3,i);
        
        double v1 = cos(yaw)*v;
        double v2 = sin(yaw)*v;
        
        // measurement model
        Zsig(0,i) = p_x;                        //r
        Zsig(1,i) = p_y;                                 //phi
        
    }
    
    std::cout << "Extracting mean predicted measurement and covariance matrix" << std::endl;

    //mean predicted measurement
    VectorXd z_pred = VectorXd(n_z);
    z_pred.fill(0.0);
    for (int i=0; i < 2*n_aug+1; i++) {
        z_pred = z_pred + weights(i) * Zsig.col(i);
    }
    
    //measurement covariance matrix S
    MatrixXd S = MatrixXd(n_z,n_z);
    S.fill(0.0);
    for (int i = 0; i < 2 * n_aug + 1; i++) {  //2n+1 simga points
        //residual
        VectorXd z_diff = Zsig.col(i) - z_pred;
        
        //angle normalization
        while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
        while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
        
        S = S + weights(i) * z_diff * z_diff.transpose();
    }
    
    //add measurement noise covariance matrix
    MatrixXd R = MatrixXd(n_z,n_z);
    R <<    std_laspx*std_laspx, 0,
    0, std_laspy*std_laspy;
    S = S + R;
    

    //create example vector for incoming radar measurement
    VectorXd z = meas_package.raw_measurements_;

    
    //create matrix for cross correlation Tc
    MatrixXd Tc = MatrixXd(n_x, n_z);
    
    /*******************************************************************************
     * Student part begin
     ******************************************************************************/
    std::cout << "Updating state with new measurement" << std::endl;


    //calculate cross correlation matrix
    Tc.fill(0.0);
    for (int i = 0; i < 2 * n_aug + 1; i++) {  //2n+1 simga points
        
        //residual
        VectorXd z_diff = Zsig.col(i) - z_pred;
        //angle normalization
        while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
        while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
        
        // state difference
        VectorXd x_diff = Xsig_pred_.col(i) - x_;
        //angle normalization
        while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
        while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;
        
        Tc = Tc + weights(i) * x_diff * z_diff.transpose();
    }
    
    //Kalman gain K;
    MatrixXd K = Tc * S.inverse();
    
    //residual
    VectorXd z_diff = z - z_pred;
    
    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
    
    //update state mean and covariance matrix
    x_ = x_ + K * z_diff;
    P_ = P_ - K*S*K.transpose();

    //calculate NIS
    VectorXd predictionError = z_pred - previous_zkp1_laser; 
    std::cout << "predictionError: " <<std::endl<< predictionError <<std::endl;
    std::cout << "S.inverse(): " <<std::endl<< S.inverse() <<std::endl;

    double NIS = predictionError.transpose()*S.inverse()*predictionError;
    std::cout << "Laser NIS: " << NIS <<std::endl;
    /* if (NIS>expected_laser_nis_)
    {
        std_a_ += 0.1;
        std_yawdd_ += 0.1;
    }
        else
    {
        std_a_ -= 0.1;
        std_yawdd_ -= 0.1;
    } */
    std::cout << "std_a_ : " << std_a_ <<std::endl;
    std::cout << "std_yawdd_ : " << std_yawdd_ <<std::endl;
    previous_zkp1_laser = z_pred;
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
    //set state dimension
    std::cout << "Predicting radar measurement" << std::endl;

    int n_x = 5;
    
    //set augmented dimension
    int n_aug = 7;
    
    //set measurement dimension, radar can measure r, phi, and r_dot
    int n_z = 3;
    
    //define spreading parameter
    double lambda = 3 - n_aug;
    
    //set vector for weights
    VectorXd weights = VectorXd(2*n_aug+1);
    double weight_0 = lambda/(lambda+n_aug);
    weights(0) = weight_0;
    for (int i=1; i<2*n_aug+1; i++) {
        double weight = 0.5/(n_aug+lambda);
        weights(i) = weight;
    }
    
    //radar measurement noise standard deviation radius in m
    double std_radr = std_radr_;  
    
    //radar measurement noise standard deviation angle in rad
    double std_radphi = std_radphi_;
    
    //radar measurement noise standard deviation radius change in m/s
    double std_radrd = std_radrd_;

    //create matrix for sigma points in measurement space
    MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug + 1);
    
    std::cout << "Transforming Sigma Points into measurement space" << std::endl;

    //transform sigma points into measurement space
    for (int i = 0; i < 2 * n_aug + 1; i++) {  //2n+1 simga points
        
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
    
    std::cout << "Extracting mean predicted measurement and covariance matrix" << std::endl;

    //mean predicted measurement
    VectorXd z_pred = VectorXd(n_z);
    z_pred.fill(0.0);
    for (int i=0; i < 2*n_aug+1; i++) {
        z_pred = z_pred + weights(i) * Zsig.col(i);
    }
    
    //measurement covariance matrix S
    MatrixXd S = MatrixXd(n_z,n_z);
    S.fill(0.0);
    for (int i = 0; i < 2 * n_aug + 1; i++) {  //2n+1 simga points
        //residual
        VectorXd z_diff = Zsig.col(i) - z_pred;
        
        //angle normalization
        while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
        while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
        
        S = S + weights(i) * z_diff * z_diff.transpose();
    }
    
    //add measurement noise covariance matrix
    MatrixXd R = MatrixXd(n_z,n_z);
    R <<    std_radr*std_radr, 0, 0,
    0, std_radphi*std_radphi, 0,
    0, 0,std_radrd*std_radrd;
    S = S + R;
    

    //create example vector for incoming radar measurement
    VectorXd z = meas_package.raw_measurements_;

    
    //create matrix for cross correlation Tc
    MatrixXd Tc = MatrixXd(n_x, n_z);
    
    /*******************************************************************************
     * Student part begin
     ******************************************************************************/
    std::cout << "Updating state with new measurement" << std::endl;


    //calculate cross correlation matrix
    Tc.fill(0.0);
    for (int i = 0; i < 2 * n_aug + 1; i++) {  //2n+1 simga points
        
        //residual
        VectorXd z_diff = Zsig.col(i) - z_pred;
        //angle normalization
        while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
        while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
        
        // state difference
        VectorXd x_diff = Xsig_pred_.col(i) - x_;
        //angle normalization
        while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
        while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;
        
        Tc = Tc + weights(i) * x_diff * z_diff.transpose();
    }
    
    //Kalman gain K;
    MatrixXd K = Tc * S.inverse();
    
    //residual
    VectorXd z_diff = z - z_pred;
    
    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
    
    //update state mean and covariance matrix
    x_ = x_ + K * z_diff;
    P_ = P_ - K*S*K.transpose();

    //calculate NIS
    VectorXd predictionError = z_pred - previous_zkp1_radar; 
    std::cout << "predictionError: " <<std::endl<< predictionError <<std::endl;
    std::cout << "S.inverse(): " <<std::endl<< S.inverse() <<std::endl;

    double NIS = predictionError.transpose()*S.inverse()*predictionError;
    std::cout << "Radar NIS: " << NIS <<std::endl;
/*      if (NIS>expected_radar_nis_)
    {
        std_a_ += 0.1;
        std_yawdd_ += 0.1;
    }
    else
    {
        std_a_ -= 0.1;
        std_yawdd_ -= 0.1;
    }  */
    std::cout << "std_a_ : " << std_a_ <<std::endl;
    std::cout << "std_yawdd_ : " << std_yawdd_ <<std::endl;


    previous_zkp1_radar = z_pred;
}
