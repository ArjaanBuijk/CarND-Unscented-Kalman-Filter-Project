#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>
#include <iomanip>      // std::setw
#include "test.h"
#include "tools.h"

// uncomment this if you want to run the unit-tests
// also uncomment unit_test1.h below
// do NOT use the unit_tests during an actual run.
//#include "unit_test2.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

bool AB_DEBUG = false;

Tools tools;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF(std::ofstream & ofs) : ofs_(ofs)
{
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored
  use_radar_ = true;

  // Process noises, tuned for best result
  std_a_     = 0.8;
  std_yawdd_ = 0.4;

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

  // Flag to check where we are in state vector initialization
  is_initialized_ = 0;


  // Timestamp
  previous_timestamp_=0.0;

  // set state dimension
  // [px py vel_abs yaw_angle yaw_rate]
  n_x_ = 5;

  // set augmented state dimension
  // + process noise for longitudinal acceleration (std_a_)
  // + process noise for yaw acceleration (std_yawdd_)
  n_aug_ = 7;

  // set sigma point spreading parameter
  lambda_ = 3 - n_aug_;

  // set radar measurement dimension [r, phi, and r_dot]
  n_z_radar_ = 3;

  // set lidar measurement dimension [px. py]
  n_z_lidar_ = 2;

  // initial state vector
  // state vector: [px py vel_abs yaw_angle yaw_rate] in SI units and rad
  // these will be initialized after first measurements have been received.
  x_ = VectorXd(n_x_);
  x_.fill(0.0);

  // initial state covariance matrix
  // See Lesson 7.32
  // Diagonal values represent the variances for each of the state variables:
  //  [std_px^2, std_py^2, std_v^2, std_psi^2, std_psi_dot^2]
  // let's use lidar noise standard deviation measurement as the basis for initialization
  P_ = MatrixXd(n_x_, n_x_);
  P_.fill(0.0);
  P_(0,0) = std_laspx_*std_laspx_;
  P_(1,1) = std_laspy_*std_laspy_;
  P_(2,2) = P_(0,0);
  P_(3,3) = P_(0,0);
  P_(4,4) = P_(0,0);

  //Predicted sigma points as columns (Lesson 7.20)
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);
  Xsig_pred_.fill(0.0);

  // Weights (Lesson 7.23)
  weights_ = VectorXd(2*n_aug_+1);
  weights_.fill(0.0);

  // set weights (Lesson 7.23)
  weights_(0) = lambda_/(lambda_+n_aug_);
  for (int j=1; j<Xsig_pred_.cols(); ++j){ // loop over the predicted sigma points. Each stored in a column of the Xsig_pred matrix.
    weights_(j) = 0.5/(lambda_+n_aug_);
  }

  //un-comment this if you want to run the unit tests
  //#include "unit_test1.h"

  // counter of measurements
  count_=-1;

  // NIS of lidar & radar
  NIS_lidar_ = 0.0;
  NIS_radar_ = 0.0;

  // write header to ukf output file
  if (ofs_)
    output_write_header();
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package)
{
  ++count_;

  if (count_==0){
    // first measurement, store timestamp for output purposes
    timestamp0_ = meas_package.timestamp_;
    previous_timestamp_ = meas_package.timestamp_;
  }

  time_ = (meas_package.timestamp_ - timestamp0_) / 1000000.0;  //time - expressed in seconds
  dt_=(meas_package.timestamp_ - previous_timestamp_) / 1000000.0;  //dt - expressed in seconds
  previous_timestamp_ = meas_package.timestamp_;

  /****************************************************************************************
  *  Initialization
  *****************************************************************************************/
  if (is_initialized_<2) {
    // still initializing
    double px  = 0.0;
    double py  = 0.0;
    double vel = 0.0;
    double yaw = 0.0;

    double vx  = 0.0;
    double vy  = 0.0;

    extract_measured_positions(meas_package,px,py);

    if (is_initialized_>0){
      // second measurement, use dx/dt for velocity estimate
      vx  = (px-x_[0])/dt_;
      vy  = (py-x_[1])/dt_;
      vel = sqrt(vx*vx+vy*vy);
      yaw = atan2(vy, vx);
    }
    x_[0] = px;                // px
    x_[1] = py;                // py
    x_[2] = vel;               // vel_abs
    x_[3] = yaw;               // yaw_angle
    x_[4] = 0.0;               // yaw_rate

    // skip the first measurement for velocity initialization.
    // it is very inaccurate
    if (count_ > 0)
      ++is_initialized_;

    // Note: no need to predict or update as part of initialization
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  Prediction();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_)
      UpdateRadar(meas_package);
  else if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_)
      UpdateLidar(meas_package);
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} dt the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction()
{
  /** Lesson 7.17: Create augmented sigma points at time k */
  MatrixXd Xsig_aug = create_augmented_sigma_points();

  /** Lesson 7.20: Calculate sigma points at time k+1*/
  predict_sigma_points(Xsig_aug);

  /** Lesson 7.23: Calculate mean and covariance of the predicted sigma points at time k+1*/
  predict_mean_and_covariance();
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package)
{
  /** Lesson 7.26: Predict radar measurement at time k+1*/
  //create matrix for sigma points in measurement space [px, py]
  MatrixXd Zsig = MatrixXd(n_z_lidar_, 2 * n_aug_ + 1);

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z_lidar_);

  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z_lidar_,n_z_lidar_);

  predict_lidar_measurement(Zsig, z_pred, S);

  /** Lesson 7.29: Update state vector & covariance matrix */
  update_lidar_measurement(Zsig, z_pred, S, meas_package.raw_measurements_);

  /** Lesson 7.31: Do consistency check, by calculating NIS */
  NIS_lidar_ = calculate_NIS_lidar(z_pred,S,meas_package.raw_measurements_);

}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package)
{
  /** Lesson 7.26: Predict radar measurement at time k+1*/
  //create matrix for sigma points in measurement space [r, phi, r_dot]
  MatrixXd Zsig = MatrixXd(n_z_radar_, 2 * n_aug_ + 1);

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z_radar_);

  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z_radar_,n_z_radar_);

  predict_radar_measurement(Zsig, z_pred, S);

  /** Lesson 7.29: Update state vector & covariance matrix */
  update_radar_measurement(Zsig, z_pred, S, meas_package.raw_measurements_);

  /** Lesson 7.31: Do consistency check, by calculating NIS */
  NIS_radar_ = calculate_NIS_radar(z_pred,S,meas_package.raw_measurements_);
}

/** Lesson 7.17: Create augmented sigma points */
MatrixXd UKF::create_augmented_sigma_points()
{
  //create augmented mean vector
  VectorXd x_aug = VectorXd(n_aug_);

  //create augmented state covariance
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);

  //create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);

  //create augmented mean state
  x_aug.fill(0.0);
  x_aug.head(n_x_) = x_;

  //create augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(n_x_,n_x_) = P_;
  P_aug(5,5) = std_a_*std_a_;
  P_aug(6,6) = std_yawdd_*std_yawdd_;

  //create square root matrix
  MatrixXd L = P_aug.llt().matrixL();

  //create augmented sigma points
  Xsig_aug.col(0)  = x_aug;
  for (int i = 0; i< n_aug_; i++)
  {
    Xsig_aug.col(i+1)        = x_aug + sqrt(lambda_+n_aug_) * L.col(i);
    Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_+n_aug_) * L.col(i);
  }

  return Xsig_aug;
}

/** Lesson 7.20: Calculate sigma points at time k+1*/
void UKF::predict_sigma_points(const MatrixXd& Xsig_aug)
{
  //predict sigma points
  //avoid division by zero
  //write predicted sigma points into right column

  for (int j=0; j<Xsig_aug.cols(); ++j){
    //extract values for better readability
    double p_x      = Xsig_aug(0,j);
    double p_y      = Xsig_aug(1,j);
    double v        = Xsig_aug(2,j);
    double yaw      = Xsig_aug(3,j);
    double yawd     = Xsig_aug(4,j);
    double nu_a     = Xsig_aug(5,j);
    double nu_yawdd = Xsig_aug(6,j);

    //predicted state values
    double px_p, py_p;

    //avoid division by zero
    if (fabs(yawd) > 0.001) {
        px_p = p_x + v/yawd * ( sin (yaw + yawd*dt_) - sin(yaw));
        py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*dt_) );
    }
    else {
        px_p = p_x + v*dt_*cos(yaw);
        py_p = p_y + v*dt_*sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd*dt_;
    double yawd_p = yawd;

    //add noise
    px_p = px_p + 0.5*nu_a*dt_*dt_ * cos(yaw);
    py_p = py_p + 0.5*nu_a*dt_*dt_ * sin(yaw);
    v_p = v_p + nu_a*dt_;

    yaw_p = yaw_p + 0.5*nu_yawdd*dt_*dt_;
    yawd_p = yawd_p + nu_yawdd*dt_;

    //write predicted sigma point into right column
    Xsig_pred_(0,j) = px_p;
    Xsig_pred_(1,j) = py_p;
    Xsig_pred_(2,j) = v_p;
    Xsig_pred_(3,j) = yaw_p;
    Xsig_pred_(4,j) = yawd_p;
  }
}

/** Lesson 7.23: Calculate mean and covariance of the predicted sigma points at time k+1*/
void UKF::predict_mean_and_covariance()
{
  //predicted state mean
  x_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
    x_ = x_+ weights_(i) * Xsig_pred_.col(i);
  }

  //predicted state covariance matrix
  P_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    normalize_angle(x_diff(3));

    P_ = P_ + weights_(i) * x_diff * x_diff.transpose() ;
  }
}

/** Lesson 7.26: Predict radar measurement at time k+1*/
void UKF::predict_radar_measurement(MatrixXd& Zsig, VectorXd& z_pred, MatrixXd& S)
{
  //transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

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
  for (int i=0; i < 2*n_aug_+1; i++) {
      z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  //measurement covariance matrix S
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    normalize_angle(z_diff(1));

    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  //add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z_radar_,n_z_radar_);
  R <<    std_radr_*std_radr_, 0                      , 0,
          0                  , std_radphi_*std_radphi_, 0,
          0                  , 0                      ,std_radrd_*std_radrd_;
  S = S + R;
}

/** Lesson 7.29: Update state vector & covariance matrix */
void UKF::update_radar_measurement(const MatrixXd& Zsig,
                                   const VectorXd& z_pred,
                                   const MatrixXd& S,
                                   const VectorXd& z)
{
  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z_radar_);

  //calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    normalize_angle(z_diff(1));

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    normalize_angle(x_diff(3));

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  //Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  //residual
  VectorXd z_diff = z - z_pred;
  normalize_angle(z_diff(1));

  //update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  normalize_angle(x_(3));
  P_ = P_ - K*S*K.transpose();

}

/** Lesson 7.31: Do consistency check, by calculating NIS */
double UKF::calculate_NIS_radar(const VectorXd& z_pred,
                                const MatrixXd& S,
                                const VectorXd& z)
{
  //residual
  VectorXd z_diff = z - z_pred;
  normalize_angle(z_diff(1));
  double NIS = z_diff.transpose() * S.inverse() * z_diff;
  return NIS;
}

/** Lesson 7.26: Predict radar measurement at time k+1*/
void UKF::predict_lidar_measurement(MatrixXd& Zsig, VectorXd& z_pred, MatrixXd& S)
{
  //transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
    // measurement model
    Zsig(0,i) = Xsig_pred_(0,i);              //px
    Zsig(1,i) = Xsig_pred_(1,i);              //py
  }

  //mean predicted measurement
  z_pred.fill(0.0);
  for (int i=0; i < 2*n_aug_+1; i++) {
      z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  //measurement covariance matrix S
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  //add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z_lidar_,n_z_lidar_);
  R <<    std_laspx_*std_laspx_, 0                      ,
          0                    , std_laspy_*std_laspy_;
  S = S + R;
}

/** Lesson 7.29: Update state vector & covariance matrix */
void UKF::update_lidar_measurement(const MatrixXd& Zsig,
                                   const VectorXd& z_pred,
                                   const MatrixXd& S,
                                   const VectorXd& z)
{
  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z_lidar_);

  //calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    normalize_angle(x_diff(3));

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  //Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  //residual
  VectorXd z_diff = z - z_pred;

  //update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  normalize_angle(x_(3));
  P_ = P_ - K*S*K.transpose();
}

/** Lesson 7.31: Do consistency check, by calculating NIS */
double UKF::calculate_NIS_lidar(const VectorXd& z_pred,
                                const MatrixXd& S,
                                const VectorXd& z)
{
  //residual
  VectorXd z_diff = z - z_pred;
  double NIS = z_diff.transpose() * S.inverse() * z_diff;
  return NIS;
}

/** Function to normalize the angle */
void UKF::normalize_angle( double& angle )
{
  // Normalize the angle to be within -pi, pi
  angle = atan2(sin(angle), cos(angle));
}

/** Function to extract px and py from a radar or lidar measurement */
void UKF::extract_measured_positions(const MeasurementPackage meas_package,
                                     double& px,double& py)
{
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    double r     = meas_package.raw_measurements_[0];
    double phi   = meas_package.raw_measurements_[1];
    //double r_dot = meas_package.raw_measurements_[2];
    //note: vx & vy can not be determined from a radar measurement
    //      only the velocity in the direction of r is known, but that
    //      is not the total velocity.

    px = r*cos(phi);
    py = r*sin(phi);
  }
  else {
    px = meas_package.raw_measurements_[0];
    py = meas_package.raw_measurements_[1];
  }
}


void UKF::output_write_header()
{
  // write the header
  ofs_<<setw(10)<<"count"<<" , "
      <<setw(10)<<"time(s)"<<" , "
      <<setw(10)<<"dt(s)"<<" , "
      <<setw(10)<<"type"<<" , "

      <<setw(10)<<"meas_px"<<" , "
      <<setw(10)<<"est_px"<<" , "
      <<setw(10)<<"gt_px"<<" , "
      <<setw(10)<<"RMSE_px"<<" , "

      <<setw(10)<<"meas_py"<<" , "
      <<setw(10)<<"est_py"<<" , "
      <<setw(10)<<"gt_py"<<" , "
      <<setw(10)<<"RMSE_py"<<" , "

      <<setw(10)<<"est_vx"<<" , "
      <<setw(10)<<"gt_vx"<<" , "
      <<setw(10)<<"RMSE_vx"<<" , "

      <<setw(10)<<"est_vy"<<" , "
      <<setw(10)<<"gt_vy"<<" , "
      <<setw(10)<<"RMSE_vy"<<" , "

      <<setw(10)<<"NIS"<<" , "
      <<setw(15)<<'\n';
  ofs_.flush();
}

void UKF::output_write_result(const VectorXd& RMSE,
                              const MeasurementPackage meas_package,
                              const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth)
{
  VectorXd estimation = estimations[estimations.size()-1]; // last value of ground_truth
  VectorXd gt_values = ground_truth[ground_truth.size()-1]; // last value of ground_truth

  double meas_px  = 0.0;
  double meas_py  = 0.0;

  extract_measured_positions(meas_package, meas_px,meas_py);

  string type;
  double NIS;

  if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    type = "Radar";
    NIS = NIS_radar_;
  }
  else{
    type = "Lidar";
    NIS = NIS_lidar_;
  }


  ofs_<<setw(10)<<count_<<" , "
      <<setw(10)<<time_<<" , "
      <<setw(10)<<dt_<<" , "
      <<setw(10)<<type<<" , "

      // px
      <<setw(10)<<meas_px<<" , "
      <<setw(10)<<estimation(0)<<" , "
      <<setw(10)<<gt_values(0)<<" , "
      <<setw(10)<<RMSE(0)<<" , "

      //py
      <<setw(10)<<meas_py<<" , "
      <<setw(10)<<estimation(1)<<" , "
      <<setw(10)<<gt_values(1)<<" , "
      <<setw(10)<<RMSE(1)<<" , "

      //vx
      <<setw(10)<<estimation(2)<<" , "
      <<setw(10)<<gt_values(2)<<" , "
      <<setw(10)<<RMSE(2)<<" , "

      //vy
      <<setw(10)<<estimation(3)<<" , "
      <<setw(10)<<gt_values(3)<<" , "
      <<setw(10)<<RMSE(3)<<" , "

      <<setw(10)<<NIS<<" , "

      <<setw(15)<<'\n';
  ofs_.flush();
}



