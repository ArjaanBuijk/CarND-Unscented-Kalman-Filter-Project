#ifndef UKF_H
#define UKF_H

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

class UKF {
public:

  ///* initially set to 0,
  int is_initialized_;

  ///* if this is false, laser measurements will be ignored (except for init)
  bool use_laser_;

  ///* if this is false, radar measurements will be ignored (except for init)
  bool use_radar_;

  ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  VectorXd x_;

  ///* save first Lidar measurements, for initialization
  double px1_, py1_, dt2_;

  ///* timestamp of previous measurement
  long long previous_timestamp_;

  ///* state covariance matrix
  MatrixXd P_;

  ///* predicted sigma points matrix
  MatrixXd Xsig_pred_;

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

  ///* measurement covariance matrix - radar */
  MatrixXd R_radar_;

  ///* State dimension
  int n_x_;

  ///* Augmented state dimension
  int n_aug_;

  ///* Sigma point spreading parameter
  double lambda_;

  ///* Radar measurement dimension [r, phi, and r_dot]
  int n_z_radar_;

  ///* Lidar measurement dimension [px, py]
  int n_z_lidar_;

  ///* Normalized Innovation Squared (NIS) for radar
  double NIS_radar_;

  ///* Normalized Innovation Squared (NIS) for lidar
  double NIS_lidar_;


  /**
   * Constructor
   */
  UKF(std::ofstream & ofs);

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
   */
  void Prediction();

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

  /** Lesson 7.17: Create augmented sigma points */
  MatrixXd create_augmented_sigma_points();

  /** Lesson 7.20: Calculate sigma points at time k+1*/
  void predict_sigma_points(const MatrixXd& Xsig_aug);

  /** Lesson 7.23: Calculate mean and covariance of the predicted sigma points at time k+1*/
  void predict_mean_and_covariance();

  /** Lesson 7.26: Predict radar measurement at time k+1*/
  void predict_radar_measurement(MatrixXd& Zsig, VectorXd& z_pred, MatrixXd& S);

  /** Lesson 7.29: Update state vector & covariance matrix */
  void update_radar_measurement(const MatrixXd& Zsig,
                                     const VectorXd& z_pred,
                                     const MatrixXd& S,
                                     const VectorXd& z);

  /** Lesson 7.31: Calculating NIS for radar */
  double calculate_NIS_radar(const VectorXd& z_pred,
                                  const MatrixXd& S,
                                  const VectorXd& z);

  /** Lesson 7.26: Predict lidar measurement at time k+1*/
  void predict_lidar_measurement(MatrixXd& Zsig, VectorXd& z_pred, MatrixXd& S);

  /** Lesson 7.29: Update state vector & covariance matrix */
  void update_lidar_measurement(const MatrixXd& Zsig,
                                     const VectorXd& z_pred,
                                     const MatrixXd& S,
                                     const VectorXd& z);

  /** Lesson 7.31: Calculating NIS for lidar */
  double calculate_NIS_lidar(const VectorXd& z_pred,
                                  const MatrixXd& S,
                                  const VectorXd& z);

  /** Helper function to normalize the angle */
  void normalize_angle( double& angle );

  /**
  * test methods - REMOVE ONCE DONE
  */
  /** A debug method to test weight initialization. */
  bool test_weight_initialization(const VectorXd& weights);

  /** A debug method to test creation of augmented sigma points. */
  bool test_create_augmented_sigma_points();

  /** A debug method to test calculation of sigma points at time k+1. */
  bool test_predict_sigma_points();

  /** A debug method to test calculation of mean and covariance of predicted sigma points at time k+1. */
  bool test_predict_mean_and_covariance();

  /** A debug method to test calculation of predicted radar measurement at time k+1. */
  bool test_predict_radar_measurement();

  /** A debug method to test update for radar measurement at time k+1. */
  bool test_update_radar_measurement();

  ///* for writing result to output file
  void output_write_result(const VectorXd& RMSE,
                           const MeasurementPackage meas_package,
                           const vector<VectorXd> &estimations,
                           const vector<VectorXd> &ground_truth);

private:

  ///* get px & py from the measurement
  void extract_measured_positions(const MeasurementPackage meas_package,
                                  double& px,double& py);

  ///* ukf output file
  std::ofstream& ofs_;

  ///* counter of measurements
  int count_;

  ///* timestamp of first measurement
  double timestamp0_;

  ///* time in seconds of current measurement, counted from first measurement
  double time_;

  ///* dt in seconds of current measurement, counted from previous measurement
  double dt_;

  void output_write_header();                                       // write header

};

#endif /* UKF_H */
