/***************************************************************************
* TEST FUNCTIONS -- USED DURING DEVELOPMENT TO MAKE SURE ALL IS OK
*/

bool UKF::test_weight_initialization(const VectorXd& weights)
{

  std::cout<<"-----------------------------------------------------------------\n";
  std::cout<<"Function test_weight_initialization";

  VectorXd weights_correct(15);
  weights_correct << -1.3333333,
                    0.16666667,
                    0.16666667,
                    0.16666667,
                    0.16666667,
                    0.16666667,
                    0.16666667,
                    0.16666667,
                    0.16666667,
                    0.16666667,
                    0.16666667,
                    0.16666667,
                    0.16666667,
                    0.16666667,
                    0.16666667;

  for (int i=0; i<weights.rows(); ++i){
    double diff = weights(i)-weights_correct(i);
    if (fabs(diff)>1.E-6){
        cout<<"ERROR: Initial Weight incorrect at index "<<i<<'\n';
        cout<<weights(i)<<" vs. "<<weights_correct(i)<<'\n';
        cout<<"diff = "<<diff<<'\n';
        return false;
    }
  }
  cout<<"OK!"<<'\n';
  return true;
}

bool UKF::test_create_augmented_sigma_points()
{

  std::cout<<"-----------------------------------------------------------------\n";
  std::cout<<"Function test_create_augmented_sigma_points\n";

  /** see test of Lesson 7.17 */
  std_a_ = 0.2;
  std_yawdd_ = 0.2;

  //set example state
  x_<<   5.7441,
         1.3800,
         2.2049,
         0.5015,
         0.3528;

  //create example covariance matrix
  P_<<     0.0043,   -0.0013,    0.0030,   -0.0022,   -0.0020,
          -0.0013,    0.0077,    0.0011,    0.0071,    0.0060,
           0.0030,    0.0011,    0.0054,    0.0007,    0.0008,
          -0.0022,    0.0071,    0.0007,    0.0098,    0.0100,
          -0.0020,    0.0060,    0.0008,    0.0100,    0.0123;

  // expected result:
  MatrixXd Xsig_aug_correct = MatrixXd(n_aug_, 2 * n_aug_ + 1);
  Xsig_aug_correct<<
  5.7441,  5.85768,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,  5.63052,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,
    1.38,  1.34566,  1.52806,     1.38,     1.38,     1.38,     1.38,     1.38,  1.41434,  1.23194,     1.38,     1.38,     1.38,     1.38,     1.38,
  2.2049,  2.28414,  2.24557,  2.29582,   2.2049,   2.2049,   2.2049,   2.2049,  2.12566,  2.16423,  2.11398,   2.2049,   2.2049,   2.2049,   2.2049,
  0.5015,  0.44339, 0.631886, 0.516923, 0.595227,   0.5015,   0.5015,   0.5015,  0.55961, 0.371114, 0.486077, 0.407773,   0.5015,   0.5015,   0.5015,
  0.3528, 0.299973, 0.462123, 0.376339,  0.48417, 0.418721,   0.3528,   0.3528, 0.405627, 0.243477, 0.329261,  0.22143, 0.286879,   0.3528,   0.3528,
       0,        0,        0,        0,        0,        0,  0.34641,        0,        0,        0,        0,        0,        0, -0.34641,        0,
       0,        0,        0,        0,        0,        0,        0,  0.34641,        0,        0,        0,        0,        0,        0, -0.34641;


  // calculate it
  MatrixXd Xsig_aug = create_augmented_sigma_points();

  // check it
  for (int i=0; i<Xsig_aug.rows(); ++i){
    for (int j=0; j<Xsig_aug.cols(); ++j){
      double diff = Xsig_aug(i,j)-Xsig_aug_correct(i,j);
      if (fabs(diff)>1.E-5){
        cout<<"ERROR: Xsig_aug incorrect at index (i,j) = ("<<i<<','<<j<<')'<<'\n';
        cout<<Xsig_aug(i,j)<<" vs. "<<Xsig_aug_correct(i,j)<<'\n';
        cout<<"diff = "<<diff<<'\n';
        return false;
      }
    }
  }

  cout<<"OK!"<<'\n';
  return true;
}

bool UKF::test_predict_sigma_points()
{

  std::cout<<"-----------------------------------------------------------------\n";
  std::cout<<"Function test_predict_sigma_points\n";

  /** see test of Lesson 7.20 */

  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
  Xsig_aug <<
    5.7441,  5.85768,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,   5.63052,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,
      1.38,  1.34566,  1.52806,     1.38,     1.38,     1.38,     1.38,     1.38,   1.41434,  1.23194,     1.38,     1.38,     1.38,     1.38,     1.38,
    2.2049,  2.28414,  2.24557,  2.29582,   2.2049,   2.2049,   2.2049,   2.2049,   2.12566,  2.16423,  2.11398,   2.2049,   2.2049,   2.2049,   2.2049,
    0.5015,  0.44339, 0.631886, 0.516923, 0.595227,   0.5015,   0.5015,   0.5015,   0.55961, 0.371114, 0.486077, 0.407773,   0.5015,   0.5015,   0.5015,
    0.3528, 0.299973, 0.462123, 0.376339,  0.48417, 0.418721,   0.3528,   0.3528,  0.405627, 0.243477, 0.329261,  0.22143, 0.286879,   0.3528,   0.3528,
         0,        0,        0,        0,        0,        0,  0.34641,        0,         0,        0,        0,        0,        0, -0.34641,        0,
         0,        0,        0,        0,        0,        0,        0,  0.34641,         0,        0,        0,        0,        0,        0, -0.34641;

  dt_ = 0.1; //time diff in sec

  // expected result:
  MatrixXd Xsig_pred_correct = MatrixXd(n_x_, 2 * n_aug_ + 1);
  Xsig_pred_correct <<
  5.93553, 6.062510, 5.922170, 5.941500, 5.923610, 5.935160, 5.93705, 5.935530, 5.808320, 5.944810, 5.929350, 5.945530, 5.935890, 5.93401, 5.935530,
  1.48939, 1.446730, 1.664840, 1.497190, 1.508000, 1.490010, 1.49022, 1.489390, 1.530800, 1.312870, 1.481820, 1.469670, 1.488760, 1.48855, 1.489390,
  2.20490, 2.284140, 2.245570, 2.295820, 2.204900, 2.204900, 2.23954, 2.204900, 2.125660, 2.164230, 2.113980, 2.204900, 2.204900, 2.17026, 2.204900,
  0.53678, 0.473387, 0.678098, 0.554557, 0.643644, 0.543372, 0.53678, 0.538512, 0.600173, 0.395462, 0.519003, 0.429916, 0.530188, 0.53678, 0.535048,
  0.35280, 0.299973, 0.462123, 0.376339, 0.484170, 0.418721, 0.35280, 0.387441, 0.405627, 0.243477, 0.329261, 0.221430, 0.286879, 0.35280, 0.318159;

  // calculate it
  predict_sigma_points(Xsig_aug);

  // check it
  for (int i=0; i<Xsig_pred_.rows(); ++i){
    for (int j=0; j<Xsig_pred_.cols(); ++j){
      double diff = Xsig_pred_(i,j)-Xsig_pred_correct(i,j);
      if (fabs(diff)>1.E-5){
        cout<<"ERROR: Xsig_pred_ incorrect at index (i,j) = ("<<i<<','<<j<<')'<<'\n';
        cout<<Xsig_pred_(i,j)<<" vs. "<<Xsig_pred_correct(i,j)<<'\n';
        cout<<"diff = "<<diff<<'\n';
        return false;
      }
    }
  }

  cout<<"OK!"<<'\n';
  return true;
}

bool UKF::test_predict_mean_and_covariance()
{

  std::cout<<"-----------------------------------------------------------------\n";
  std::cout<<"Function test_predict_mean_and_covariance\n";

  /** see test of Lesson 7.23 */

  //create example matrix with predicted sigma points
  Xsig_pred_ <<
         5.9374,  6.0640,   5.925,  5.9436,  5.9266,  5.9374,  5.9389,  5.9374,  5.8106,  5.9457,  5.9310,  5.9465,  5.9374,  5.9359,  5.93744,
           1.48,  1.4436,   1.660,  1.4934,  1.5036,    1.48,  1.4868,    1.48,  1.5271,  1.3104,  1.4787,  1.4674,    1.48,  1.4851,    1.486,
          2.204,  2.2841,  2.2455,  2.2958,   2.204,   2.204,  2.2395,   2.204,  2.1256,  2.1642,  2.1139,   2.204,   2.204,  2.1702,   2.2049,
         0.5367, 0.47338, 0.67809, 0.55455, 0.64364, 0.54337,  0.5367, 0.53851, 0.60017, 0.39546, 0.51900, 0.42991, 0.530188,  0.5367, 0.535048,
          0.352, 0.29997, 0.46212, 0.37633,  0.4841, 0.41872,   0.352, 0.38744, 0.40562, 0.24347, 0.32926,  0.2214, 0.28687,   0.352, 0.318159;

  // expected result for predicted state mean, x:
  VectorXd x_correct = VectorXd(n_x_);
  x_correct <<
  5.936370,
  1.490350,
  2.205280,
  0.536853,
  0.353577;

  // expected result for predicted covariance matrix, P:
  MatrixXd P_correct = MatrixXd(n_x_, n_x_);
  P_correct <<
   0.00543425, -0.00240530, 0.003415760, -0.003481960, -0.002993780,
  -0.00240530,  0.01084500, 0.001492300,  0.009801820,  0.007910910,
   0.00341576,  0.00149230, 0.005801290,  0.000778632,  0.000792973,
  -0.00348196,  0.00980182, 0.000778632,  0.011923800,  0.011249100,
  -0.00299378,  0.00791091, 0.000792973,  0.011249100,  0.012697200;

  // calculate it
  predict_mean_and_covariance();

  // check it: x
  for (int i=0; i<x_.rows(); ++i){
    double diff = x_(i)-x_correct(i);
    if (fabs(diff)>1.E-5){
      cout<<"ERROR: x_ incorrect at index (i) = ("<<i<<')'<<'\n';
      cout<<x_(i)<<" vs. "<<x_correct(i)<<'\n';
      cout<<"diff = "<<diff<<'\n';
      return false;
    }
  }

  // check it: P
  for (int i=0; i<P_.rows(); ++i){
    for (int j=0; j<P_.cols(); ++j){
      double diff = P_(i,j)-P_correct(i,j);
      if (fabs(diff)>1.E-5){
        cout<<"ERROR: P_ incorrect at index (i,j) = ("<<i<<','<<j<<')'<<'\n';
        cout<<P_(i,j)<<" vs. "<<P_correct(i,j)<<'\n';
        cout<<"diff = "<<diff<<'\n';
        return false;
      }
    }
  }

  cout<<"OK!"<<'\n';
  return true;
}

bool UKF::test_predict_radar_measurement()
{

  std::cout<<"-----------------------------------------------------------------\n";
  std::cout<<"Function test_predict_radar_measurement\n";

  /** see test of Lesson 7.26 */

  //radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  //radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.0175;

  //radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.1;

  // measurement covariance matrix - rada
  R_radar_ <<  std_radr_*std_radr_, 0                      , 0,
               0                  , std_radphi_*std_radphi_, 0,
               0                  , 0                      ,std_radrd_*std_radrd_;


  //create example matrix with predicted sigma points
  MatrixXd Xsig_pred = MatrixXd(n_x_, 2 * n_aug_ + 1);
  Xsig_pred <<
         5.9374,  6.0640,   5.925,  5.9436,  5.9266,  5.9374,  5.9389,  5.9374,  5.8106,  5.9457,  5.9310,  5.9465,  5.9374,  5.9359,  5.93744,
           1.48,  1.4436,   1.660,  1.4934,  1.5036,    1.48,  1.4868,    1.48,  1.5271,  1.3104,  1.4787,  1.4674,    1.48,  1.4851,    1.486,
          2.204,  2.2841,  2.2455,  2.2958,   2.204,   2.204,  2.2395,   2.204,  2.1256,  2.1642,  2.1139,   2.204,   2.204,  2.1702,   2.2049,
         0.5367, 0.47338, 0.67809, 0.55455, 0.64364, 0.54337,  0.5367, 0.53851, 0.60017, 0.39546, 0.51900, 0.42991, 0.530188,  0.5367, 0.535048,
          0.352, 0.29997, 0.46212, 0.37633,  0.4841, 0.41872,   0.352, 0.38744, 0.40562, 0.24347, 0.32926,  0.2214, 0.28687,   0.352, 0.318159;

  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z_radar_, 2 * n_aug_ + 1);

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z_radar_);

  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z_radar_,n_z_radar_);

  // expected result for z_pred:
  VectorXd z_pred_correct = VectorXd(n_z_radar_);
  z_pred_correct <<
  6.121550,
  0.245993,
  2.103130;

  // expected result for S:
  MatrixXd S_correct = MatrixXd(n_z_radar_,n_z_radar_);
  S_correct <<
   0.094617100, -0.000139448,  0.004070160,
  -0.000139448,  0.000617548, -0.000770652,
   0.004070160, -0.000770652,  0.018091700;

  // calculate it
  predict_radar_measurement(Zsig, z_pred, S);

  // check it: z_pred
  for (int i=0; i<z_pred.rows(); ++i){
    double diff = z_pred(i)-z_pred_correct(i);
    if (fabs(diff)>1.E-5){
      cout<<"ERROR: z_pred incorrect at index (i) = ("<<i<<')'<<'\n';
      cout<<z_pred(i)<<" vs. "<<z_pred_correct(i)<<'\n';
      cout<<"diff = "<<diff<<'\n';
      return false;
    }
  }

  // check it: S
  for (int i=0; i<S.rows(); ++i){
    for (int j=0; j<S.cols(); ++j){
      double diff = S(i,j)-S_correct(i,j);
      if (fabs(diff)>1.E-5){
        cout<<"ERROR: S incorrect at index (i,j) = ("<<i<<','<<j<<')'<<'\n';
        cout<<S(i,j)<<" vs. "<<S_correct(i,j)<<'\n';
        cout<<"diff = "<<diff<<'\n';
        return false;
      }
    }
  }

  cout<<"OK!"<<'\n';
  return true;
}

bool UKF::test_update_radar_measurement()
{

  std::cout<<"-----------------------------------------------------------------\n";
  std::cout<<"Function test_update_radar_measurement\n";

  /** see test of Lesson 7.29 */

  //create example matrix with predicted sigma points in state space
  Xsig_pred_ <<
         5.9374,  6.0640,   5.925,  5.9436,  5.9266,  5.9374,  5.9389,  5.9374,  5.8106,  5.9457,  5.9310,  5.9465,  5.9374,  5.9359,  5.93744,
           1.48,  1.4436,   1.660,  1.4934,  1.5036,    1.48,  1.4868,    1.48,  1.5271,  1.3104,  1.4787,  1.4674,    1.48,  1.4851,    1.486,
          2.204,  2.2841,  2.2455,  2.2958,   2.204,   2.204,  2.2395,   2.204,  2.1256,  2.1642,  2.1139,   2.204,   2.204,  2.1702,   2.2049,
         0.5367, 0.47338, 0.67809, 0.55455, 0.64364, 0.54337,  0.5367, 0.53851, 0.60017, 0.39546, 0.51900, 0.42991, 0.530188,  0.5367, 0.535048,
          0.352, 0.29997, 0.46212, 0.37633,  0.4841, 0.41872,   0.352, 0.38744, 0.40562, 0.24347, 0.32926,  0.2214, 0.28687,   0.352, 0.318159;

  //create example vector for predicted state mean
  x_ <<
     5.93637,
     1.49035,
     2.20528,
    0.536853,
    0.353577;

  //create example matrix for predicted state covariance
  P_ <<
  0.0054342,  -0.002405,  0.0034157, -0.0034819, -0.00299378,
  -0.002405,    0.01084,   0.001492,  0.0098018,  0.00791091,
  0.0034157,   0.001492,  0.0058012, 0.00077863, 0.000792973,
 -0.0034819,  0.0098018, 0.00077863,   0.011923,   0.0112491,
 -0.0029937,  0.0079109, 0.00079297,   0.011249,   0.0126972;

  //create example matrix with sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z_radar_, 2 * n_aug_ + 1);
  Zsig <<
      6.1190,  6.2334,  6.1531,  6.1283,  6.1143,  6.1190,  6.1221,  6.1190,  6.0079,  6.0883,  6.1125,  6.1248,  6.1190,  6.1188,  6.12057,
     0.24428,  0.2337, 0.27316, 0.24616, 0.24846, 0.24428, 0.24530, 0.24428, 0.25700, 0.21692, 0.24433, 0.24193, 0.24428, 0.24515, 0.245239,
      2.1104,  2.2188,  2.0639,   2.187,  2.0341,  2.1061,  2.1450,  2.1092,  2.0016,   2.129,  2.0346,  2.1651,  2.1145,  2.0786,  2.11295;

  //create example vector for mean predicted measurement
  VectorXd z_pred = VectorXd(n_z_radar_);
  z_pred <<
      6.12155,
     0.245993,
      2.10313;

  //create example matrix for predicted measurement covariance
  MatrixXd S = MatrixXd(n_z_radar_,n_z_radar_);
  S <<
      0.0946171, -0.000139448,   0.00407016,
   -0.000139448,  0.000617548, -0.000770652,
     0.00407016, -0.000770652,    0.0180917;

  //create example vector for incoming radar measurement
  VectorXd z = VectorXd(n_z_radar_);
  z <<
      5.9214,   //rho in m
      0.2187,   //phi in rad
      2.0062;   //rho_dot in m/s


  // expected result for x:
  VectorXd x_correct = VectorXd(n_x_);
  x_correct <<
  5.922760,
  1.418230,
  2.155930,
  0.489274,
  0.321338;

  // expected result for S:
  MatrixXd P_correct = MatrixXd(n_x_, n_x_);
  P_correct <<
   0.003615790, -0.000357881,  0.00208316, -0.000937196, -0.00071727,
  -0.000357881,  0.005398670,  0.00156846,  0.004553420,  0.00358885,
   0.002083160,  0.001568460,  0.00410651,  0.001603330,  0.00171811,
  -0.000937196,  0.004553420,  0.00160333,  0.006526340,  0.00669436,
  -0.000717190,  0.003588840,  0.00171811,  0.006694260,  0.00881797;

  // calculate it
  update_radar_measurement(Zsig, z_pred, S, z);

  // check it: x
  for (int i=0; i<x_.rows(); ++i){
    double diff = x_(i)-x_correct(i);
    if (fabs(diff)>1.E-5){
      cout<<"ERROR: x_ incorrect at index (i) = ("<<i<<')'<<'\n';
      cout<<x_(i)<<" vs. "<<x_correct(i)<<'\n';
      cout<<"diff = "<<diff<<'\n';
      return false;
    }
  }

  // check it: P
  for (int i=0; i<P_.rows(); ++i){
    for (int j=0; j<P_.cols(); ++j){
      double diff = P_(i,j)-P_correct(i,j);
      if (fabs(diff)>1.E-5){
        cout<<"ERROR: P_ incorrect at index (i,j) = ("<<i<<','<<j<<')'<<'\n';
        cout<<P_(i,j)<<" vs. "<<P_correct(i,j)<<'\n';
        cout<<"diff = "<<diff<<'\n';
        return false;
      }
    }
  }

  cout<<"OK!"<<'\n';
  return true;
}

