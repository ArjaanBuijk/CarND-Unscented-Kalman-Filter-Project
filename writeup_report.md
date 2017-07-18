# Unscented Kalman Filters

---

Unscented Kalman Filter Project.

---

The goals / steps of this project are the following:

* Implement Lidar and Radar sensor fusion in C++, using an Unscented Kalman filter.
* Test implementation by tracking a moving object, and optimize settings for process noise.
* Validate implementation by comparing predicted trajectory to provided ground truth trajectory.
* Compare accuracy to standard Kalman filter for Lidar and Extended Kalman filter for Radar.

---

# 1. Files

My project includes the following files:

- [<b>C++</b> - The source code](https://github.com/ArjaanBuijk/CarND-Unscented-Kalman-Filter-Project/tree/master/src)
- [<b>writeup_report.md</b> - A summary of the project](https://github.com/ArjaanBuijk/CarND-Unscented-Kalman-Filter-Project/blob/master/writeup_report.md)
- [<b>videoProject.mp4</b> - A video showing prediction (green) compared to sensor data (red=Lidar, blue=Radar)](https://github.com/ArjaanBuijk/CarND-Unscented-Kalman-Filter-Project/blob/master/videoProject.mp4)

    ![track1](https://github.com/ArjaanBuijk/CarND-Unscented-Kalman-Filter-Project/blob/master/videoProject.gif?raw=true)

---

# 2. Theory

The unscented kalman filter that is implemented uses a constant turn rate and velocity magnituge model, or CTRV:

<div style="text-align:center"><img src="https://github.com/ArjaanBuijk/CarND-Unscented-Kalman-Filter-Project/blob/master/Images/CTRV_diagram.jpg?raw=true" style="width: 400px;"/></div>

The CTRV model assumes a circular motion, described by 5 state parameters:

<div style="text-align:center"><img src="https://github.com/ArjaanBuijk/CarND-Unscented-Kalman-Filter-Project/blob/master/Images/CTRV_state_parameters.jpg?raw=true" style="width: 200px;"/></div>

The prediction of the state at time k+1 is given by non-linear equations that are evaluated at 11 so called Sigma Points. For these sigma points at time k+1 a closest approximation of a covariance matrix is calculated. Note that this is an approximation, because the equations for the prediction are non-linear, and the actual prediction will not provide a normal distribution.

The predicted state at time k+1 is then transformed from the CTRV model space into the appropriate measurement space of either Lidar (px, py) or Radar (r, psi, r_dot).

The final step is to update the state at k+1 using the provided measurement.

The CTRV model assumes a circular motion with constant velocity and radius. This is not an actual motion, which is taken into account through two process noise parameters:

| parameter | description|
|-----------|------------|
|std\_a_    | Process noise standard deviation longitudinal acceleration in m/s^2|
|std\_yawdd_| Process noise standard deviation yaw acceleration in rad/s^2|

The object being tracked is a bicycle, and good values for these process noise parameters are expected to be around 1.0, based on a maximum expected acceleration of ~2.0 m/s^2. Several tests were done to determine an optimal choice, and it was validated that the resulting Unscented Kalman Filter is behaving in a consistent manner by plotting the Normalized Innovation Squared (NIS).


---

# 3. Implementation

The implementation is done in these files:

| File | Description |
|-------|-------------|
|main.cpp| Implements a web-server that receives sensor measurments. When it receives a measurment from the simulator, the following is done in this function:<br>- extract the data<br>- create a 'measurement package'<br>- call ukf.ProcessMeasurement to process the measurement<br>- calculates RSME for px, py, vx, vy <br>- calls ukf.output_write_result, to log the measurements, ground truth, estimations and NIS values<br>- sends updated location and RSME back to simulator
|ukf.cpp|Processes the measurements, as follows:<br>- skip initial measurement. It is too inaccurate<br>- initialize location (px,py) from second measurement<br>- initialize velocity and yaw_angle from third measurement<br>- does predict and update step for next Lidar and Radar measurements<br>- stores estimate of new state at time k+1 in a public member variable <br><br>The initialization of the object's state is using the position of second & third measurements. The first measurement was skipped because it is too inaccurate. The velocity measurement of a Radar sensor is not very accurate and not suitable to initialize the velocity of the object.<br><br>The initial state is calculated as:<br> - px = dx/dt<br> - py = dy/dt<br> - vel = sqrt(vx\*vx+vy*vy)<br> - yaw_angle = atan2(vy,vx)<br> - yaw_rate = 0.0


The implementation was done in a straight-forward manner by using VectorXd and MatrixXd from the provided Eigen library. This keeps the implementation simple and easily readable since the code is nearly identical as the actual equations in Matrix notation.

---

# 3. Results

The choise of process noise parameters was optimized by running several studies, and looking at the RMSE. The target for this project was to achieve RMSEs for px, py, vx and vy below [.09, .10, .40, .30]

|std\_a_ | std\_yawdd_ |RMSE<br>px|RMSE<br>py|RMSE<br>vx|RMSE<br>vy|
|-------|------------|----------|----------|----------|----------|
|0.1|0.1|0.1305|0.1341|0.5454|0.3111|
|0.2|0.1|0.1162|0.1164|0.4541|0.3129|
|0.4|0.1|0.1131|0.1090|0.4548|0.3172|
|0.4|0.2|0.0763|0.0866|0.3954|0.2416|
|0.4|0.4|0.0654|0.0853|0.3771|0.2165|
|<b>0.8</b>|<b>0.4</b>|<b>0.0672</b>|<b>0.0805</b>|<b>0.3754</b>|<b>0.2166</b>|
|0.8|0.8|0.0648|0.0823|0.3743|0.2191|
|1.6|0.8|0.0686|0.0816|0.3769|0.2257|
|1.6|1.6|0.0683|0.0839|0.3850|0.2473|

To confirm that the Unscented Kalman Filter is consistent, the Normalized Innovation Squared (NIS) was plotted for both Lidar and Radar measurements against their expected 95% values of 5.991 (Lidar) and 7.815 (Radar):

<br> <div style="text-align:center"><img src="https://github.com/ArjaanBuijk/CarND-Unscented-Kalman-Filter-Project/blob/master/Images/NIS_Lidar.jpg?raw=true" style="width: 600px;"/></div>

<br> <div style="text-align:center"><img src="https://github.com/ArjaanBuijk/CarND-Unscented-Kalman-Filter-Project/blob/master/Images/NIS_Radar.jpg?raw=true" style="width: 600px;"/></div>


The trajectory predictions with optimized noise parameters were then compared with the EKF filter, for Lidar only, Radar only and with Sensor Fusion of Lidar & Radar:

|Method        |RMSE<br>px|RMSE<br>py|RMSE<br>vx|RMSE<br>vy|
|--------------|----------|----------|----------|----------|
|EKF Lidar only    |0.1839|0.1544|0.5931|0.5833|
|UKF Lidar only    |0.1019|0.0953|0.4250|0.2098|
|EKF Radar only    |0.2327|0.3348|0.5607|0.7178|
|UKF Radar only    |0.1396|0.1639|0.3960|0.2626|
|EKF Lidar & Radar |0.0988|0.0850|0.4328|0.4751|
|UKF Lidar & Radar |0.0672|0.0805|0.3754|0.2166|



# 5. Summary

The result can be summarized as follows:

- When using Lidar only, the result is more accurate than using Radar only.
- Even though Radar-only is giving a less accurate result than Lidar-only, when fusing the Radar sensor measurements with the Lidar sensor measurements, the prediction becomes more accurate.
- Only with sensor fusion is the RMSE [0.0672, 0.0805, 0.3754, 0.2166] below the required target [.09, .10, .40, .30].
