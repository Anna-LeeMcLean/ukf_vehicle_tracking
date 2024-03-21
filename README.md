# Tracking Vehicle Traffic with an Unscented Kalman Filter
Estimate and track the states of three (3) vehicles on a highway using an Unscented Kalman Filter (UKF) which fuses LiDAR and Radar data collected from an ego car.

![UKF Traffic State Estimation and Tracking](https://github.com/Anna-LeeMcLean/ukf_vehicle_tracking/assets/60242931/a37532e0-a301-46fd-9920-1922827bfecf)

[UKF Traffic Tracking Full Video](https://drive.google.com/file/d/1waTpi-bz6Hv2raLWvd0hSLEpJ72nJB9q/view?usp=sharing)

The green car is the ego car while the blue cars are the traffic cars being tracked. The pink arrows show the estimated position of the traffic cars at every timestep while the green arrows indicate the estimated direction of velocity.

## Motion and Measurement Model Characterization
The UKF estimates five (5) states for each vehicle - posixition in x and y, radial velocity, turning angle and angular velocity. The motion/process update step uses a Constant Turn Rate and Velocity (CTRV) model where the radial and angular velocities are assumed to remain constant throughout time. It also includes an estimated process noise which accounts for any changes in linear and angular acceleration that might occur which obviously are not considered in the motion model due to the constant velocity assumptions. The measurement update step alternates between using lidar and radar data at each time step to create a posterior estimate for each of the 5 states. The final estimate incorporates the kalman gain to bias either the estimate from the motion model or the measurement model as with all kalman filter implementations. The Unscented version of the kalman filter is used to handle the non-linear motion model and process noise functions shown in Figure 1. 

![image](https://github.com/Anna-LeeMcLean/ukf_vehicle_tracking/assets/60242931/f802124c-a6d7-4214-9e6a-1570a3dd34fb)

*Figure 1 showing state vector (x) and non-linear motion models when angualar velocity (psi dot) is zero and non-zero*

The lidar measurments provide position in x and y, implying that the mapping from the lidar measurement space to the state space is linear since we can directly map these measurements to their corresponding states in the state vector. On the other hand, the radar measurements provide euclidean distance to the traffic vehicle, angle between direction of motion and traffic vehicle and finally radial velocity of the traffic vehicle. The mapping between this measurement space and the state space is non-linear so the UKF also provides a means to handle the radar's measurement model.

![image](https://github.com/Anna-LeeMcLean/ukf_vehicle_tracking/assets/60242931/dde5719f-a7eb-4052-81c4-9aff13d09f9f)

*Figure 2 showing **radar** measurement model and its non-linear functions*

## UKF Parameters and Implementation
The UKF approximates the non-linear distribution of every state by creating sigma points which spread evenly across the distribution while considering process noise and uncertainity. The sigma points are then ran through the motion model so that a mean state estimate and updated process covariance matrix can be extracted from the processed sigma points. The number of sigma points used was determined by the following equation which is standard in UKF implementation: *ns = 2nx + 1*; where *ns* is the number of sigma points and *nx* is the number of states being estimated. The UKF design parameter *lambda*, which works to approximate the magnitude of each state's variance, was calculated as: *lambda = 3 - nx*. This equation is also standard in UKF implementation. The equation used to generate all sigma points for each state is given below where *Xk* is the sigma point matrix and *Pk* is the initial process covariance matrix.

![image](https://github.com/Anna-LeeMcLean/ukf_vehicle_tracking/assets/60242931/a82b2283-e047-46ca-864a-bfbc2184ec1e)

*Figure 3 showing generation of sigma point matrix of size (nx x ns)*

In this implementation, the initial state vector and process covariance matrix were augmented to consider the process noise. The values for the noise variance in acceleration were estimated from assumptions about the maximum possible linear and angular accelerations a car could achieve; where standard deviation was estimated as half of the maximum linear/angular acceleration. The Normalized Innovation Squared (NIS) threshold was used to tweak these values. More on this later.

### Estimating State using Motion Model
The following equations were used to extract the predicted mean and updated process covariance from the processed sigma points:

![image](https://github.com/Anna-LeeMcLean/ukf_vehicle_tracking/assets/60242931/4c4f1bf7-0e29-49fb-acf8-4ceb4f9681a4)

*Figure 4 showing equations for calculation of prior state mean and covariance beliefs*

The predicted mean is simply a weighted sum of all the processed sigma points. The predicted covariance is the sum of weighted, squared differences between every processed sigma point and the inital state belief to produce a square matrix with size equal to the number of states. The weights are required in these equations to undo the scaling of covariance done by the lambda term in Figure 3.

### Updating State Estimate using Radar Measurement Model
The posterior state belief is determined in a similar fashion when incorporating **radar** measurements. The processed sigma points from the motion update step are used once again and ran through the measurement model which produces sigma points that represent the non-linear radar measurement distribution. The mean measurement vector and measurement covariance matrix are determined to be used in the final update step. The equations used are shown below:

![image](https://github.com/Anna-LeeMcLean/ukf_vehicle_tracking/assets/60242931/6dfb0595-ef45-40c5-bf6e-77b1d79abf67)

*Figure 5 showing equations used to calculate predicted measurement mean and covariance*

### Updating State Estimate using LiDAR Measurement Model
For LiDAR measurements, the original kalman filter measurement update step is used since its measurement model is linear.

### Final UKF State Estimate
We've talked about predicting a prior belief and its covariance using a non-linear motion model then predicting a measurement mean using the prior belief. The error between the measurement mean and the actual measurement is used to calculate the final state belief. This error is scaled by the Kalman gain as shown in the update state equation in Figure 6.

![image](https://github.com/Anna-LeeMcLean/ukf_vehicle_tracking/assets/60242931/34c5f20e-10c2-4a5a-b3de-fccc6131caed)

*Figure 6 showing equations used to update the predicted state using the measurement mean*

### NIS Thresholding

The chi-squared distribution is used to validate the estimates used for the variance of linear and angular acceleration noise. Earlier, we estimated the standard deviations as half the maximum possible acceleration. These values were tweaked using the chi-squared distribution which provides a guide to determining if the standard deviations chosen over or under estimate the real-world process noise. The table below shows the [Chi-squared distribution](https://en.wikipedia.org/wiki/Chi-squared_distribution) table used in this UKF implementation:

![chi-squared distribution](https://github.com/Anna-LeeMcLean/ukf_vehicle_tracking/assets/60242931/2092b9b3-50f8-41c8-82a6-3e568280a8ea)

*Figure 7 shwoing chi-squared distribution*

The radar measurement vector has dimension 3, therefore we should expect that not more than 5% (p-value) of NIS values for a radar measurement update exceed 7.81 as shown in Figure 7. In the same way, the lidar measurement vector has dimension 2, therefore we expect that not not than 5% of NIS values for a lidar measurement update exceed 5.99. If more than 5% of NIS values exceed these thresolds, then we know we have underestimated the standard deviations. Figures 8 and 9 show the radar and LiDAR NIS plots for every timestep for one of the cars being tracked on the highway. The black horizontal line represents the NIS threshold while the red plot shows the NIS values for every timestep.

![image](https://github.com/Anna-LeeMcLean/ukf_vehicle_tracking/assets/60242931/e00014b1-1114-4638-911e-8086846e26b5)

*Figure 8 showing example of LiDAR NIS plot with its corresponding threshold*

![image](https://github.com/Anna-LeeMcLean/ukf_vehicle_tracking/assets/60242931/e52c150d-f52b-499b-86fe-d11e38c930e2)

*Figure 9 showing example of RADAR NIS plot with its corresponding threshold*

## Local Install Instructions

`git clone https://github.com/Anna-LeeMcLean/ukf_vehicle_tracking.git`

`cd ukf_vehicle_tracking`

`mkdir build && cd build`

`cmake ..`

`make`

`./ukf_highway`


## Dependencies
* cmake >= 3.5
* make >= 4.2.1 (Linux, Mac), 3.81 (Windows)
    Linux: make is installed by default on most Linux distros
    Mac: install Xcode command line tools to get make
    Windows: Click here for installation instructions
* gcc/g++ >= 9.4.0
    Linux: gcc / g++ is installed by default on most Linux distros
    Mac: same deal as make - install Xcode command line tools
    Windows: recommend using MinGW
* PCL 1.2
* C++ 14
