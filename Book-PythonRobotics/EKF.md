# [Extended Kalman Filter Localization](https://pythonrobotics.readthedocs.io/en/latest/modules/localization.html#extended-kalman-filter-localization)

![EKF](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/Localization/extended_kalman_filter/animation.gif)

This is a sensor fusion localization with Extended Kalman Filter(EKF).
- The blue line is true trajectory, the black line is dead reckoning trajectory,
- the green point is positioning observation (ex. GPS), 
- and the red line is estimated trajectory with EKF.

The red ellipse is estimated covariance ellipse with EKF.

> Code: [PythonRobotics/extended\_kalman\_filter\.py at master · AtsushiSakai/PythonRobotics](https://github.com/AtsushiSakai/PythonRobotics/blob/master/Localization/extended_kalman_filter/extended_kalman_filter.py)


### Filter design

In this simulation, the robot has a state vector includes 4 states at time $t$.

$$\textbf{x}_t=[x_t, y_t, \phi_t, v_t]$$

x, y are a 2D x-y position, $\phi$ is orientation, and v is velocity.

In the code, "xEst" means the state vector. [code](https://github.com/AtsushiSakai/PythonRobotics/blob/916b4382de090de29f54538b356cef1c811aacce/Localization/extended_kalman_filter/extended_kalman_filter.py#L168)

And, $P_t$ is covariace matrix of the state,

$Q$ is covariance matrix of process noise, 

$R$ is covariance matrix of observation noise at time $t$ 

　

The robot has a speed sensor and a gyro sensor.

So, the input vecor can be used as each time step

$$\textbf{u}_t=[v_t, \omega_t]$$

Also, the robot has a GNSS sensor, it means that the robot can observe x-y position at each time.

$$\textbf{z}_t=[x_t,y_t]$$

The input and observation vector includes sensor noise.

In the code, "observation" function generates the input and observation vector with noise [code](https://github.com/AtsushiSakai/PythonRobotics/blob/916b4382de090de29f54538b356cef1c811aacce/Localization/extended_kalman_filter/extended_kalman_filter.py#L34-L50)

