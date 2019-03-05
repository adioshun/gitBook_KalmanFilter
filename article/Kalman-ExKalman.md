# [Sensor Fusion Algorithms For Autonomous Driving: Part 1 — The Kalman filter and Extended Kalman Filter](https://medium.com/@wilburdes/sensor-fusion-algorithms-for-autonomous-driving-part-1-the-kalman-filter-and-extended-kalman-a4eab8a833dd)


## 1. The Basic Kalman Filter — using Lidar Data

The Kalman filter is over 50 years old, but is still one of the most powerful sensor fusion algorithms for smoothing noisy input data and estimating state. 

It assumes that location variables are gaussian i.e. can be completely parametrized by the mean and the covariance




## 2. The Extended Kalman filter — using Radar Data


### 2.1 State Prediction

linear state space model 채택 [[다른 모델들 참고]](https://uk.mathworks.com/help/ident/ug/what-are-state-space-models.html)
- at a time t can be estimated from state at time t-1 according to follow equation

![](https://cdn-images-1.medium.com/max/1200/1*rv35k4mQgQr2UVrVRH9uMg.png)


### 2.2 Measurement Update

 algorithm that use real measurements z to update the predicted state x′ by a scaling factor (called the Kalman Gain) proportional to the error between the measurment and the the predicted state.
 
 
![](https://cdn-images-1.medium.com/max/800/1*mOphU54_P-DgzPeeRLQP8g.png)


 