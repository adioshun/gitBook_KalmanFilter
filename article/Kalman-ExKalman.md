# [Sensor Fusion Algorithms For Autonomous Driving: Part 1 — The Kalman filter and Extended Kalman Filter](https://medium.com/@wilburdes/sensor-fusion-algorithms-for-autonomous-driving-part-1-the-kalman-filter-and-extended-kalman-a4eab8a833dd)

> [깃허브](https://github.com/asterixds/ExtendedKalmanFilter)

## 1. The Basic Kalman Filter — using Lidar Data

The Kalman filter is over 50 years old, but is still one of the most powerful sensor fusion algorithms for smoothing noisy input data and estimating state. 

It assumes that location variables are gaussian i.e. can be completely parametrized by the mean and the covariance


### 1.1 State Prediction

linear state space model 채택 [[다른 모델들 참고]](https://uk.mathworks.com/help/ident/ug/what-are-state-space-models.html)
- at a time t can be estimated from state at time t-1 according to follow equation

![](https://cdn-images-1.medium.com/max/1200/1*rv35k4mQgQr2UVrVRH9uMg.png)


### 1.2 Measurement Update

 algorithm that use real measurements z to update the predicted state x′ by a scaling factor (called the Kalman Gain) proportional to the error between the measurment and the the predicted state.
 
 
![](https://cdn-images-1.medium.com/max/800/1*mOphU54_P-DgzPeeRLQP8g.png)


### 1.3 example

Model Assumptions
![](https://cdn-images-1.medium.com/max/800/1*K2Jzlu-aFUjb5bkenhwkIQ.png)

```python 
"""prediction step"""
def predict(x, P):
    x = (F * x) + u 
    P = F * P * F.transpose() #Acceleration noise Q is assumed to be zero
    return x, P

"""measurement update step"""
def update(x, P,z):
    # measurement update
    Z = matrix([z])
    y = Z.transpose() - (H * x)
    S = H * P * H.transpose() + R
    K = P * H.transpose() * S.inverse()
    x = x + (K * y)
    P = (I - (K * H)) * P
    return x, P
```

The final step iterates through the measurements and applies the prediction and update steps of the filter as listed above.

```python 
plot_position_variance(x,P,edgecolor='r')  #plot initial position and covariance in red   
for z in measurements:
    x,P = predict(x, P)
    x,P = update(x, P,z)
    plot_position_variance(x,P,edgecolor='b') #plot updates in blue
    print(x)
    print(P)
```


![](https://cdn-images-1.medium.com/max/800/1*SNMQY6Lduj0otwqC8CCn5A.png)

The above figure illustrates each iteration of the kalman filter for the px and py dimensions of the state vector along with the positional covariance. 

The red circle is a visualisation of our initial process uncertainty. 

As we go through the incremental predictions and measurement updates, we begin to develop a better estimate of state with less uncertainty (variance). 

As you can see, the final state vector x=[11.99, 2.05] is very close to the final measurement value and the positional state variance is also minimal at 0.05


## 2. The Extended Kalman filter — using Radar Data




