# [Object tracking with LIDAR, Radar, sensor fusion and Extended Kalman Filter](http://www.coldvision.io/2017/04/15/object-tracking-with-lidar-radar-sensor-fusion-and-extended-kalman-filter/)

> Self-Driving Car engineer program designed by Udacity

## 1. A minimal implementation of the Kalman Filter in python for the simplest 1D motion model

목적 : simplest Kalman filter for estimating the motion in 1D.

입력  The input are 
- the noisy 1D position measurements and 
- the sigmas for weighting during the update and predict steps.

출력 The output are 
- estimated position (mu) and 
- its trust (sigma) at a certain time.

```python 
# Write a program that will iteratively update and
# predict based on the location measurements 
# and inferred motions shown below. 
 
def update(mean1, var1, mean2, var2):
    new_mean = float(var2 * mean1 + var1 * mean2) / (var1 + var2)
    new_var = 1./(1./var1 + 1./var2)
    return [new_mean, new_var]
 
def predict(mean1, var1, mean2, var2):
    new_mean = mean1 + mean2
    new_var = var1 + var2
    return [new_mean, new_var]
 
measurements = [5., 6., 7., 9., 10.]
motion = [1., 1., 2., 1., 1.]
measurement_sig = 4.
motion_sig = 2.
mu = 0.
sig = 10000.
 
#Please print out ONLY the final values of the mean
#and the variance in a list [mu, sig]. 
 
# Insert code here
for n in range(len(measurements)):
    [mu,sig] = update(mu, sig, measurements[n], measurement_sig)
    print 'update: ', (mu,sig)
    [mu,sig] = predict(mu, sig, motion[n], motion_sig)
    print 'predict: ', (mu,sig)
 
print [mu, sig]  #estimation position(mu), Trust(sigma)

 """결과물 
 update: (4.998000799680128, 3.9984006397441023) 
 predict: (5.998000799680128, 5.998400639744102) 
 update: (5.999200191953932, 2.399744061425258) 
 predict: (6.999200191953932, 4.399744061425258) 
 update: (6.999619127420922, 2.0951800575117594) 
 predict: (8.999619127420921, 4.09518005751176) 
 update: (8.999811802788143, 2.0235152416216957) 
 predict: (9.999811802788143, 4.023515241621696) 
 update: (9.999906177177365, 2.0058615808441944) 
 predict: (10.999906177177365, 4.005861580844194) 
 [10.999906177177365, 4.005861580844194]
 """
```

The initial position is set to 0 and its sigma to 10000 (very high uncertainty), and after only a few steps the position gets close to the measurement and with a smaller sigma (smaller uncertainty).

## 2. A formal implementation of the Kalman Filter in Python using state and covariance matrices for the simplest 1D motion model

목적 : A multi-dimensional Kalman filter for estimating the motion in 1D, with the state defined by position and velocity.

입력 The input is defined by the initial state x (position and velocity) both set to 0. 

For estimating the state at a later time the `state transition matrix F(모션모델)` is used, matrix which embeds the motion model in 1D:

$$
x_{k+1} = x_{k} + velocity * dt
$$
