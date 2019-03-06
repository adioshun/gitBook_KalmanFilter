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


출력 : The output is defined by 
- the state x at a certain time, and 
- its uncertainty matrix P.


```python 
# Write a function 'kalman_filter' that implements a multi-
# dimensional Kalman Filter for the example given
 
from math import *
 
class matrix:
    
    # implements basic operations of a matrix class
    
    def __init__(self, value):
        self.value = value
        self.dimx = len(value)
        self.dimy = len(value[0])
        if value == [[]]:
            self.dimx = 0
    
    def zero(self, dimx, dimy):
        # check if valid dimensions
        if dimx < 1 or dimy < 1:
            raise ValueError, "Invalid size of matrix"
        else:
            self.dimx = dimx
            self.dimy = dimy
            self.value = [[0 for row in range(dimy)] for col in range(dimx)]
    
    def identity(self, dim):
        # check if valid dimension
        if dim < 1:
            raise ValueError, "Invalid size of matrix"
        else:
            self.dimx = dim
            self.dimy = dim
            self.value = [[0 for row in range(dim)] for col in range(dim)]
            for i in range(dim):
                self.value[i][i] = 1
    
    def show(self):
        for i in range(self.dimx):
            print self.value[i]
        print ' '
    
    def __add__(self, other):
        # check if correct dimensions
        if self.dimx != other.dimx or self.dimy != other.dimy:
            raise ValueError, "Matrices must be of equal dimensions to add"
        else:
            # add if correct dimensions
            res = matrix([[]])
            res.zero(self.dimx, self.dimy)
            for i in range(self.dimx):
                for j in range(self.dimy):
                    res.value[i][j] = self.value[i][j] + other.value[i][j]
            return res
    
    def __sub__(self, other):
        # check if correct dimensions
        if self.dimx != other.dimx or self.dimy != other.dimy:
            raise ValueError, "Matrices must be of equal dimensions to subtract"
        else:
            # subtract if correct dimensions
            res = matrix([[]])
            res.zero(self.dimx, self.dimy)
            for i in range(self.dimx):
                for j in range(self.dimy):
                    res.value[i][j] = self.value[i][j] - other.value[i][j]
            return res
    
    def __mul__(self, other):
        # check if correct dimensions
        if self.dimy != other.dimx:
            raise ValueError, "Matrices must be m*n and n*p to multiply"
        else:
            # subtract if correct dimensions
            res = matrix([[]])
            res.zero(self.dimx, other.dimy)
            for i in range(self.dimx):
                for j in range(other.dimy):
                    for k in range(self.dimy):
                        res.value[i][j] += self.value[i][k] * other.value[k][j]
            return res
    
    def transpose(self):
        # compute transpose
        res = matrix([[]])
        res.zero(self.dimy, self.dimx)
        for i in range(self.dimx):
            for j in range(self.dimy):
                res.value[j][i] = self.value[i][j]
        return res
    
    # Thanks to Ernesto P. Adorio for use of Cholesky and CholeskyInverse functions
    
    def Cholesky(self, ztol=1.0e-5):
        # Computes the upper triangular Cholesky factorization of
        # a positive definite matrix.
        res = matrix([[]])
        res.zero(self.dimx, self.dimx)
        
        for i in range(self.dimx):
            S = sum([(res.value[k][i])**2 for k in range(i)])
            d = self.value[i][i] - S
            if abs(d) < ztol:
                res.value[i][i] = 0.0
            else:
                if d < 0.0:
                    raise ValueError, "Matrix not positive-definite"
                res.value[i][i] = sqrt(d)
            for j in range(i+1, self.dimx):
                S = sum([res.value[k][i] * res.value[k][j] for k in range(self.dimx)])
                if abs(S) < ztol:
                    S = 0.0
                res.value[i][j] = (self.value[i][j] - S)/res.value[i][i]
        return res
    
    def CholeskyInverse(self):
        # Computes inverse of matrix given its Cholesky upper Triangular
        # decomposition of matrix.
        res = matrix([[]])
        res.zero(self.dimx, self.dimx)
        
        # Backward step for inverse.
        for j in reversed(range(self.dimx)):
            tjj = self.value[j][j]
            S = sum([self.value[j][k]*res.value[j][k] for k in range(j+1, self.dimx)])
            res.value[j][j] = 1.0/tjj**2 - S/tjj
            for i in reversed(range(j)):
                res.value[j][i] = res.value[i][j] = -sum([self.value[i][k]*res.value[k][j] for k in range(i+1, self.dimx)])/self.value[i][i]
        return res
    
    def inverse(self):
        aux = self.Cholesky()
        res = aux.CholeskyInverse()
        return res
    
    def __repr__(self):
        return repr(self.value)
 
 
########################################
 
# Implement the filter function below
 
def kalman_filter(x, P):
    for n in range(len(measurements)):
        
        # measurement update
        Z = matrix([[measurements[n]]])
        y = Z - (H * x)
        S = H * P * H.transpose() + R
        K = P * H.transpose() * S.inverse()
        x = x + (K * y)
        
        P = (I - (K * H)) * P
 
        # prediction
        x = (F * x) + u
        P = F * P * F.transpose()
        
    return x,P
 
############################################
### use the code below to test your filter!
############################################
 
measurements = [1, 2, 3]
 
x = matrix([[0.], [0.]]) # initial state (location and velocity)
P = matrix([[1000., 0.], [0., 1000.]]) # initial uncertainty
u = matrix([[0.], [0.]]) # external motion
F = matrix([[1., 1.], [0, 1.]]) # next state function
H = matrix([[1., 0.]]) # measurement function
R = matrix([[1.]]) # measurement uncertainty
I = matrix([[1., 0.], [0., 1.]]) # identity matrix
 
print kalman_filter(x, P)
# output should be:
# x: [[3.9996664447958645], [0.9999998335552873]]
# P: [[2.3318904241194827, 0.9991676099921091], [0.9991676099921067, 0.49950058263974184]]

"""결과물
([[3.9996664447958645], [0.9999998335552873]], [[2.3318904241194827, 0.9991676099921091], [0.9991676099921067, 0.49950058263974184]])
"""
```

## 3. A formal implementation of the Kalman Filter in C++ and Eigen using state and covariance matrices for the simplest 1D motion model

> 위와 같은 C코드 

## 4. Object motion tracking in 2D by fusing noisy measurements from LIDAR and Radar sensors

> sensor fusion 깃북에 기록 