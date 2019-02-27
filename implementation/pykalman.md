# pykalman

> [github](https://github.com/pykalman/pykalman), [메뉴얼](https://pykalman.github.io/)


simple [Kalman Filter], [Kalman Smoother], and [EM] library for Python


## 1. Installation 

```python 
# 위존 패키지 
$ easy_install numpy scipy Sphinx numpydoc nose pykalman

# 설치 
$ easy_install pykalman

# 설치 (from code)
$ git clone git@github.com:pykalman/pykalman.git pykalman
$ cd pykalman
$ sudo python setup.py install
```

## 2. Kalman Filter User’s Guide

### 2.1 Basic Usage 

구현물 `This module implements two algorithms for tracking / one algorithm for parameter learner: `
- the Kalman Filter and 
- Kalman Smoother. 
- EM algorithm : model parameters learner 


칼만 스무터 파라미터 `In order to apply the **Kalman Smoother**, one need only specify `
- the size of the state and observation space. 

칼만 스무터 파라미터 설정 법 `This can be done `
- directly by setting `n_dim_state` or `n_dim_obs` or 
- indirectly by specifying an initial value for any of the model parameters from which the former can be derived:

```python
>>> from pykalman import KalmanFilter
>>> kf = KalmanFilter(initial_state_mean=0, n_dim_obs=2)
```

기존 칼만 필터는 모델 파라미터를 사전에 알고 있어야 했지만, 학습을 통해서도 가능하다`KalmanFilter.em()`. `The traditional Kalman Filter assumes that model parameters are known beforehand. The KalmanFilter class however can learn parameters using KalmanFilter.em() (fitting is optional). `

Then the hidden sequence of states can be predicted using KalmanFilter.smooth():

```python 
>>> measurements = [[1,0], [0,0], [0,1]]
>>> kf.em(measurements).smooth([[2,0], [2,1], [2,2]])[0]
array([[ 0.85819709],
       [ 1.77811829],
       [ 2.19537816]])
```

캄만 필터 파라미터 `The Kalman Filter is parameterized by `
- 3 arrays for state transitions, 
- 3 for measurements, and 
- 2 more for initial conditions. 


> 참고 코드 : `examples/standard/plot_sin.py` Tracking a sine signal


#### A. Choosing Parameters

보통 사전에 정의된 초기값을 사용 한다. `The KalmanFilter class can thus be initialized with any subset of the usual model parameters and used without fitting.  Sensible defaults values are given for all unspecified parameters `
- zeros for all 1-dimensional arrays and 
- identity matrices for all 2-dimensional arrays


A Kalman Filter/Smoother is fully specified by 
- its initial conditions (initial_state_mean and initial_state_covariance), 
- its transition parameters (transition_matrices, transition_offsets, transition_covariance), and 
- its observation parameters (observation_matrices, observation_offsets, observation_covariance). 

These parameters define a probabilistic model from which the unobserved states and observed measurements are assumed to be sampled from. 


The following code illustrates in one dimension what this process is.

```python 
from scipy.stats import norm
import numpy as np
states = np.zeros((n_timesteps, n_dim_state))
measurements = np.zeros((n_timesteps, n_dim_obs))
for t in range(n_timesteps-1):
   if t == 0:
      states[t] = norm.rvs(initial_state_mean, np.sqrt(initial_state_covariance))
      measurements[t] = (
          np.dot(observation_matrices[t], states[t])
          + observation_offsets[t]
          + norm.rvs(0, np.sqrt(observation_covariance))
      )
  states[t+1] = (
      np.dot(transition_matrices[t], states[t])
      + transition_offsets[t]
      + norm.rvs(0, np.sqrt(transition_covariance))
  )
  measurements[t+1] = (
      np.dot(observation_matrices[t+1], states[t+1])
      + observation_offsets[t+1]
      + norm.rvs(np.sqrt(observation_covariance))
  )
```

이러한 값을 찾는것은 쉬운 일이 아니며 `section on fitting`에서 자세히 살펴볼 것이다. `The selection of these variables is not an easy one, and, as shall be explained in the section on fitting,` should not be left to KalmanFilter.em() alone. 

If one ignores the random noise, the parameters dictate that the next state and the current measurement should be an affine function of the current state. The additive noise term is then simply a way to deal with unaccounted error.

모델 파라미터의 간단한 예는 떨어지는 공과 같다. `A simple example to illustrate the model parameters is a free falling ball in one dimension. `

상태벡터는 위치, 속도, 가속도로 표현될수 있으며 `transition matrix`는 아래 공식으로 구할수 있다. `The state vector can be represented by the position, velocity, and acceleration of the ball, and the transition matrix is defined by the equation:`
```python 
position[t+dt] = position[t] + velocity[t] dt + 0.5 acceleration[t] dt^2
```

Taking the zeroth, first, and second derivative of the above equation with respect to dt gives the rows of transition matrix:

```python
A = np.array([[1, t, 0.5 * (t**2)],
              [0, 1,            t],
              [0, 0,            1]])
```

We may also set the transition offset to zero for the position and velocity components and -9.8 for the acceleration component in order to account for gravity’s pull.

It is often very difficult to guess what appropriate values are for for the transition and observation covariance, so it is common to use some constant multiplied by the identity matrix. 

Increasing this constant is equivalent to saying you believe there is more noise in the system. 

This constant is the amount of variance you expect to see along each dimension during state transitions and measurements, respectively.



#### B. Inferring States

The KalmanFilter class comes equipped with two algorithms for prediction: 
- the Kalman Filter and 
- the Kalman Smoother. 

차이점 : While the former(칼만필터) can be updated recursively (making it ideal for online state estimation), the latter(칼만스무스) can only be done in batch. 
- These two algorithms are accessible via KalmanFilter.filter(), KalmanFilter.filter_update(), and KalmanFilter.smooth().


칼만 스무스 : Functionally, Kalman Smoother should always be preferred. Unlike the Kalman Filter, the Smoother is able to incorporate “future” measurements as well as past ones at the same computational cost of O(Td^3) where 
- T is the number of time steps and 
- d is the dimensionality of the state space. 

칼만 필터가 더 좋은 이유 : The only reason to prefer the Kalman Filter over the Smoother is in its ability to incorporate new measurements in an online manner:

```python 
>>> means, covariances = kf.filter(measurements)
>>> next_mean, next_covariance = kf.filter_update(
    means[-1], covariances[-1], new_measurement
)
```

두 알고리즘 모두 시간에 따라 가변적인 파라미터를 사용할수 있다. `Both the Kalman Filter and Kalman Smoother are able to use parameters which vary with time. `
- In order to use this, one need only pass in an array `n_timesteps` in length along its first axis:

```python 
>>> transition_offsets = [[-1], [0], [1], [2]]
>>> kf = KalmanFilter(transition_offsets=transition_offsets, n_dim_obs=1)
```

> 참고 코드 : `examples/standard/plot_online.py` Online State Estimation
> 참고 코드 : `examples/standard/plot_filter.py` Filtering and Smoothing


#### C. Optimizing Parameters

In addition to the Kalman Filter and Kalman Smoother, the `KalmanFilter class` implements the **Expectation-Maximization algorithm**. 

This iterative algorithm is a way to maximize the likelihood of the observed measurements (recall the probabilistic model induced by the model parameters), which is unfortunately a non-convex optimization problem. 


EM알고리즘 사용시 고려 할점 #1 : This means that even when the EM algorithm converges, there is no guarantee that it has converged to an optimal value. 
- Thus it is important to select good initial parameter values.

EM알고리즘 사용시 고려 할점 #2 : A second consideration when using the EM algorithm is that the algorithm lacks regularization, meaning that parameter values may diverge to infinity in order to make the measurements more likely. 

Thus it is important to choose which parameters to optimize via the `em_vars` parameter of KalmanFilter. 


For example, in order to only optimize the `transition` and `observation covariance matrices`, one may instantiate KalmanFilter like so:

```
>>> kf = KalmanFilter(em_vars=['transition_covariance', 'observation_covariance'])
```

It is customary optimize only the `transition_covariance`, `observation_covariance`, `initial_state_mean`, and `initial_state_covariance`, which is the default when `em_vars` is unspecified. 

오버피팅 방지를 위해 EM알고리즘의 `반복 횟수`를 지정 할수도 있다. `In order to avoid overfitting, it is also possible to specify the `number of iterations` of the EM algorithm to run during fitting:`

```python 
>>> kf.em(X, n_iter=5)
```

Each iteration of the EM algorithm requires running the Kalman Smoother anew, so its computational complexity is O(Tnd^3) 
- where T is the number of time steps, 
- n is the number of iterations, and 
- d is the size of the state space.


> 참고 코드 : `examples/standard/plot_em.py` Using the EM Algorithm


#### D. Missing Measurements

실환경에서 센서 데이터가 수집 되지 않는 경우는 흔하며 이에 대처 하는 방법도 적용 되어 있다. `In real world systems, it is common to have sensors occasionally fail. The Kalman Filter, Kalman Smoother, and EM algorithm are all equipped to handle this scenario. `

방법 : To make use of it, one only need apply a NumPy mask to the measurement at the missing time step:

```python 
>>> from numpy import ma
>>> X = ma.array([1,2,3])
>>> X[1] = ma.masked  # hide measurement at time step 1
>>> kf.em(X).smooth(X)
```

> 참고 코드 : `examples/standard/plot_missing.py` State Estimation with Missing Observations


### 2.2 Mathematical Formulation


## 3. Unscented Kalman Filter User’s Guide

### 3.1 Basic Usage 


### 3.2 Mathematical Formulation




