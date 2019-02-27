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






#### B. Inferring States


#### C. Optimizing Parameters


#### D. Missing Measurements




### 2.2 Mathematical Formulation


## 3. Unscented Kalman Filter User’s Guide

### 3.1 Basic Usage 


### 3.2 Mathematical Formulation




