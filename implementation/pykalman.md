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

This module implements two algorithms for tracking: 
- the Kalman Filter and 
- Kalman Smoother. 
- EM algorithm : model parameters learner 


In order to apply the **Kalman Smoother**, one need only specify 
- the size of the state and observation space. 

This can be done 
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

The Kalman Filter is parameterized by 
- 3 arrays for state transitions, 
- 3 for measurements, and 
- 2 more for initial conditions. 


> 참고 코드 : `examples/standard/plot_sin.py` Tracking a sine signal



#### A. Choosing Parameters


#### B. Inferring States


#### C. Optimizing Parameters


#### D. Missing Measurements




### 2.2 Mathematical Formulation


## 3. Unscented Kalman Filter User’s Guide

### 3.1 Basic Usage 


### 3.2 Mathematical Formulation




