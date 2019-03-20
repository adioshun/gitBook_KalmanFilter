# [FilterPy](https://filterpy.readthedocs.io/en/latest/index.html)

## 1. 설치

요구 사항

* FilterPy requires Numpy \[2\] and SciPy \[3\] to work. 
* The tests and examples also use matplotlib \[4\]. For testing I use py.test \[5\].

설치

```python
# with Pip
$ pip install filterpy

#or
$ git clone --depth=1 https://github.com/rlabbe/filterpy.git
$ cd filterpy
$ python setup.py install

#check 
>>> import filterpy
>>> filterpy.__version__
```

구현 모듈

* filterpy.kalman
  * Linear Kalman Filters : [[sample]](https://github.com/rlabbe/filterpy/blob/master/filterpy/kalman/tests/test_kf.py), [[sensorfusion sample]](https://github.com/rlabbe/filterpy/blob/master/filterpy/kalman/tests/test_sensor_fusion.py)
  * Extended Kalman Filter : [[sample]](https://github.com/rlabbe/filterpy/blob/master/filterpy/kalman/tests/test_ekf.py)
  * Unscented Kalman Filter : [[sample]](https://github.com/rlabbe/filterpy/blob/master/filterpy/kalman/tests/test_ukf.py)
  * Ensemble Kalman Filter
* filterpy.common
* filterpy.stats
* filterpy.monte\_carlo : Markov Chain Monte Carlo \(MCMC\) computation, mainly for particle filtering
* filterpy.discrete\_bayes
* filterpy.gh
* filterpy.memory
* filterpy.hinfinity
* filterpy.leastsq

## 2. 활용

```python
>>> from filterpy.kalman import KalmanFilter
>>> kf = KalmanFilter(dim_x=3, dim_z=1)
```



