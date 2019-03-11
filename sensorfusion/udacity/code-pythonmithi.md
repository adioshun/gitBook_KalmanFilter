# [An extended Kalman Filter implementation in Python for fusing lidar and radar sensor measurements](https://github.com/mithi/fusion-ekf-python)

> mithi


## 1. [Fusion-EKF-Sample-Usage.ipynb](https://github.com/mithi/fusion-ekf-python/blob/master/Fusion-EKF-Sample-Usage.ipynb)


### 1.1 `def parse_data()`

- 입력 : txt파일 

- 출력 : two lists of DataPoint() instances 
  - all_sensor_data
  - all_ground_truths

### 1.2 `def get_state_estimations()`

- 입력 : EKF, all_sensor_data
- 출력 : all_state_estimations

#### A. `EKF.process(data)`







