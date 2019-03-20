---
https://github.com/mithi/fusion-ekf-python/blob/master/Fusion-EKF-Sample-Usage.ipynb

```python
lidar_R = np.matrix([[0.01, 0], 
                     [0, 0.01]])

radar_R = np.matrix([[0.01, 0, 0], 
                     [0, 1.0e-6, 0], 
                     [0, 0, 0.01]])

lidar_H = np.matrix([[1, 0, 0, 0],
                     [0, 1, 0, 0]])

P = np.matrix([[1, 0, 0, 0],
               [0, 1, 0, 0],
               [0, 0, 1000, 0], 
               [0, 0, 0, 1000]])

Q = np.matrix(np.zeros([4, 4]))
F = np.matrix(np.eye(4))

d = {
  'number_of_states': 4, 
  'initial_process_matrix': P,
  'radar_covariance_matrix': radar_R,
  'lidar_covariance_matrix': lidar_R, 
  'lidar_transition_matrix': lidar_H,
  'inital_state_transition_matrix': F,
  'initial_noise_matrix': Q, 
  'acceleration_noise_x': 5, 
  'acceleration_noise_y': 5
}

EKF1 = FusionEKF(d)
EKF2 = FusionEKF(d)

all_state_estimations = get_state_estimations(EKF1, all_sensor_data)

```


