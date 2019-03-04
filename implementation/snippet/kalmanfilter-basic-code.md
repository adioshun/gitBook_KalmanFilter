# [Sensor Fusion — Part 2: Kalman Filter Code](https://towardsdatascience.com/sensor-fusion-part-2-kalman-filter-code-78b82c63dcd)

> 이론 설명은 : Article - Kalman Filter basics 참고 (Sensor Fusion — Part 1: Kalman Filter basics)

https://colab.research.google.com/drive/1SZjG07PQxJNfBKLbmGH1_ZZKjFchhfSC#scrollTo=ffMXTGuekEhA

Udacity’s github 데이터 활용 실습 (`obj_pose-laser-radar-synthetic-input.txt`) 
- 해당 데이터에는 Lidar + radar이지만 본 실습은 lidar만 활용
- 추후 exKalman filter다룬시 Lidar+radar 실습 진행 


데이터 구조 
```
For a row containing radar data, the columns are: sensor_type (R), rho_measured, phi_measured, rhodot_measured, timestamp, x_groundtruth, y_groundtruth, vx_groundtruth, vy_groundtruth, yaw_groundtruth, yawrate_groundtruth.

For a row containing lidar data, the columns are: sensor_type (L), x_measured, y_measured, timestamp, x_groundtruth, y_groundtruth, vx_groundtruth, vy_groundtruth, yaw_groundtruth, yawrate_groundtruth.
```

## 1. file Read 

```python 
#**************Importing Required Libraries*************
import numpy as np
import pandas as pd
from numpy.linalg import inv

#*************Declare Variables**************************
#Read Input File
measurements = pd.read_csv('obj_pose-laser-radar-synthetic-input.txt', header=None, delim_whitespace = True, skiprows=1)

```

옵션 : ‘skiprows’ = 1
    - 사전 정보 없음 `because, when we begin implementing KF algorithm, we don’t have any prior knowledge about state of vehicle. `
    - 첫번쨰 reading은 기본으로 설정 `In such cases usually we default our state to first reading from sensor output. `

## 2. State X 정의 
    
```python 
# Manualy copy initial readings from first row of input file.
prv_time = 1477010443000000/1000000.0
x = np.array([
        [0.312242],
        [0.5803398],
        [0],
        [0]
        ])
```
    
- X값을 파일의 첫 줄로 설정 `In below shown code, we will initialize our state X with reading from first row of input file.`
    - And as this reading has already been used, we simply skip it while reading input file. 
    
- 첫번째 입력 time을 previous Time으로 설정 `Along with initial values of state vector X, we will also use first timestamp from input file as our previous time.`
    - As explained in previous post, we need current and previous timestamps to calculate delta_t. 

- 시간 단위는 ms이므로 10^6로 나눈` Timestamps provided are in unit microseconds, which we will divide by 10^6. This has two reasons, `
    - 첫번째 이유 : 관리가 쉬움 `first, a smaller number is easier to maintain.`
    - 두번째 이유 : GT와 단위 맞추기 ` Second, the velocity groundtruth readings (and hence velocity values in our code) are in units of seconds`
    
## 3. GT & RMSE 정의 

```python
#Initialize variables to store ground truth and RMSE values
ground_truth = np.zeros([4, 1])
rmse = np.zeros([4, 1])
```

Next, we Initialize variables to store **ground truth** and **RMSE** values. 

RMSE (Root Mean Square Error) is used to judge performance of our algorithm against ground truth values.


## 4. matrix P & A 정의 

```python 
#Initialize matrices P and A
P = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1000, 0],
        [0, 0, 0, 1000]
        ])
A = np.array([
        [1.0, 0, 1.0, 0],
        [0, 1.0, 0, 1.0],
        [0, 0, 1.0, 0],
        [0, 0, 0, 1.0]
        ])
```

We initialize matrix P and A. 

- A = 속도x시간은 거리인 운동학적 공식 `Basically matrix A is used to implement kinematics equations of distance, speed and time, and `

- P = matrix P is State Covariance Matrix having variances in x, y, vx and vy as its diagonal elements. 

- P초기값의 의미 `What this initial P values mean is that`
    - we have high confidence in our positional values, indicated by relatively low variance value, and low confidence in velocity values, indicated by relatively large variance value.
    

## 5. Matrix H & I 

```python 

H = np.array([
        [1.0, 0, 0, 0],
        [0, 1.0, 0, 0]
        ])

I = np.identity(4)

z_lidar = np.zeros([2, 1])

```

- Next we define H and I matrices, which, as I explained in last post, will be 4 x 2 and 4 x 4 matrices respectively. 

- Z : LIDAR 센서 값 We define vector Z, which, as our Lidar readings will consist of 2 positional readings (x and y), will be a 2 x 1 vector.



## 6. Matrix R

```python 
R = np.array([
        [0.0225, 0],
        [0, 0.0225]
        ])
```

We define Measurement Covariance matrix R, which again, as per last post will be a 2 x 2 matrix. 

We will talk more about how to get values for **R matrix** and **noise\_ax** and **noise\_ay** in future articles.


## 7. noise\_ax, noise\_ay and matrix Q.




Next we define noise_ax, noise_ay and matrix Q.

```python 
noise_ax = 5
noise_ay = 5
Q = np.zeros([4, 4])
```

---

kinematics공식을 보면 위치와 속도에 대하여 가속도 Factor가 사용되는것을 알수 있다. `If we revisit kinematics equations, you can see that there is an acceleration factor in positional and velocity terms.`

They have been rewritten here for ease.

```
Px(t+1) = Px + delta_t * vx + 0.5 * ax * delta_t²
Py(t+1) = Py + delta_t * vy + 0.5 * ay * delta_t²
Vx(t+1) = Vx + ax * delta_t
Vy(t+1) = Vy + ay * delta_t

```


가속도는 알려진 정보가 아니므로 noise를 더할수 있다. 그리고 이 무작위 noise는 .... `Since the acceleration is unknown we can add it to the noise component, and this random noise would be expressed analytically as the last terms in the equation derived above. `



따라서 Vector V는 아래와 같이 Zero Mean과 Covariance matrix Q로 구성 할수 있다. `So, we have a random acceleration vector v in this form, which is described by a zero mean and a covariance matrix Q.`

![](https://cdn-images-1.medium.com/max/600/1*KGh-ppbBlYZH6K50MflbNQ.png)


