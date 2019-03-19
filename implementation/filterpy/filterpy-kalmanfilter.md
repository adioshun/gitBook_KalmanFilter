# [FilterPy - KalmanFilter](https://filterpy.readthedocs.io/en/latest/kalman/KalmanFilter.html)


## 1. 사전 작업 

### 1.1 construction the filter

- size of the **state vector** with dim_x 
- size of the **measurement** vector that you will be using with dim_z. 


### 1.2 Instance Variables

You will have to assign reasonable values to all of these before running the filter. 

> All must have dtype of **float**.

- x : ndarray (dim_x, 1), default = [0,0,0…0] :     filter state estimate
- P : ndarray (dim_x, dim_x), default eye(dim_x) :     covariance matrix
- Q : ndarray (dim_x, dim_x), default eye(dim_x) :     Process uncertainty/noise
- R : ndarray (dim_z, dim_z), default eye(dim_x) :     measurement uncertainty/noise
- H : ndarray (dim_z, dim_x) :     measurement function
- F : ndarray (dim_x, dim_x) :     state transistion matrix
- B : ndarray (dim_x, dim_u), default 0 :     control transition matrix 


Optional Instance Variables

- alpha : float : Assign a value > 1.0 to turn this into a fading memory filter.

Read-only Instance Variables

- K : ndarray :     Kalman gain that was used in the most recent update() call.
- y : ndarray :     Residual calculated in the most recent update() call. 
    - I.e., the different between the measurement and the current estimated state projected into measurement space (z - Hx)
- S : ndarray :     System uncertainty projected into measurement space. 
    - I.e., HPH’ + R. Probably not very useful, but it is here if you want it.
- likelihood : float :     Likelihood of last measurment update.
- log_likelihood : float :     Log likelihood of last measurment update. 

## 2. Example 

- Here is a filter that tracks position and velocity using a sensor that only reads position.

### 2.1 construction the filter

```python 
from filterpy.kalman import KalmanFilter
f = KalmanFilter (dim_x=2, dim_z=1)
"""

dim_x : int

    Number of state variables for the Kalman filter. For example, if you are tracking the position and velocity of an object in two dimensions, dim_x would be 4. This is used to set the default size of P, Q, and u
dim_z : int

    Number of of measurement inputs. For example, if the sensor provides you with position in (x,y), dim_z would be 2.
dim_u : int (optional)

    size of the control input, if it is being used. Default value of 0 indicates it is not used.
compute_log_likelihood : bool (default = True)

    Computes log likelihood by default, but this can be a slow computation, so if you never use it you can turn this computation off.

"""
```

### 2.2 Instance Variables

```python 

# 1. Assign the initial value for the state (position and velocity). You can do this with a two dimensional array like so:
f.x = np.array([[2.],    # position
                [0.]])   # velocity

#or 
f.x = np.array([2., 0.])


# 2. Define the state transition matrix:

f.F = np.array([[1.,1.],
                [0.,1.]])

# 3. Define the measurement function:

f.H = np.array([[1.,0.]])

# 4. Define the covariance matrix. 
## Here I take advantage of the fact that P already contains np.eye(dim_x), and just multiply by the uncertainty:

f.P *= 1000.

### I could have written:

f.P = np.array([[1000.,    0.],
                [   0., 1000.] ])

### You decide which is more readable and understandable.

# 5. Now assign the measurement noise. 
### Here the dimension is 1x1, so I can use a scalar

f.R = 5

### I could have done this instead:

f.R = np.array([[5.]])

### Note that this must be a 2 dimensional array, as must all the matrices.

# 6. Finally, I will assign the process noise. Here I will take advantage of another FilterPy library function:
from filterpy.common import Q_discrete_white_noise
f.Q = Q_discrete_white_noise(dim=2, dt=0.1, var=0.13)


```

### 2.3 실행 

```python
z = get_sensor_reading()
f.predict()
f.update(z)

do_something_with_estimate (f.x)

# Example
while True:
    z, R = read_sensor()
    x, P = predict(x, P, F, Q)
    x, P = update(x, P, z, R, H)


```