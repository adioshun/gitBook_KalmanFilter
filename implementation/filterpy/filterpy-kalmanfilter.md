# [FilterPy - KalmanFilter](https://filterpy.readthedocs.io/en/latest/kalman/KalmanFilter.html)


## 1. 사전 작업 

# construction the filter

- size of the **state vector** with dim_x 
- size of the **measurement** vector that you will be using with dim_z. 


## Instance Variables

You will have to assign reasonable values to all of these before running the filter. All must have dtype of **float**.

- x : ndarray (dim_x, 1), default = [0,0,0…0] :     filter state estimate
- P : ndarray (dim_x, dim_x), default eye(dim_x) :     covariance matrix
- Q : ndarray (dim_x, dim_x), default eye(dim_x) :     Process uncertainty/noise
- R : ndarray (dim_z, dim_z), default eye(dim_x) :     measurement uncertainty/noise
- H : ndarray (dim_z, dim_x) :     measurement function
- F : ndarray (dim_x, dim_x) :     state transistion matrix
- B : ndarray (dim_x, dim_u), default 0 :     control transition matrix 


## Optional Instance Variables

alpha : float

Assign a value > 1.0 to turn this into a fading memory filter.