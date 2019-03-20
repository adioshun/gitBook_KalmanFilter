## 3. Unscented Kalman Filter User’s Guide

같은점 : Like the Kalman Filter, the Unscented Kalman Filter is an unsupervised algorithm for tracking a single target in a continuous state space. 

다른점 : The difference is that while the Kalman Filter restricts dynamics to affine functions, the Unscented Kalman Filter is designed to operate under **arbitrary dynamics**.


장점 `The advantages of the Unscented Kalman Filter implemented here are:`
- Ability to handle non-affine state transition and observation functions
- Ability to handle not-quite-Gaussian noise models
- Same computational complexity as the standard Kalman Filter

단점 `The disadvantages are:`
- No method for learning parameters
- Lack of theoretical guarantees on performance
- Inability to handle extremely non-Gaussian noise




### 3.1 Basic Usage 


### 3.2 Mathematical Formulation


