# 칼만필터 발전 

필터
- 측정데이타에 포함된 불확실성(noise)을  필터링하는 것
- 과거의 측정데이타(기존에 알고있던것)와  새로운 측정데이타를 사용하여   데이터에 포함된 노이즈를 제거시켜 새로운 결과를 **추정(estmate)**하는데 사용하는 알고리즘


![](https://i.stack.imgur.com/rgeEB.jpg)


> Arjun V Shenoy, `Comparison of unconstrained nonlinear state estimation techniques on a MMA polymer reactor`2009,  논문의 분류표 캡쳐후 첨부 


||선형|비선형|
|-|-|-|
|가우시안|KF|EFK<br> UKF|
|비가우시안||PF|
## 1. KF  (linear) 

- 정의 : 과거와 현재값을 가지고  재귀적(recursive)연산(data processing)을 통하여  최적 (optimal) 값을 추적하는 것이다.

The Kalman filter (KF) is a popular choice for estimating motion in robotics. Since position information is linear, standard Kalman filtering can be easily applied to the tracking problem without much difficulty.
However, most robotic motions also contain nonlinearity requiring a modification to the KF.

장점 `The advantages of Kalman Filter are:`
- No need to provide labeled training data
- Ability to handle noisy observations

단점` The disadvantages are:`
- Computational complexity is cubic in the size of the state space
- Parameter optimization is non-convex and can thus only find local optima
- Inability to cope with non-Gaussian noise


## 2. EKF (nonlinear)

> Taylor expansion을 이용하여 선형화 

The extended Kalman filter (EKF) provides this modification by linearizing all nonlinear models (i.e., process and measurement models) so the traditional KF can be applied.

Unfortunately, the EKF has two important potential drawbacks. First, the derivation of the Jacobian matrices, the linear approximates to the nonlinear functions, can be complex causing implementation difficulties. Second, these linearizations can lead to instability if the time-step intervals are not sufficiently small.

## 3. UKF (nonlinear)

> unscented transform을 이용하여 선형화, EKF의 성능 향상 버젼

To address these limitations, the unscented Kalman filter (UKF) was developed. The UKF operates on the premise that it is easier to approximate a Gaussian distribution than it is to approximate an arbitrary nonlinear function. Instead of linearizing using Jacobian matrices, the UKF using a deterministic sampling approach to capture the mean and covariance estimates with a minimal set of sample points.
The UKF is a powerful nonlinear estimation technique and has been shown to be a superior alternative to the EKF in a variety of applications.


장점 `The advantages of the Unscented Kalman Filter implemented here are:`
- Ability to handle non-affine state transition and observation functions
- Ability to handle not-quite-Gaussian noise models
- Same computational complexity as the standard Kalman Filter

단점 `The disadvantages are:`
- No method for learning parameters
- Lack of theoretical guarantees on performance
- Inability to handle extremely non-Gaussian noise


> 세개의 칼만필터 분석 : `[A Comparison of Unscented and Extended Kalman Filtering for Estimating Quaternion Motion.](https://ieeexplore.ieee.org/document/1243440)`




---

[Summary on the Kalman Filter & Friends: KF, EKF, UKF, EIF, SEIF](http://ais.informatik.uni-freiburg.de/teaching/ws12/mapping/pdf/slam08-kf-wrapup.pdf): ppt