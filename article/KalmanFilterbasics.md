# Sensor Fusion 


## 1. [Kalman Filter basics](https://towardsdatascience.com/sensor-fusion-part-1-kalman-filter-basics-4692a653a74c)


내용 : 칼만필터 기반 차량 추적 및 센서 퓨전 `I will try to explain Kalman filter algorithm along with an implementation example of tracking a vehicle with help of multiple sensor inputs, often termed as Sensor Fusion.`


기본 3가지 구성 요소 `Kalman filter in its most basic form consists of 3 steps. `
- A) Predict — Based on previous knowledge of a vehicle position and kinematic equations, we predict what should be the position of vehicle after time t+1. 
- B) Measurement — Get readings from sensor regarding position of vehicle and compare it with Prediction 
- C) Update — Update our knowledge about position (or state) of vehicle based on our prediction and sensor readings. 

여러 변형들 `All variants of Kalman filter are just different variants of above 3 steps, depending upon `
- different Kinematic equations you want to use and 
- different kind of sensor reading which you want to incorporate in algorithm. 

차량 추적을 예로 각 단계들을 설명 하겠다. 

### 1.1 Predict 

기본적 속도, 거리, 시간 정보로 공식 세움 `Using basic equations of velocity, distance and time, we know that if position of my vehicle at time t is at x location, then at time t+1 the vehicle will be at location x + ((t + 1) — t) * v, where v is velocity of vehicle. `
- x + ((t + 1) — t) * v 

가속도 정보를 알고 있으면 이를 고려함 `Digging in further, if we have acceleration value available with us, we can add it to above equation and update new position of vehicle to x + ((t + 1) — t) * v + 0.5 * a * ((t + 1) — t)2. `
- x + ((t + 1) — t) * v + 0.5 * a * ((t + 1) — t)2

공식 : Formalizing above equations, lets denote current state of vehicle, 
- which consists of its current location and velocity as a vector x. 
- So vector x will have 4 elements, 
    - namely px, py, vx, vy; 
    - representing position and velocity in x and y directions respectively.
    

예시를 간소화 하기 위해 2D(x,y)로 한정한다. `The positional x and y distances are with respect to our vehicle’s own position. So if we say a vehicle is at position px and py, we mean that it is at a distance of px and py from our vehicle.`

Moving on, let’ define velocity and acceleration as velocity and acceleration vectors having velocity and acceleration data in x and y direction respectively. We will denote change in time as delta_t. 

With this, our individual equations for tracking position and velocity becomes:
- Px(t+1) = Px + delta_t * vx + 0.5 * ax * delta_t 2
- Py(t+1) = Py + delta_t * vy + 0.5 * ay * delta_t 2
- Vx(t+1) = Vx + ax * delta_t
- Vy(t+1) = Vy + ay * delta_t


매트릭스 연산이 성능에 더 좋다. `Without going into details, one thing we should understand is that performing matrix operation is much more computationally effective than solving individual equation. `

A simple justification being, if we consider above 4 equations, a controller will need to perform following actions 4 times. 
- i) Create a variable Px(t+ 1), 
- ii). get value of Px 
- iii). get value of delta_t 
- iv). get value of vx 
- v). get value of ax 
- vi). perform mathematical operations 
- vii). store result. 

All these operations will have to be performed one time each for each equation. 

예 : 심부름 `To explain analogy better, let’s say your mom asks you to go to grocery and bring some Onions (being analogous to getting Px, vx, ax values from memory). `
- 양파 사와라, 도구 사와라, 토마토 사와라 등등(=매 순간 센서 리딩)...그냥 처음에 살것들 목록을 주는게 좋지 않을까? `Being the obedient child you are, you go immediately and bring it. She then asks you to cut them (analogous to mathematical operations), you do that, and then just when you were about to watch your Football game, she asks you to bring Tomatoes from store. Again, being the obedient child, you go back and bring tomatoes, gather all equipment (knives, cutting pad and a bowl to keep them), cut tomatoes and store it for her. And then she asks you to bring some fruits…and them some more. You get the gist here, suffice to say, you won’t be a happy kid that evening. Wouldn’t it have been sooooo much better if your mom had given you all the list of things to bring and instructed you to cut them before you start your tasks. Same goes for computers.`


Coming back to 1s and 0s, lets convert our 4 individual equations into following matrix form. 

So now our 4 equation gets clubbed together and they look like this.

$$ x_new = A * x + B * u $$

![](https://cdn-images-1.medium.com/max/1600/1*MaQHT-LfjvssRgZlszejQg.png)


첫행 : Taking first row from above matrix equation

$$

px\_new = [(1 * px) + (0 * py) + (delta_t * vx) + (0 * vy) ] + [ (0.5 * delta_t * delta_t * acceleration_x) + (0 * acceleration_y) ]
$$
$$
px\_new = px + delta_t * vx + 0.5 * acceleration_x * delta_t 2
$$

which is same as one we had come up with before converting individual equations to matrix form. 

Here **matrix A and B** are just matrices representing **kinematic equations** for position, velocity and acceleration.


다음 예 : 
- 추가 심부름 발생 
- 축구 경기를 봐야 함으로 나중에 가기로 함
- 언제 갈껀데? 
- **약 ** 두시간 후 (불확실성) 
`Oh, but wait..uuhh…mom is asking ..to….go…bbaacck…to grocery (and she is apologetic about it this time)…so you snap back and say hey mom, I got this football game to see, I cant go right now. And mom says so tell me (Predict) when can you go. And you reply I will go in about 2 hrs. So what’s the interesting part about this conversation you ask…well it’s the “about” part. As you can sense, you are little bit uncertain about the time when you can go back. You know it will be in 2 hrs, but there is little bit of +- 10 (or 15 or pick a number) minutes of uncertainty you are not sure about. Maybe there is lot of extra time added in game, maybe game is getting boring (read it as your team is losing) and you decide not to wait till end, or maybe it was a great victory in a derby match and you want to wait and watch after match celebration and expert analysis.`


컴퓨터에도 불확실성은 존재 `This uncertainty factor holds true for computers too. `

칼만필터의 특별한 점 A thought might have entered your mind saying, so what’s so great about Kalman filter, above mentioned kinematic equations been known for decade, we can use it in matrix form if you like it and start predicting. 

차량의 위치 예측시 불확실성도 같이 간주 해야 함 `So when we are asking computer to predict the value of our new state of vehicle, we need to ask it about its uncertainty too. `

In Kalman filter language this **uncertainty** is represented as **covariance matric**. 

> 칼만 필터에서 불확실성은 **covariance matric**로 표현 된다. 

###### State Covariance Matrix

Lets denote our State Covariance Matrix as

```python
P = [variance_px, 0, 0, 0],
    [0, variance_py, 0, 0],
    [0, 0, variance_vx, 0],
    [0, 0, 0, variance_vy]
    """
    variance_px = uncertainty in predicted x position
    variance_py = uncertainty in predicted y position 
    variance_vx = uncertainty in predicted x velocity 
    variance_vy = uncertainty in predicted y velocity  
    """
    ```

0의 의미 : 서로 비의존적이다. `All the remaining 0 terms are to say that uncertainty in one value is independent of other value, meaning to say my position x uncertainty is nothing to do with my uncertainty in y direction. `

> - 항상 비 의존적이지는 않지만 간소화를 의해 본 예시에서는 0으로 처리 `This need not be true in every case. In some case the variables you are tracking may actually depend on each other. But we will use the simplistic approach here and make other values as 0.`

This actually makes it a ‘variance’ matrix, but **covariance** is much more widely used term.

###### Noise Covariance matrix

공식이 복작해 지긴 하지만 **Noise Covariance matrix Q**도 포함 해야 한다. `At the risk of making our equations a bit complex, I want to introduce Process Noise Covariance matrix Q. `

계산시 노이즈를 고려 해야함 `In any covariance matrix calculations (which is matrix P in our case), you can have a certain amount of noise which needs to be added into it to come up with a new predicted covariance in the estimation.`

칼만 필터는 순환프로세스 이기에 매 센서 정보 수신시 **covariance matrix**와 함께 predict를 수행 해야 한다. `As Kalman filtering is a continuously iterative process, we need to keep predicting the state vector along with its covariance matrix every time we have a new reading from sensor, `
- so that we can compare the predicted value (step a) with sensor value (step b) and 
- update our information about the vehicle we are tracking (step c). 

우리는 이미 **updating state vector** 공식은 알고 있으며 **update covariance matrix** 공식은 아래와 같다. `We already have equations for updating state vector, and the equation to update covariance matrix is as given below.`


$$ State vector: x = A * x + B * u$$

$$ Covariance matrix: P = A * P * AT * Q$$

 
**step a of predicting**의 최종 공식 `And with this we conclude our step a of predicting with following two equations.`




### 1.2 Measurement

모든 시작은 센서 정보 수신 부터다 
- Now let’s say we receive a sensor reading for the position of the vehicle we are tracking. 
- Actually the sequence of operation is, we trigger Kalman filter calculation only when we receive sensor readings. 
- The sensor reading will usually have a timestamp associated with each reading. 

We read that timestamp, calculate difference between timestamp of last reading and this one. 
- Perform **prediction** as per above explanation using delta_t = timestamp difference between two readings and 
- then come to **Measurement** and **update** part for sensor input. 

Lidar센서는 위치 정보는 있지만 속도 정보는 없다. `Commonly use sensors like Lidar and Time of flight sensors will give you readings regarding position of a vehicle, but not its velocity. `
- So our sensor reading will be of form vector **z = [px, py]. **

STEP A단계에서 이 측정값(z)과 예측값(x)을 비교 한다. `As stated before, we need to compare this measurement value with the value predicted in step a. `

그러나 이 두값은 **different order**이다. `But looking at both vectors (predicted state vector x and measurement vector z), we see that both are of different order. `
- ‘x’ matrix = 4 x 1 (속도 정보 포함) 
- ‘z’ matrix = 2 x 1 

따라서, state matrix(x??)를 z와 같은 크기로 변환 하기 위해 **H Matrix**가 필요하다. `Hence to convert state matrix to same size as z, we introduce matrix “H”.`

매트릭스 연산의 기초에 따라 matrix H는 `2 x 4` 이다. `Getting into matrices basics, to convert a 4 x 1 ‘x’ matrix to 2 x 1 ‘z’ matrix (while keeping top two values as it is and removing velocity variables) we use a 2 x 4 matrix H. `

비교 공식 : So the equation comparing **predicted value** with **measurement** becomes $$y = z - Hx$$, 
- 결과로 **다름정도** 출력 `which gives us the difference between both values.`












