# 선형 & 비 선형 


선형 
- 원점을 지나는 1차직선, Degree 1 
- **직선의 특징**을 가짐 : 중첩의 원리(principle of superposition)를 다름
- 단순, 하나의 원인에는 하나의 결과가 있을 뿐이며, 결과를 보고 원인이 어떤 것이었는지 짐작할 수 있다
- eg. 한명이 10kg의 짐을 옮긴다면 2명은 20kg의 짐을 옮긴다. 
- eg. y = 3x 

비선형 
- 우리가 예상하지 못했던 훨씬 높은 차원의 현상
- eg. exponents, square root
- eg. 한명이 10kg의 짐을 옮기는데 2명은 30kg의 짐을 옮긴다. 
- eg. y = 3x^2



## 1. Linearity 

- 요구 사항 # 1 
    - addivity 
    - f(a+b) = f(a) + f(b)
- 요구 사항 # 2 
    - homogeniety 
    - f(c*x) = c*f(x)  
    
    
## 2. 중첩의 원리 

- 중첩[super (겹쳐) + position(위치시킴)]

- A라는 인자가 일으킨 현상과 B라는 인자가 일으킨 현상을 합한 것은 A와 B 두 인자가 함께 일으킨 현상과 동일하다

- 선형 미분 방정식의 해의 선형 결합이 선형 미분 방정식의 또다른 해가 된다는 원리

|![](https://t1.daumcdn.net/cfile/tistory/2761F3505550195A2D)|![](https://t1.daumcdn.net/cfile/tistory/2602794755501D042D)|
|-|-|

- 선형 함수 : 이를 중첩의 원리 적용, 이는 선형함수를 예측가능하게 만들어 준다
- 비선형 함수 : 함수의 수식이 알려지지 않았을 때, 함수값을 예측하기가 매우 어렵다는 특징


---

# [왜 Radar는 비선형 인가?](https://www.facebook.com/groups/AutonomousDrivingKR/permalink/637827600020530/)

[질문] 센서 퓨전을 보고 있는데 입력으로

- Lidar : cartesian좌표(x,y)
- Radar : polar좌표(rho, phi, drho)

설명에는 `lidar sensor are linear and radar sensors are non-linear, so we use the jacobian algorithm`라고 되어 있습니다.

[1] 왜 Radar는 Non-linear라고 하는건가요?

[2] 좌표 변환함수 쓰면 cartesian - polar 변환도 가능할텐데 굳이 Non-linear라서 jacobian을 써가면서 ex칼만을 쓸필요가 있는건가요? (Lidar는 basic 칼만을 씁니다.)

의견 부탁 드립니다.


@이종훈 좌표계 그 자체보다는 센서에 특성에 따른 데이터 분포가 linear하다 non linear하다 혹은 그에 따른 최적화 관점에서 linear로 풀 수 있다 non linear로 풀어야 된다라는 맥락


@ 윤승제 보통 superposition 이라고, f(ax+by)=af(x)+bf(y) 가 성립되면 f 는 linear 라고 하는데, polar 는 저 superposition 이 성립이 안되지 않나요?

@이종훈 어차피 라이다도 측정 자체는 극자표로 하는 경우가 많습니다. 결국 이를 변환 하거나 선형 근사 하였을 때 선형으로 표현이 잘 될 수 있는 데이터 분포를 가지냐 그렇지 않냐의 문제일 것 같습니다.


@김동민 
1. 센서 특성상 레이더에서 들어오는 극좌표값을 기존에 정의해놓은 라이다에서 사용하는 상태벡터에 퓨전을 위해 같은 좌표계를 써야해서 직교좌표계로 바꾸는 걸로 알고있습니다. 그러면 삼각함수가 들어가게되고 레이더에 대해서 non linear한 상태벡터가 만들어져 그런것 같습니다.
2.번도 위에서 설명한것과 같이 변환을 해서 계산할때 저렇게 non linear한 요소가 나와서 그런것 같습니다. 저도 유다시티 자율주행강의를 들어서 저렇게 기억하고 있는데 확실한 내용인지는 잘 모르겠네요 ㅎㅎ

@박성근 
1) 라이더/레이더 모두 거리와 각도 형태의 극좌표계로 들어올겁니다. 당연히 좌표 변환을 이용해 직계좌표계로 표현할 수 있습니다. 하지만,
2) 라이더의 경우 정확도가 극히 높기때문에, 실제 극좌표계 상에서의 노이즈가 몹시 적고, 이를 직계 좌표계로 변환하더라도, 실제 축이 변환되면서 생기는 노이즈의 변화가 없다고 봐도 무방할겁니다
3) 레이더의 경우 노이즈가 상당히 큰 센서인데, 노이즈의 분포 방향이 극좌표계 기준(거리/각도등)으로 생성될 것이고 이를 단순 직계좌표계로 변환할 경우, 베이지안 필터류에 적용할때 넣는 노이즈 값들이 실제 분포와 다른 분포를 갖게 될 수 있겠죠.
4) 이러한 이유로 원래 노이즈 데이터의 모델링을 위해 레이더의 경우에 극좌표계의 상태 방정식을 유지한 뒤, 이를 EKF나 PF와 같은 비선형 필터에 적용합니다.
5) 물론 천하무적(?) 칼만 필터는 대충 때려박아도 잘 되니 귀찮으시면 그냥 다 직계 좌표계로 변환 후 하셔도 경험상 큰 문제는 없다는..... 것이 함정이네요...

@이종훈 박성근님 의견에 동의합니다! 너무 잘 정리 해주셔서 살짝만 덧붙이면 좌표계 자체는 라이다도 종류에 따라 차이는 있지만 최종적인 출력을 cartesian좌표(x, y)로 해주는 것이지 측정자체는 극좌표를 이용한 경우가 많습니다. 다만 여기서 성근님 말씀대로 라이다는 좌표계 변환을 하여도 사실상 linear한 특성이 유지가 되기 때문에 'lidar sensor are linear'라고 표현한 것 같습니다.


---

https://sdolnote.tistory.com/entry/LinearityNonlinearityFunction

[선형이라는 것의 의미 (Linear 하다는 것의 의미)(https://sdolnote.tistory.com/entry/Linearity)

[The Difference Between Linear & Nonlinear Equations](https://sciencing.com/the-difference-between-linear-nonlinear-equations-12751668.html)