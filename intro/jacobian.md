# Jacobian 

## 1. 배경 

- Joint space - rotatation matrix - jacobian 

- Joint space(로봇의 움직임, 로봇중) \<\- **jacobian** \-\> Task space(3차원 공간의 움직임, 인간 중심)

    - Forward Kinematics (정기구학,FK) : Joint > Task  : x = f(q)
    - Inverse Kinematics (역기구학,IK) : Task > Joint  : q = f-1(x)
    

    
### 1.1 Forward Kinematics
- 아래와 같은 공식만 알고 있으면 쉽게 해경 
- eg. 관절각도가 oo도 일 때 손끝의 위치(x,y,z)는 무엇이냐?
    

![](http://1.bp.blogspot.com/-Q6mQjXXuSZg/UqgB7LK6r6I/AAAAAAAACFU/7yX_fFcz8b8/s400/daum_equation_1386742162854.png)



### 1.2 inverse kinematics
- 상대적으로 어려움 : 위 식에서 c1, c2, c3를 구해야 하는 문제 
- eg.  x,y,z를 넣어줬을 때 어떠한 각도값들이 필요한지 q = f-1(x)를 구하는 것
- 이를 해결 하기 위함이 **Jacobian ** 

## 2. Jacobian

- 복잡하게 얽혀있는 식을 미분을 통해 linear approximation시킴으로써 간단한 근사 선형식으로 만들어주는 것

- 미분기울기를 통해 △x 후의 y값을 선형 근사하여 예측하는 것과 비슷한 원리

> wiki 정의 : "The Jacobian matrix is the matrix of all first-order partial derivatives of a vector-valued function" 
> 다변수 함수일 때의 미분값











---

- [자코비안(Jacobian)이란 무엇인가](http://t-robotics.blogspot.com/2013/12/jacobian.html)