# Particle filter (=Monte Carlo)


## 1. 정의 / 비교 

비교 
- EKF은 Gaussian 분포만을 이용하여 시스템을 모델링하였다. 
- Particle filter은 Gaussian이 아닌 임의의 분포를 다루기 위한 접근 방법

몬테카를로 & 파티클 필터 
-  Monte Carlo 방법은 난수를 이용하여 함수의 값을 확률적으로 계산하는 알고리즘이다. 
- 계산하려는 값이 close form(정확한 수식으로 계산되는 형태)이 아니거나 너무 복잡하여 근사적으로 구해야 할 때 주로 사용된다. 
- particle filter는 이러한 Monte Carlo 방법이다.




---

-[Particle Filter Explained without Equations](https://www.youtube.com/watch?v=aUkBa1zMKv4):youtube 7:30초

![](https://cdn-images-1.medium.com/max/800/1*s2kA7oclIHoCAQsao2fXhw.jpeg)

![image](https://user-images.githubusercontent.com/17797922/40173698-a45fd8ae-5a0d-11e8-8e37-681f95210626.png)


# 표윤석, ROS 로봇 프로그래밍, 350page 참고 