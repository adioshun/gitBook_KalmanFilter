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

### 2.1 여러가지 정의 

- 복잡하게 얽혀있는 식을 미분을 통해 linear approximation시킴으로써 간단한 근사 선형식으로 만들어주는 것
- 미분기울기를 통해 △x 후의 y값을 선형 근사하여 예측하는 것과 비슷한 원리
- "The Jacobian matrix is the matrix of all first-order partial derivatives of a vector-valued function" 
- 다변수 함수일 때의 미분값
- 편미분을 간단하게 수행 : 편미분을 할 변수들이 많고, 그 변수들로 이루어져 있는 함수가 많을 때, 
    - 단순히 곱해서 더하는 폼으로 만들어 놓을 수 있는 것

### 2.2 공식 

####  A.  'n'개의 변수를 가진 함수가 'm'개 있다는 가정

![](https://postfiles.pstatic.net/20140512_105/jinohpark79_1399895378163qBl6n_GIF/jacobian1.GIF?type=w2)


#### B. 모두 편미분하려면 아래 식의=  J와 같은 벡터로 표현 이것이 Jacobian matrix 


![](https://postfiles.pstatic.net/20140512_15/jinohpark79_1399895493407FS5R1_GIF/jacobian2.GIF?type=w2)


### 2.3 예시 

#### A. [기본 변](https://en.wikipedia.org/wiki/Jacobian_matrix_and_determinant)


![](https://i.stack.imgur.com/6RBkW.png)

#### B. [polar-Cartesian transformation](https://en.wikipedia.org/wiki/Jacobian_matrix_and_determinant)


![](https://i.imgur.com/DeqD2Zu.png)



---

```python 
from pprint import pprint
from numpy import array, zeros, diag, diagflat, dot

def jacobi(A,b,N=25,x=None):
    """Solves the equation Ax=b via the Jacobi iterative method."""
    # Create an initial guess if needed                                                                                                                                                            
    if x is None:
        x = zeros(len(A[0]))

    # Create a vector of the diagonal elements of A                                                                                                                                                
    # and subtract them from A                                                                                                                                                                     
    D = diag(A)
    R = A - diagflat(D)

    # Iterate for N times                                                                                                                                                                          
    for i in range(N):
        x = (b - dot(R,x)) / D
    return x

A = array([[2.0,1.0],[5.0,7.0]])
b = array([11.0,13.0])
guess = array([1.0,1.0])

sol = jacobi(A,b,N=25,x=guess)

print "A:"
pprint(A)

print "b:"
pprint(b)

print "x:"
pprint(sol)


"""출력 
A:
array([[ 2.,  1.],
       [ 5.,  7.]])
b:
array([ 11.,  13.])
x:
array([ 7.11110202, -3.22220342])
"""
```


---

- [자코비안(Jacobian)이란 무엇인가](http://t-robotics.blogspot.com/2013/12/jacobian.html)

- [편미분을 간단하게! Jacobian Matrix](https://blog.naver.com/jinohpark79/110190680093)

- [Part 19: The Jacobian](https://home.wlu.edu/~levys/kalman_tutorial/kalman_19.html)

- [Jacobi Method in Python and NumPy](https://www.quantstart.com/articles/Jacobi-Method-in-Python-and-NumPy)

- [youtube] [Jacobian#1야코비안-기저변환과 넓이의 측정](https://www.youtube.com/watch?v=kzKQUVNEPhc&list=PL0q7DjoZohFuMRNxE9nvU16nx607cRGxI) 

- khan [The Jacobian Determinant](https://www.youtube.com/watch?v=p46QWyHQE6M)