# [칼만필터_2 kalman filter_2](https://msnayana.blog.me/80107534127)


> [링크]((https://msnayana.blog.me/80107534127)에 코드 설명 포함 

```c
int main()
{ 

   float  i, x, x_next, P, P_next, K, Q, R, z;

 

   //초기화 
   P = 1;                                //P는 예측공분산 매트릭스
   Q = 1.0/100000.0;                //Q는 예측노이즈 공분산
   R = 0.1*0.1;                        //R은 측정노이즈 공분산
   x = 0.0;                              //x는 예측상태출력값

 

   printf("x:%x, z:%x", x,z);  

   for(i=0;i<20000;i++)                //2만번 반복하고 끝내자.
   {

       x_next = x;                               //x를 갱신
       P_next = P + Q;                        //다음상태 P= 현재P +  예측노이즈 공분산Q
       K = P_next / ( P_next + R );       //K는 칼만게인이다.  이것은 R(측정노이즈 공분산)값과 예측노이즈가 영향준다.

       z = -0.37727 + normal(0.1);       //z는 측정잡음으로 정규분포의 산포0.1을 가진 랜덤값이다..
       x = x_next + K * ( z - x_next );    //현상태값x은 칼만게인K과  측정잡음z 및 과거상태x에 영향받는다. 
       P = ( 1 - K ) * P_next;               //예측공분산P는 칼만게인K과  이전상태P

 

       printf("x:%x, z:%x", x,z);         //출력은  x값인 현재예측상태값이다. z는 측정잡음
   }
   return 0;
}


float normal( float  std_dev )  
{

    float uni_rand,sum,mean,nor_rand;
    int i,j,n=12;
    sum=0;
    mean = (float)random(32000)/32000;    //평균
    for( j=0; j<12; j++)
    {

          uni_rand=(float)random(32000)/32000;
          sum=sum+ uni_rand;
     }
     nor_rand=mean+ std_dev*((sum-n/2)/(sqrt((float)n/12)));
     return(nor_rand);
}
```