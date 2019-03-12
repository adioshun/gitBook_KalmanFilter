# [What are all those matrices for the Kalman filter? ](https://discussions.udacity.com/t/what-are-all-those-matrices-for-the-kalman-filter-part-i-x-f-p-h-r-u/57046)

> Capital letters are for 2-D matrices, 
> bold lower case are for vectors, which are 1-D matrices.

Part I: x, F, P, H, R, u

## 1. Vector x, the variables

**x** is the values of all the variables you’re considering in your system. 

for example
- It might be position and velocity. 
- if you were using a Kalman filter to model polling results, it might be Rick Santorum’s support and momentum in Ohio Presidential polls. 
- It might be a sick person’s temperature and how fast the temperature is going up. 

It’s a vector. Let’s say it has n elements— that will be important for the size of other matrices.

For example, let’s say we’re modeling a sick person’s temperature. She’s lying there in the hospital bed with three temperature sensors stuck to her, and we want to see if her temperature is going up fast, in which case we’ll call a nurse. (For purposes of this example, we’ll assume that in the time period we’re observing, the rate of rise is constant.)

We might initialize x with a temperature of 98.6 and a temperature rising rate of 0. The Kalman filter algorithm will change x—that’s the whole point, to find the correct values for x.