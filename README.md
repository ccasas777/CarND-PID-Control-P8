# Self-Driving Car Engineer Nanodegree Program

## PID Controller

Given a driving simulator which provides the cross track error (CTE) and the velocity (mph), the goal of this project is to develop a PID controller in c++ that successfully drives the vehicle around the track. In order to navigate the car around the track. I have implemented 2 PID controllers, one for steering angle control and the other for speed control.

## Tuning

The most important part of the project is to tune the hyperparameters. This can be done by different methods such as manual tuning, Zieglor-Nichols tuning, SGD, Twiddle. I have done manual tuning. Manual tuning is hard but, the process of tuning help us better understand every single effect of PID parameters. The following table summerizes the effect of each parameter on the system.


#### Proportional (P): 
This parameter controls the error proportionally. Increasing the proportional gain has the effect of proportionally increasing the control signal for the same level of error.

#### Integral (I): 
This parameter controls the accumulating error. Addition of this term reduces the steady state error. If there is a bias in the system, the integrator builds and builds, thereby increasing the control signal and driving the error down. 

#### Derivative (D): 
This parameter controls the rate of change of error. This anticipation tends to add damping to the system, thereby decreasing overshoot, but the negative effect would cause slowly reaching the target(CTE = 0).


##Conditional analysis

#### In the Steady state:
(P) : almost no work.

(I) : Mainly work for the bias/offset/stably turning. 

(D) : no work.
#### being the perturbation 
(ex: changing the state from going straight to turn left or from turning back to going straight)

(P) : mainly work.

(I) : almost no work.

(D) : anti-work to (P).



## Tuning method

* Set all gains to zero.
* Increase the P gain until the response to a disturbance is steady oscillation.
* Increase the D gain until the the oscillations go away (i.e. it's critically damped).
* Repeat steps 2 and 3 until increasing the D gain does not stop the oscillations.
* Set P and D to the last stable values.
* Increase the I gain until it brings you to the setpoint with the number of oscillations desired.

##Judgement method

Initially, I only set the "total_cte" as the judgement of the pid-parameters. However, I found when I get a smallest cte but the car would have small oscillations. There is no sense for the final result. So, I add another judgement of the "total_angle". It means that the current_angle - previous car_angle. So if have smallest oscillations in the round, I would get the smallest "total_angle".

####case 1 :
|   | Steering  | Speed  |
|---|---|---|
| Kp  |  0.25 |  0.18 |
| Ki  | 0.0022  |  0.007 |
| Kd  | 5.0  |  0.0 |

In this case, I have the smallest total_cte <600, but the total_angle is >60000 that is a relative value.
####case 2:
 
|   | Steering  | Speed  |
|---|---|---|
| Kp  |  0.08 |  0.18 |
| Ki  | 0.003  |  0.007 |
| Kd  | 3.0  |  0.0 |

In this case, I have the not smallest total_cte <1000, but the total_angle reduce to <30000 (-50%+)

it means that the car get the proper curve when driving, though not driving on the perfect line(cte=0). I think the setting would more fit the driving behavior in the reality.


Here is the video that I implemented the project in the two cases. Which one would you prefer to?
./
https://youtu.be/_-sLaUVVxpI

Notice: 

The performance of the controller would be different when using different computer or different setting of the simulator even in the same parameters.  I wrote the project in my local computer. I found that there is different peformance when I run simulator in the udacity computer, but the car still drive safely to pass the track fitting the criterions.

