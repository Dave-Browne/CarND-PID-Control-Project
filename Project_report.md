## Project Report

### Choosing PID coefficients
Speed: Speed control was set to Kp=0.1, Kd=0.01 and Ki=0 (Integral control is not necessary). It is good to have a fixed speed controller in order to 'Twiddle' the steering coefficients. A maximum speed of 50mph is chosen. The ideal speed is modelled using the following formula `x = max_speed - 0.7*max_speed*sqrt(fabs(steer));`. This non-linear model gives more realistic results compared to a linear model.

Steering: Initially the steering coefficients for `Kp, Kd, Ki` were chosen as `0.2, 0.3, 0.004` respectively. These were given as a reasonable starting point from lectures.
The Twiddle algorithm is set up to test the PID steering coefficients over a cycle of 300 car movements in the Udacity simulator, whereafter the simulator resets, one parameter is updated and the 300 step cycle runs again. If the accumulated error `curr_err` over the cycle is less than the `best_error`, the parameters which gave that result are kept.

Note: In order to Twiddle the steering parameters, set `bool training = true;` on line 59 of main.cpp. `bool training = false` will instruct the simulator to drive normally with the initialised PID coefficients.

### The effect of PID coefficients
The proportional `P` term is directly proportional to the cross track error (cte), so if the car is far from the center of the road, it will steer heavily towards the center. Using this term on it's own results in overshooting the target.

The differential `D` term is proportional to the difference between the previous cte and the current cte. So if the car is going straight along the center lane, it's change in cte will be negligable and the differential term will be close to 0. If however the car is driving at a maximum angle to the center lane at a high speed, it will have a high delta cte and therefore influence the new steering angle significantly. This reduces large overshoot, but also introduces 'zig-zagging' of the car.

The integral `I` term is used to correct the cars error if it has been off target for a significant time period. It is proportional to the accumulated cte. So as long as the car is offset to one side of the center lane, the integral term will increase it's influence on the new steering angle to reduce the subsequent cte.
