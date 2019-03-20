# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

### Goals
The objective of this project is to implement a PID controller that can safely navigate around the driving track 

#### Input
We are provided  the following a program inputs:
* The Cross-track error of the car

#### Expectations
* The vehicle must drive safely around the track

## Reflection
---
The PID controller provides for a combination of 3 driving parameters and how they affect how hard (steering angle) a 
car steers relative to the Cross Track Error (CTE).

### How the components affects my implementation
##### P component (Proportional)
This component steers the car directly proportional to the cross track error.
It tries to orient the car to the center of the car's trajectory. Higher values gets the car swiftly to the center 
of the track, while lower values slows down the orientation to the center.
```
cte * -tau
```
 
 ##### I Component (Integral)
 This component helps to offset systematic bias in the car's model that may prevent the car from directly reaching
 the center of the track. This bias may appear in the form of wheel alignment errors. A human driver intuitively corrects 
 for this error by steering harder in the opposite direction of the error. For an autonomous vehicle, we offset this
 error with the I component that is proportional to the cumulative CTE.
 
 ```
 sum_cte = SUMMATION(cte)
sum_cte * -tau_i
```

#### D Component (Differential)
This component helps to intercept the driving trajectory by opposing the P component as the car approaches the driving trajectory.
It is in the form of a differential that makes the car approach the track's center line while reducing the tendency to 
overshoot it. 

### Selecting my Parameters
I used a combination of manual tuning and Twiddle to achieve drivable parameters.
I trialled various *P* values to achieve a value capable of nudging the car towards the center. I discovered that there
is very negligible wheel alignment in the simulator there by no need for a *I* component. I selected a random *D* value.
I used this value as a starting point in my Twiddle algorithm. This can be found in the `twiddle.cpp` file. 

> The video below shows how my parameters drive the car around the car.
 
 [![PID Controller][image1]](https://youtu.be/U9Yq8vmvu9I)
 
 [image1]: ./images/pid-driving.png "PID Controller"

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 
