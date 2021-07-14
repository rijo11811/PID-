# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

## Dependencies

* cmake >= 3.5th
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

Fellow students have put together a guide to Windows set-up for the project [here](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/files/Kidnapped_Vehicle_Windows_Setup.pdf) if the environment you have set up for the Sensor Fusion projects does not work for this project. There's also an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3).

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

## Introduction:
The project aims to create a PID controller for a car which should keep it centered on the calculated trajectory. The PID controller is tested on car running in an udacity simulator. 

## Ruberic points discussion:

# code should compile:
the code compiles without any errors.

# Impilmetaion:
The PID controller is designed using the techniques discussed in the nanodegree program PID section.

# Discussion:

P or proportional componenet of  PID controller is the primary error correction coefficint. It creates correction for the car to take it into the opposite direction of CTE. 

D or differnential component of PID controller minimises oscillations cretated by the propotiona ,P, corrections. Finsing the correct value for differential error coefficient is central to a controled drive around the track. I used it to reduce the oscillations in the car's motion.

I or Integral component of the cotroller helps to tackle the steady state error which is central to increasing the overall accurecy of the controller. In this project, I component helps the car to stay centered to the lane during most part of the drive.

# Choosing the hyperparameters:
# P
I used manual tuning approch in this PID controller project. First, step was to increase the P coefficient to a point where it induces oscillations in the motion of the car, while keeping other coefficints as 0. This value turned out to be 0.05.

[v1]: ./videos/P-alone.mp4 "P alone"
![alt text][v1]

# D
In ordered to dampen the oscillations, I slowly increased the Differntial error coeficientto a point where it countered oscillations generated by the proportal part of the P controller.Thsi value turned out to be 1.15. 
# I
The performance of the PID controller at this stage with Kp = 0.05, Kd = 1.15 and Ki= 0 was decent with car occasionally crossing the yellow marker and possible drive over the road boundary in turns. This issue was resolved by tuning the I coefficint of the PID controller. 

In case of scenario where the target value is constant, the Integral error considers all the past erros, however in our case where the target value is constantly changing, kepping track of all the past errrors does not make sense. Therfore , I desided to use only past 10 erros for caculating the integral error. Thsi was achived by using a vector which can only store 10 elemnet s at a timne and deletes the oldest error if the limit is exceeded. By tuning the Ki, I reached at a value of 0.01, which helped keeping the car cented during most of its drive.

Thus the final values are: Kp = 0.05, Ki = 0.01 ,Kd = 1.15.











