## Self-Driving Car - PID Controller

[![Build Status](https://travis-ci.org/joemccann/dillinger.svg?branch=master)](https://travis-ci.org/joemccann/dillinger)

In this repository, a `PID controller` is implemented by using C++ to drive a steering angle for a vehicle in a circular track where has sharp left and right turns. The system must be able to stay the car in the center of the lane and try to take smooth left and right without running over the edges of the road. 
One of the critical parts of the project is working on a robotic machine is the control module. The control module defines the action, which the robotic system performs to achieve a task. These actions can vary on the type of system and type of job. A little more complex system needs a control system to move around. Highly complex systems such as self-driving cars, product manufacturing factory units require control modules for multiple tasks at the same time. 
In this case, there is a fantastic controller that is named `PID` which consist of three main parts such as
`Proportional (P)`, `Differential (D)`, and `Integral (I)`. This type of controller is the most popular controller and is used in applications across domains.

### PID Controller Principle
Satisfying a boundary value issue is the main principle of the working of a `PID controller`. The most common instances are either decreasing the total error or increasing of the total gain in the system. These errors or gain problems are represented mathematically and are used to govern the effect to `P`, `I`, and `D` components of a `PID controller` as below:
* `Proportional (P)`: Mathematically, the `P` component establishes a linear relationship with the problem. The effect of the `P` component is equivalent to the value of the problem at the current time step.
* `Differential (D)`: Mathematically, the `D` component establishes a linear relationship with the rate of change of problem. The effect of the D component is proportional to the rate of change problem from the last time step to the current time step.
* `Integral (D)`: Mathematically, the `I` component establishes a linear relationship between the average value of the problem over time. The effect of the `I` part is proportional to the average amount of the problem between the beginning and current time step.

### Running the project
If you want to run this project on your machine, you should consider some essential dependencies that I have listed below:
* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    sudo apt-get update
    sudo apt-get install git libuv1-dev libssl-dev gcc g++ cmake make
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    mkdir build
    cd build
    cmake ..
    make 
    sudo make install
    cd ../..
    sudo ln -s /usr/lib64/libuWS.so /usr/lib/libuWS.so
    sudo rm -r uWebSockets
    ```
* Simulatore
    For testing this project you need to download a [simulator](https://github.com/udacity/self-driving-car-sim/releases) that is made by the [Udacity](https://www.udacity.com/).

Once all dependencies are ready, follow below structures:
* Clone this repository: `git@github.com:PooyaAlamirpour/PIDControl.git`
* Make a build directory: `mkdir build && cd build`
* Compile: `cmake .. && make`
* Run it: `./pid`