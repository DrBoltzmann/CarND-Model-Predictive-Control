[image001]: vehicle_kinematics_model.png "vehicle_kinematics_model"


# CarND-Controls-MPC
---
## Model Predictive Controller (MPC)
---

### Overview
This project is focused on the implementation of a MPC to control a self-driving car in a simulation environment. Essentially the MPC takes the 2D state of the vehicle and predicts the future position. The project code implements a vehicle kinematics model and control of the vehicle based on a generated trajectory which attempts to match to an input trajectory. The MPC is designed to follow the input trajectory according to the vehicle kinematics model, and actuation commands are then sent to the vehicle (steering, acceleration and braking). The difference between the two trajectories is minimized via a cost function.

The project rubic can be accessed here:
https://review.udacity.com/#!/rubrics/896/view

### State and Kinematics Model
The state is defined by position (x, y), orientation angle (psi) and velocity (vx, vy). This is defined for a time, t. The state is essentially an input to the kinematic model which predicts the future state at t+1.

The model utilizes a cross track error (cte) value, which is the distance from the input waypoint trajectory to the state. The orientation error (epsi) similarly characterizes the mismatch from the trajectory orientation and the vehicle state.

The future state of the vehicle is determined by the kinematics model:

![alt text][image001]

### Polynomial Fitting and MPC Preprocessing
In the preprocessing stage, the waypoint data is transformed from the global map reference to the local vehicle coordinate system in main.cpp line 57-65. Then, a 3rd degree polynomial line can be fit to the waypoint dataset (main.cpp line 78). The equation coefficients of the Polynomial then define the cte and epsi values in computing the trajectory.

### Actuation Parameters
The actuation of the vehicle is performed with acceleration and delta values, which are defined as follows:

* The acceleration is in the range of -1 to 1, which translates as a full actuation of the throttle or full braking.
* The steering angle is defined in the range of -25° to 25°.

### Time Prediction Window
The separation between t and t + 1 steps is defined by a time-step internal distance (N) and the duration (dt) in seconds. These parameters can be tuned to predict far into the future along the input trajectory, however this comes at the cost of increased computation time. The prediction time into the future is then simply N * dt. The N and dt values were evaluated to obtain ideal driving behavior.

### Cost Function Constraints
Alongside the time-step and duration, the constraints placed on the cost function were critical in producing smooth steering on the vehicle. The constraint parameter weights were essential to minimize the orientation error and actuations.

### Latency Integration
While a computer simulation can essentially execute commands near instantaneously, in reality a latency exists between driver decisions and actions (steering, throttle, etc.). This is accounted for with a 100 ms delay integrated into the main.cpp code at line 90 - 98. Here the calculation of state variables are delayed by 0.1 sec (100 ms).

Latency is accounted for in the MPC since the time forecast (time prediction window) is 750 ms (N * dt = 15 * 0.05 = 0.75), and therefore extends beyond 100 ms.

### Weights Optimization
A key method of producing smooth movement was the addition and tuning of weights to the equations governing driving behavior (steering, acceleration, etc.). In the following code blocks from MPC.cpp, the large weight values and tuning of the 0.55 weight for velocity were determined through various simulation runs. If weights were too low, driving behaviour would be erratic as left-right oscillations of steering would quickly become so large that the vehicle would leave the road surface. These weights provided a balance between smooth acceleration and trajectory accuracy for a successful drive around the simulation course.

```
    // Minimize the cte, epsi and reference speed values
    // Weights were tested to optimize driving behaviour
    for (int t = 0; t < N; t++) {
      fg[0] += 2000*CppAD::pow(vars[cte_start + t], 2);
      fg[0] += 2000*CppAD::pow(vars[epsi_start + t], 2);
      fg[0] += 0.55*CppAD::pow(vars[v_start + t] - ref_v, 2);
    }
    
    // Minimize the actuator strengths
    // Weights were tested to optimize driving behaviour
    for (int t = 0; t < N - 1; t++) {
      fg[0] += 300*CppAD::pow(vars[delta_start + t], 2);
      fg[0] += 50*CppAD::pow(vars[a_start + t], 2);
    }
    
    // A high weight coefficient defines how smooth the steering angle is
    // Weight reduced a bit to improve maneuvarability
    for (int t = 0; t < N - 2; t++) {
      fg[0] += 30000*CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      fg[0] += 100*CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }
```

### Simulation Video
A video of the driving simulation is available at:
MPC_Solution_002
https://youtu.be/pPOldf-f5Wk

MPC_Solution_002 improves greatly on MPC_Solution_001 version, which attained a lower maximum speed and had more erratic handling on corners. The initial solution, which had an inverted steering angle in the main.cpp file and incorrect latency definition is available below:
MPC_Solution_001
https://youtu.be/ybz404VX6m4

# Udacity README Content
### This describes the original project description and details for necessary dependencies, packages, and build instructions.
---

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
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Build with Docker-Compose
The docker-compose can run the project into a container
and exposes the port required by the simulator to run.

1. Clone this repo.
2. Build image: `docker-compose build`
3. Run Container: `docker-compose up`
4. On code changes repeat steps 2 and 3.

## Tips

1. The MPC is recommended to be tested on examples to see if implementation behaves as desired. One possible example
is the vehicle offset of a straight line (reference). If the MPC implementation is correct, it tracks the reference line after some timesteps(not too many).
2. The `lake_track_waypoints.csv` file has waypoints of the lake track. This could fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
5. **VM Latency:** Some students have reported differences in behavior using VM's ostensibly a result of latency.  Please let us know if issues arise as a result of a VM environment.

## Editor Settings

We have kept editor configuration files out of this repo to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. We omitted IDE profiles to ensure
students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. Most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio and develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).
