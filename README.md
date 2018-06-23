[//]: # (Image References)

[image1]: ./images/self_driving_stack.PNG "self driving stack"
[image2]: ./images/state_machine.PNG "state machine"

# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Simulator

You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).

### Goals

In this project, your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH 
of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map
list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which 
means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting
other cars at all cost as well as driving inside of the marked road lanes at all times unless going from one lane to 
another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH,
it should take a little over 5 minutes to complete 1 loop. Also, the car should not experience total acceleration over 
10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt

Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value
is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector 
pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, the distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

## Details

1. The car uses a perfect controller and will visit every (x,y) point it receives in the list every .02 seconds. The 
units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going
from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal 
directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the 
planner receives should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. 
(NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably 
be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code 
usually its not very long maybe just 1-3 time steps. During this delay, the simulator will continue using points that it 
was last given, because of this it is a good idea to store the last points you have used so you can have a smooth transition. 
previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the 
simulator controller with the processed points already removed. You would either return a path that extends this previous 
path or makes sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using 
http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single header file is really easy to use.

---

## Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
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

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

## Implementation

Self-driving car software composes of the following components: 

* sensor fusion
* localization
* prediction
* behavior
* trajectory
* motion control

![alt text][image1]

In this project, I am going to complete the prediction, behavior and trajectory modules so that the car drives 
along the track without any incidents (e.g go over the speed limit, driving too jerky or make the car follow a trajectory
which is not possible.)

Before explaining the implementation of prediction, behavior and trajectory modules, let me walk you over what the role
the simulator plays. 

### Localization and Sensor fusion module (Provided)

The simulator acts as sensor fusion and localization modules

For message structure, please refer to ```./sensor_fusion/sensor_fusion.h``` and ```./localization/localization.h```

The simulator provides the following data.

#### Main car's localization Data (No Noise)

* ["x"] The car's x position in map coordinates
* ["y"] The car's y position in map coordinates
* ["s"] The car's s position in frenet coordinates
* ["d"] The car's d position in frenet coordinates
* ["yaw"] The car's yaw angle in the map
* ["speed"] The car's speed in MPH

#### Previous path data given to the Planner

Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

* ["previous_path_x"] The previous list of x points previously given to the simulator
* ["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

* ["end_path_s"] The previous list's last point's frenet s value
* ["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

* ["sensor_fusion"] A 2D vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

### Motion control module (Provided)

The simulator also provides the function of motion control. Given a trajectory represented by ~50 x-y coordinates.
The simulator drives the car to visit these coordinates every .02 seconds.

### Prediction module

Prediction module consumes the output of sensor fusion module to generate trajectories of detected vehicles.
In this implementation, I assume that all the car is driving along its own track at constant speed.

This assumption is not true, but given that the module predicts in every ~0.1s, it should be able to notify other
components to be careful if another car changes lane in front of the car under controlled.

The implementation is at ```./prediction/prediction.h``` and ```./prediction/prediction.cpp```

### Behavior and trajectory module

Behavior module consumes the output of localization, sensor fusion, and prediction modules to find out the best next
 possible trajectory. 
 
A good trajectory should have the following properties:
1. allow the car to drive as fast as possible
2. avoid any obstacle at all cost.
3. keep distance between vehicles.
4. Able to be realized by the motion control module, in another words, to abide by the acceleration and deceleration limits.
5. jerk should not be larger than the threshold, so that passengers feel comfortable. 

The implementation is at ```./planning/planning.h``` and ```./planning/planning.cpp```

Under normal circumstances, the trajectory is generated from the fourth last position of the last trajectory.
If there is a foreseeable collision if following the last trajectory, a new trajectory is generated from the 
5th position of the last trajectory.

By keeping 2 seconds safety distance, most collisions could be avoided.

#### State machine

Behavior module has a state machine to keep track of the next possible actions the car can take.

Below graph shows the states and transitions used in the implementation.

![alt text][image2]

Given the current state, state machine enumerates all the possible next states by calling 
```vector<string> GetSuccessorStates(int lane_id)```. Trajectories are generated for
those states and evaluated to find out the next best state.

#### Determine the kinematics and trajectory generation

Given the current position, velocity and acceleration of the car, function ```GetKinematics()``` proposed 
the position, velocity and the acceleration of the car after 1s (```planning_time_length```). 

A trajectory is generated by function ```__planning__generate_path_point``` given by current s, v, a and the target s, v, a.

#### Cost function

Here are the cost functions for evaluating trajectories:

| Cost function | Weights  | Description  |
|:-----| :-----|:-----|
| Acceleration limit    | 10  | 0 if within the acceleration limit, 1 otherwise. |
| Speed limit           | 10  | 0 if within the speed limit, 1 otherwise. |
| Off track cost        | 10  | small cost when close to lane center. |
| Distance to obstacle  | 15  | small cost when the distance to the nearest obstacle is large. |
| Lane inefficient      | 30  | small cost when the car at lane is running at high speed. |
| Keep Lane             | 10  | 0 if keep lane, 1 otherwise. |
| Change to free lane   | 5   | 0 if changing to a clear lane, 1 otherwise. |

## Result

The car is able to drive 4.2 miles without any incidents. Here is the [link](https://youtu.be/kmDdvQ9Wi_c) 

## Conclusion

In this project, I implemented the prediction, behavior, and trajectory modules such that the car drive around
the track smoothly.

There are the enhancements I can think of if more time is allowed.
1. Tune costs to make the car drive faster.
2. Use JMT instead of hard-coded 1.3s for lane changing.
3. Create a Naive Bayes classifier to predict if the car is going to change lane. Current implementation assumes all cars
keep lane and drive at constant speed. 
