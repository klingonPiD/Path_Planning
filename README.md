# Term3 - Project 1: Path Planning Project
### Ajay Paidi

# Objective
The objective of this project is to design and implement a path planner that creates smooth, safe paths for a virtual car to to follow along a 3 lane highway track in a simulator.

# File structure
- **ReadMe.md**: This file
- **main.cpp**: The main executable program that implements and calls the path planner modules to compute the future path coordinates and communicates with the virtual car in the simulator using uwebsockets.
- **BehaviorPlanner.h** and **BehaviorPlanner.cpp**:
- **TrajectoryPlanner.h** and **TrajectoryPlanner.cpp**:
- **Utility.cpp** and **Utility.h**

# Description

The following are the basic modules required to implement a simple path planner
1. Prediction module.
2. Behavior planning module.
3. Trajectory generation module.

### Prediction module
Since we are dealing with highway driving where all the cars are traveling at or close to the speed limit, a simple linear predictor does a decent job of predicting the future car positions. The linear predictor extrapolates the position of the car based on its current velocity.
Future car pos = car velocity * time increment * number of time samples.    
As can be seen, the prediction module is pretty simple and just bundled into the behavior planning module.

### Behavior planning module
Since the simulator mimics a highly structured highway driving environment with well defined rules, I used a simple pseudo finite state model to suggest suitable states the ego car can take. The 4 states used in my model are
1. KL - Keep Lane.
2. KLS - Keep Lane with speed same as ahead vehicles.
3. LCL - Lane Change Left.
4. LCR - Lane Change Right.

The pseudo algorithm for the implemented behavior planner is as follows
1. Filter the cars in the sensor fusion list by looking for cars that are within 20m from the ego car. A simple linear predictor (described above in the prediction module) is used to determine the future positions of the cars before filtering them based on their distance from the ego car.
2. Group the filtered cars according to lanes (only 3 possible lanes - 0,1,2). In the ego lane, consider only the cars head of the ego car. This is because the cars directly behind the ego car are inconsequential to decision making. This is not necessarily true for adjacent cars in the non-ego lanes.
3. If cars exist in the ego lane, then
  a. If no cars exist in the left lane, then set state to LCL.
  b. If no cars exist in the right lane, then set state to LCR.
  c. If LCL / LCR is not possible, then KLS.
4. If cars do not exist in the ego lane, then set state to KL.


### Trajectory generation module
The idea here is that given a start and end position, one needs to find a smooth (jerk minimization) trajectory between the two points while taking into account adjacent vehicles. In our case, the environment is structured (highway driving) and the dimension space is 3D (s, d and t). Inorder to generate a smooth trajectory, I made use of splines. An anchor spline was generated using 2 previous points, 1 current point and 2 future points.


# Results

Here is a video of the path planner in action

[![MPC](https://img.youtube.com/vi/PwYwwBCqS5c/0.jpg)](https://youtu.be/PwYwwBCqS5c)

# References

Starter code provided by Udacity and videos/lecture notes.
