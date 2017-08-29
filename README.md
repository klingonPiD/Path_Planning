# Term3 - Project 1: Path Planning Project
### Ajay Paidi

# Objective
The objective of this project is to design and implement a path planner that creates smooth, safe paths for a virtual car to to follow along a 3 lane highway track in a simulator.

# File structure
- **ReadMe.md**: This file
- **main.cpp**: The main executable program that implements and calls the path planner modules to compute the future path coordinates and communicates with the virtual car in the simulator using uwebsockets.
- **BehaviorPlanner.h** and **BehaviorPlanner.cpp**: Implements behavior planning module
- **TrajectoryPlanner.h** and **TrajectoryPlanner.cpp**: Implements the trajectory planning module
- **Utility.cpp** and **Utility.h**: Implements all the necessary helper functions

# Description

The following are the basic modules implemented in this simple path planner
1. Prediction module.
2. Behavior planning module.
3. Trajectory generation module.

### Prediction module
Since we are dealing with highway driving where all the cars are traveling at or close to the speed limit, a simple linear predictor does a decent job of predicting the future car positions. The linear predictor extrapolates the position of the car based on its current velocity.

Future car pos = car velocity * time increment * number of time samples.

The time increment is 0.02 seconds for the simulator.    
As can be seen, the prediction module is pretty simple and its implementation is just bundled into the behavior planning module.

### Behavior planning module
Since the simulator mimics a highly structured highway driving environment with well defined rules, I used a simple pseudo finite state model to suggest suitable states the ego car can take. The 4 states used in my model are
1. KL - Keep Lane.
2. KLS - Keep Lane with speed same as ahead vehicles.
3. LCL - Lane Change Left.
4. LCR - Lane Change Right.

The pseudo algorithm for the implemented behavior planner is as follows
1. Filter the cars in the sensor fusion list by looking for cars that are within 20m from the ego car. A simple linear predictor (described above in the prediction module) is used to determine the future positions of the cars before filtering them based on their distance from the ego car.
2. Group the filtered cars according to lanes (only 3 possible lanes - 0,1,2). In the ego lane, consider only the cars ahead of the ego car. This is because the cars directly behind the ego car are inconsequential to decision making. This is not necessarily true for adjacent cars in the non-ego lanes.
3. If cars exist in the ego lane, then
  - If no cars exist in the left lane, then set state to LCL.
  - If no cars exist in the right lane, then set state to LCR.
  - If LCL / LCR is not possible, then KLS.
4. If cars do not exist in the ego lane, then set state to KL.

As can be inferred from above, the path planner adopts the approach of defensive driving. There is no need for an explicit collision detector. Essentially the ego car transitions into a new lane if and only if there is no other car in the +-20m vicinity of the new lane. In the ego lane, the ego car either drives at the speed limit or simply follows the car head (at the speed of the ahead car).

### Trajectory generation module
Once the transition state has been determined, one needs to compute a smooth (jerk minimization) trajectory that helps the ego car to realize this new state.  There are multiple ways to compute smooth trajectories. It has been shown that one can obtain minimal jerk trajectories by fitting a quintic polynomial (5th order polynomial with 6 co-efficients) to the desired variables (in our case, frenet s or d along time t). These coefficients can be computed by applying the boundary conditions (start position and end position) and solving a linear system of equations.

Another approach is using splines. Splines fit piecewise cubic polynomials to a given set of points (eg. frenet s, t, or cartesian x and y). One can obtain a minimal jerk trajectory by fitting a spline to a set of reasonably well spaced 5 points (to mimic a quintic polynomial). I followed this approach by making use of this spline library http://kluge.in-chemnitz.de/opensource/spline/.

The implementation of the trajectory generation module consists of the following parts
#### Generate anchor points
5 anchor points (x,y car positions) are chosen and a reference spline is fit to these anchor points. The 5 points must be carefully chosen to encompass the possible start and end positions of the ego car for the about to be computed trajectory. In my case I chose 2 previous points, the current point and 2 potential future points (at 30 m and 60 m respectively from the current car pos). These points first have to be  translated and rotated (by the car yaw angle) to put the car in the center of the new frame of reference. The spline is then fit to these anchor points.

#### Generate path
Once we have our anchor spline, path generation is just an exercise of extracting points along this spline at a sampling rate proportional to the ego car's target velocity. The behavior planner computes the target velocity of the ego car (which is typically either the speed limit or the speed of the ahead car).

Target dist = N * delta_t * target_v

Here N is the number of sample points, delta_t is the time interval (0.02 s for the simulator). Assuming a target distance of 30m, one can easily get the value of N. Getting the new trajectory path is then getting points (x,y) where x = x + x/N and y = spline(x).

In order to maintain continuity of the trajectory the new points are appended to previous points (points computed from the previous run) before sending it to the simulator.


# Results

Here is a video of the implemented path planner in action.

[![Path Planning](https://img.youtube.com/vi/PwYwwBCqS5c/0.jpg)](https://youtu.be/PwYwwBCqS5c)

# Discussion

The implemented path planner is pretty basic and works only in a highly structured environment like a highway with 3 lanes. Potential areas of improvement include
- Using a more advanced predictor to predict positions of adjacent cars (eg. a running average filter, regression models, etc. )
- Handling more complex situations in the behavior planner. For eg. When there is a cohort of cars in the ego lane and adjacent lane, the ego car can execute a manoeuvre where it slows down a bit and executes a lane change to a lane that has less traffic.
- A more advanced trajectory planner where several trajectories and their associated cost are computed and the trajectory with the least cost is selected.

# References

Starter code provided by Udacity and videos/lecture notes.
