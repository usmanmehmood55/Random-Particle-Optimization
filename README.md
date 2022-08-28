# Random Particle Optimization

Based on the paper [Optimal Path Planning of a Mobile Robot using Quadrant Based Random Particle Optimization Method](https://www.researchgate.net/publication/325170465_Optimal_Path_Planning_of_a_Mobile_Robot_using_Quadrant_Based_Random_Particle_Optimization_Method)

![Graph](https://github.com/usmanmehmood55/Random-Particle-Optimization/blob/master/graph.PNG)

Was originally ported to Arduino but I'm currently testing it on MATLAB, so the Arduino port is outdated.

The code shows cartesian coordinates for the robot to move while avoiding a single obstacle anywhere in a static environment. This can of course easily be scaled to navigate in a dynamic environment with multiple obstacles. 
This code DOES NOT operate a robot right away, it simply takes environment properties as inputs and gives coordinates as output. 

- [RPO cpp](https://github.com/usmanmehmood55/Random-Particle-Optimization/tree/master/RPO%20cpp) folder contains the C++ implementation
- [RPO MATLAB](https://github.com/usmanmehmood55/Random-Particle-Optimization/tree/master/RPO%20MATLAB) contains a matlab script and functions, currently being worked upon

## What works
- Trajectory, obstacle, and goal plotting
- The algorithm itself, but in very limited situations

## What doesn't work
- The algorithm itself, in most situations
- Wildly different response when the number of artificial points or step-size is changed

## To Do
- Recheck the math behind gaussian potential calculations, since the paper uses a slightly modified version of a 2D gaussian
- Plot APs on each iteration along with their potentials, to recheck the AP selection criteria
- Implement changes in Arduino and cpp, once the algo works