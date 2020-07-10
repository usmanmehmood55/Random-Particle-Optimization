# Random-Particle-Optimization
Random Particle Optimization (RPO) code ported to Arduino.
The code shows cartesian coordinates for the robot to move while avoiding a single obstacle anywhere in a static environment. This can of course easily be scaled to navigate in a dynamic environment with multiple obstacles. 
This code DOES NOT operate a robot right away, it simply takes environment properties as inputs and gives coordinates as output. 

.ino files work togeather in Arduino IDE. main.cpp is a standalone c++ file, and RPO.m is a standalone matlab script that also outputs a graph of trajectory. 
