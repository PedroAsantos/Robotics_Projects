# Robotics_Projects
This repository contains the projects developed during the course of robotics. 

## Project 1
The first project consists on the development of the control software of a black/white transition follower robot.
This project contains 3 extra challenges.
  The robot must be prepared to:
  * deal with sharp edges;
  * detect an object and avoid them and them continuing following the black/white transition;
  * find in straight segments, two circles that mark the beginning and end of a controlled speed segment of the track. Between 
these two lines the robot must move at a constant speed (to be defined) and turn on its LED;

Futhermore, after completing the two first laps, the start line will be removed and the robot is supposed to stop as close as 
possible to the original position of the start line.

## Project 2
The main objective is the development of a robotic agent to command a mobile robot,making it move to an unknown target 
position in a semi-structured environment and then return back to the start position through the shortest path. At the end, 
the agent should also show the acquired map and the path used to return.

In order to fulfil the main objective:
* Information from the obstacle sensors was used in order to avoid and follow the walls.
* The target position is detected using the ground sensor, as the value changes when the robot center is inside this region; 
the start position is not marked in any way.
* There are no encoder sensors but the robot pose was estimated using the motor model and the velocity commands sent 
to the simulated robot.
* To deal with the noise in sensors, it was used Kalman filter

####Project 3
The robot is not simulated and it should go through a maze following black lines and searhing for a larger black line that it is 
considered the token. The maze has loops and the robot must explore the map and find the token. After finding the token, the robot
must return to the initial point using the best possible path. 

![alt text](https://raw.githubusercontent.com/PedroAsantos/Robotics_Projects/master/p3/maze.png)
![alt text](https://raw.githubusercontent.com/PedroAsantos/Robotics_Projects/master/p3/robot.png)
