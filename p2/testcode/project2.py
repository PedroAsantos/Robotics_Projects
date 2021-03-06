import socket, sys, numpy, math
from robotGPS import *
from robot import Map
from croblink import CRobLink
from croblink import CRobLinkAngs

#######################   IMPORTANT: README   ##################################
# This file uses the robotGPS library which only works with GPS enabled.
# For the setup, go to the simulator window -> Options -> GPS enabled
# ALso, go to 'edit configuration' (tool symbol) -> Noise ->
# Set GPS and Compass noise to zero
#
# The robot can also detect walls in an aray: [north, east, south, west]
# 1: Wall detected, 0: no wall detected; -1: not sure
# sometimes if there was no proper wall detection the robot can visit the cell
# again for a new and hopefully better detection
#
# If the robot still bumps into a wall it will return to its original position
###############################################################################

interface = CRobLinkAngs('speedy', 0, [0.0,90.0,180.0,-90.0], 'localhost')
systemModel = 0
if interface.status!=0:
    print "Connection refused or error"
    quit()
robot = Robot(interface, systemModel)
controller = Controller(robot)
map = Map()
iteration = 0

while 1:
    iteration += 1
    if robot.isCentered:
        direction = raw_input("Enter direction (up, down, left, right):")
    #    map.putWalls(robot.currentNode,robot.walls)
        controller.move(direction)

    controller.setControlValue()
    #Debug:
    print ('Iter: {0:4d}; state: {1:4.1f} {8:4.1f}, {6:5.2f}; center: {2:d}; node: {3:}; target: {4:}, dir: {5:}; walls: {7:}'
    .format(iteration, robot.state[0] , robot.isCentered, robot.currentNode, robot.targetNode, robot.orientation, robot.state[2], robot.walls, robot.state[1] ))
