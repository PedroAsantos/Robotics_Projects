import socket, sys, numpy, math
from robotGPS import *
from croblink import CRobLink
from croblink import CRobLinkAngs

#######################   IMPORTANT: SETUP   ##################################
# This file uses the robotGPS library which only works with GPS enabled.
# For the setup, go to the simulator window -> Options -> GPS enabled
# ALso, go to 'edit configuration' (tool symbol) -> Noise ->
# Set GPS and Compass noise to zero
###############################################################################

interface = CRobLinkAngs('speedy', 0, [0.0,90.0,180.0,-90.0], 'localhost')
systemModel = 0
if interface.status!=0:
    print "Connection refused or error"
    quit()
robot = Robot(interface, systemModel)
mood = 1.25                         # 0.66: normal 1: aggressive; 1.25: max;
controller = Controller(mood, robot)
iteration = 0

while 1:
    iteration += 1
    if robot.isCentered:
        direction = raw_input("Enter direction (up, down, left, right):")
        controller.move(direction)
    controller.setControlValue()
    #Debug:
    print ('Iter: {0:4d}; state: {1:4.1f} {8:4.1f}, {6:5.2f}; center: {2:d}; node: {3:}; target: {4:}, dir: {5:}; irSensor: {7:}'
    .format(iteration, robot.state[0] , robot.isCentered, robot.currentNode, robot.targetNode, robot.orientation, robot.state[2], robot.measurements.irSensor,robot.state[1] ))
