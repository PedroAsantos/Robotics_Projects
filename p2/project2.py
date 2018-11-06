import socket, sys, numpy, math
from robotGPS import *
from croblink import CRobLink
from croblink import CRobLinkAngs
###############IMPORTANT COMMANDS##################################
# initialize robot with name, starting pos, sensor pos and host IP
#   cif = CRobLinkAngs(robname, pos, [0.0,-60.0,60.0,90.0], host)
#access distance Sensors
#   cif.measures.irSensor[0]
#set wheel velocities
#   cif.driveMotors(-0.1,+0.1)
# read robot Sensors
#   cif.readSensors()
#ground beacon detection
#   if cif.measures.ground==0:
#       cif.setVisitingLed(1);
#check if connection exists
#   if cif.status!=0:
#    print "Connection refused or error"
#    quit()
###################################################################


interface = CRobLinkAngs('speedy', 0, [0.0,90.0,180.0,-90.0], 'localhost')
systemModel = 0
if interface.status!=0:
    print "Connection refused or error"
    quit()
robot = Robot(interface, systemModel)
controller = Controller(1, robot)
iteration = 0
while 1:
    iteration += 1
    if robot.isCentered:
        direction = raw_input("Enter direction (up, down, left, right):")
        controller.move(direction)
    controller.setControlValue()
    #Debug:
    print ('Iteration: {0:4d}; state: {1:5.1f}, {8:5.1f}; centered: {2:d}; node: {3:}; target: {4:}, orientation: {5:}; compass: {6:5.1f}; irSensor: {7:}'
    .format(iteration, robot.state[0] , robot.isCentered, robot.currentNode, robot.targetNode, robot.orientation, robot.measurements.compass, robot.measurements.irSensor,robot.state[1] ))
