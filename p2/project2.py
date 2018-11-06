import socket, sys, numpy, math
from robot import *
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
if interface.status!=0:
    print "Connection refused or error"
    quit()
robot = Robot(interface, systemModel)
controller = Controller(1, robot)

while 1:
    controller.move(direction)
    controller.setControlValue()
