import socket, sys, numpy, math
from robot import *
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

EXPLORINGMAP=0
EXPLORINGMAPAFTERCHEESE=1
RETURNINGTOCHEESE=2
RETURNINGTOBASE=3



interface = CRobLinkAngs('speedy', 0, [0.0,90.0,180.0,-90.0], 'localhost')
systemModel = SystemModel()
if interface.status!=0:
    print "Connection refused or error"
    quit()
robot = Robot(interface, systemModel)
controller = Controller(robot)
map = Map()
iteration = 0
#command=0
#dir=['right','right','right','up','left','up','right','right','up','right','right','right','right','right','down','left','right','down','down','right']
controller.setControlValue()
controller.setControlValue()
state = EXPLORINGMAP
while 1:
    if robot.isCentered:
        if state == EXPLORINGMAP:
            map.putWalls(robot.currentNode, robot.walls)
            map.updateSizeMap(robot.currentNode)
            #map.updateTargetNode()
            if robot.currentNode == [10,2]: # robot.measurements.ground==0: #
                print("#####################################CHEESE######################################")
                map.saveCheeseCoord(robot.currentNode)
                interface.setVisitingLed(1)
                if map.checkIfBestPathIsAvailable():
                    print("RETURNINGTOBASE")
                    state = RETURNINGTOBASE
                else:
                    #map.resetAStar("hard")
                    state = EXPLORINGMAPAFTERCHEESE
            else:
                controller.move(map.getMovementDirectionStateExploringMap(robot.currentNode,robot.orientation))

        if state == EXPLORINGMAPAFTERCHEESE:
            map.putWalls(robot.currentNode, robot.walls)
            map.updateSizeMap(robot.currentNode)

            if map.checkIfBestPathIsAvailable():
                state = RETURNINGTOCHEESE
                map.resetAStar("hard")
            else:
                controller.move(map.getMovementDirectionToFindBestPath(robot.currentNode,robot.orientation))

        if state == RETURNINGTOCHEESE:
            if robot.currentNode == map.cheeseCoord:
                state = RETURNINGTOBASE
            else:
                controller.move(map.getMovementDirectionToGoToCheese(robot.currentNode,robot.orientation))

        if state == RETURNINGTOBASE:
            print("RETURNINGTOBASE")
            if robot.currentNode != [0,0]:
                controller.move(map.getMovementDirectionFinal(robot.currentNode,robot.orientation))
            else:
                interface.setReturningLed(1)
                print("END!!!!")
        #command += 1
    controller.setControlValue()
    #Debug:
    iteration += 1
    print ('Iter: {0:4d}; state: {1:4.1f} {8:4.1f}, {6:5.2f}; center: {2:d}; node: {3:}; target: {4:}, dir: {5:}; walls: {7:}'
    .format(iteration, robot.state[0] , robot.isCentered, robot.currentNode, robot.targetNode, robot.orientation, robot.state[2], robot.walls, robot.state[1] ))
