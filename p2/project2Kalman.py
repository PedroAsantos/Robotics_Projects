import socket, sys, numpy, math
from robot import *
from croblink import CRobLinkAngs

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
navigation = Navigation()
iteration = 0

controller.setControlValue()
controller.setControlValue()
state = EXPLORINGMAP
while 1:
    if robot.isCentered:
        if state == EXPLORINGMAP:
            navigation.putWalls(robot.currentNode, robot.walls)
            navigation.updateSizeMap(robot.currentNode)

            if robot.measurements.ground==0: #robot.currentNode == [3,-1]:
                print("#####################################CHEESE######################################")
                navigation.saveCheeseCoord(robot.currentNode)
                interface.setVisitingLed(1)
                if navigation.checkIfBestPathIsAvailable():
                    print("RETURNINGTOBASE")
                    state = RETURNINGTOBASE
                else:
                    #map.resetAStar("hard")
                    state = EXPLORINGMAPAFTERCHEESE
            else:
                controller.move(navigation.getMovementDirectionStateExploringMap(robot.currentNode,robot.orientation))

        if state == EXPLORINGMAPAFTERCHEESE:
            navigation.putWalls(robot.currentNode, robot.walls)
            navigation.updateSizeMap(robot.currentNode)

            if navigation.checkIfBestPathIsAvailable():
                state = RETURNINGTOCHEESE
                navigation.resetAStar("hard")
            else:
                controller.move(navigation.getMovementDirectionToFindBestPath(robot.currentNode,robot.orientation))

        if state == RETURNINGTOCHEESE:
            if robot.currentNode == navigation.cheeseCoord:
                state = RETURNINGTOBASE
                interface.setVisitingLed(0)
                interface.setReturningLed(1)
            else:
                controller.move(navigation.getMovementDirectionToGoToCheese(robot.currentNode,robot.orientation))

        if state == RETURNINGTOBASE:
            print("RETURNINGTOBASE")
            if robot.currentNode != [0,0]:
                controller.move(navigation.getMovementDirectionFinal(robot.currentNode,robot.orientation))
            else:
                interface.setReturningLed(0)
                print("END!!!!")
    controller.setControlValue()
    #Debug:
    #iteration += 1
    #print ('Iter: {0:4d}; state: {1:4.1f} {8:4.1f}, {6:5.2f}; center: {2:d}; node: {3:}; target: {4:}, dir: {5:}; walls: {7:}'
    #.format(iteration, robot.state[0] , robot.isCentered, robot.currentNode, robot.targetNode, robot.orientation, robot.state[2], robot.walls, robot.state[1] ))
