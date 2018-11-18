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
map = Map()
iteration = 0
<<<<<<< HEAD
#command=0
#dir=['right','right','right','up','left','up','right','right','up','right','right','right','right','right','down','left','right','down','down','right']
=======
>>>>>>> 9c49e5ec7e45f8762cf09039eb9a1c0a2ba4c7e0
controller.setControlValue()
controller.setControlValue()
state = EXPLORINGMAP
while 1:
    if robot.isCentered:
        if state == EXPLORINGMAP:
            map.putWalls(robot.currentNode, robot.walls)
            map.updateSizeMap(robot.currentNode)
            #map.updateTargetNode()
            if robot.measurements.ground==0: #robot.currentNode == [3,-1]:
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
<<<<<<< HEAD
        #command += 1
=======
>>>>>>> 9c49e5ec7e45f8762cf09039eb9a1c0a2ba4c7e0
    controller.setControlValue()
    #Debug:
    #iteration += 1
    #print ('Iter: {0:4d}; state: {1:4.1f} {8:4.1f}, {6:5.2f}; center: {2:d}; node: {3:}; target: {4:}, dir: {5:}; walls: {7:}'
    #.format(iteration, robot.state[0] , robot.isCentered, robot.currentNode, robot.targetNode, robot.orientation, robot.state[2], robot.walls, robot.state[1] ))
