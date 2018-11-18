import socket, sys, numpy, math
from croblink import CRobLinkAngs
from copy import copy
import operator
import pdb
from random import randint
from copy import deepcopy

class Robot():
    def __init__(self, interface, systemModel):
        self.interface = interface
        self.state = [0, 0, 0]              #[x y theta]
        self.walls = [0, 0, 0, 0]           #[north east south west] -1: unknown
        self.isCentered = True              #robot reached target node
        self.currentNode = [0, 0]           #[n_x, n_y]
        self.targetNode = [0, 0]            #[n_x, n_y]
        self.Kalman = Kalman(copy(self.state), systemModel)
        self.map = Map()
        self.orientation = ''               #general robot direction (e.g. east)
        self.irStd = []
        self.getMeasurements()
    def motorAction(self, u):               #apply control input u
        if (not self.measurements.start) or (self.measurements.stop):
            u = [0, 0]
            print('Simulator stop condition: Disabling drives')
        self.Kalman.setU(u)
        self.interface.driveMotors(u[1],u[0])
    def getMeasurements(self):              #get measurements from interface
        self.interface.readSensors()
        self.measurements = copy(self.interface.measures)
        self.walls = [-1, -1, -1, -1]       #walls initially unknown
        compass = copy(self.measurements.compass)
        compass = compass*math.pi/180
        if compass < 0: compass += 2*math.pi    #only positive angles
        #map angle to continuous measurement, not just one rotation:
        while self.state[2] - compass > math.pi:
            compass += 2*math.pi
        while self.state[2] - compass < -math.pi:
            compass -= 2*math.pi
        self.measurements.compass = compass
        irSensor = copy(self.measurements.irSensor)
        self.irStd = [abs(1/(x-0.1) - 1/x) if x>0.1 else 1 for x in irSensor]
        irSensor = [1/x if x > 0.0 else 100.0 for x in irSensor] #distance is inverse of measurement
        # map local orientation to global orientation:
        if self.orientation == 'north':
            irSensor = [irSensor[0]] + [irSensor[3]] + [irSensor[2]] + [irSensor[1]]
        if self.orientation == 'east':
            irSensor = [irSensor[1]] + [irSensor[0]] + [irSensor[3]] + [irSensor[2]]
        if self.orientation == 'south':
            irSensor = [irSensor[2]] + [irSensor[1]] + [irSensor[0]] + [irSensor[3]]
        if self.orientation == 'west':
            irSensor = [irSensor[3]] + [irSensor[2]] + [irSensor[1]] + [irSensor[0]]
        self.walls, self.measurements.irSensor = self.checkForWalls(irSensor)
        return copy(self.measurements)
    def checkForWalls(self, irSensor): # checks for walls and calculates position
        angle = math.fmod(self.state[2], 2*math.pi)
        # angle to closest general direction (north, east etc.):
        nextAngle = abs(angle - (math.pi/2) * round(float(angle)/(math.pi/2)))
        # only check for walls when robot is aligned
        threshold = -1.5*nextAngle + 0.9 # first: check for walls
        walls = [1 if x <= threshold else -1 for x in irSensor]
        threshold = +0.9*nextAngle + 0.9 # second: check for no walls
        walls = [0 if irSensor[x] >  threshold else walls[x] for x in range(4)]
        # else: unknown (-1)
        # calculate distance to closest cell:
        dX1 = self.targetNode[0]*2 - self.state[0]
        dY1 = self.targetNode[1]*2 - self.state[1]
        delta1 = math.sqrt(dX1**2 + dY1**2)
        dX2 = self.currentNode[0]*2 - self.state[0]
        dY2 = self.currentNode[1]*2 - self.state[1]
        delta2 = math.sqrt(dX2**2 + dY2**2)
        delta = min(delta1, delta2)
        if delta >= 0.4: walls = [-1, -1, -1, -1]
        # map sensor distance readings to global x,y positions:
        nearestX = 2 * round(float(self.state[0])/2)
        nearestY = 2 * round(float(self.state[1])/2)
        irSensor[0] = -irSensor[0] + nearestY + 0.40
        irSensor[1] = -irSensor[1] + nearestX + 0.40
        irSensor[2] =  irSensor[2] + nearestY - 0.40
        irSensor[3] =  irSensor[3] + nearestX - 0.40
        irSensor = [irSensor[x] if walls[x] ==1 else [] for x in range(4)]
        # empty entry: no valid measurement
        return (walls, irSensor)
    def getState(self):
        self.state = self.Kalman.kalmanStep(self.getMeasurements(), self.irStd)
        theta = self.state[2]
        while theta < 0:        # only positive angles
            theta += 2*math.pi
        # calculate general robot direction:
        theta = math.fmod(theta, 2*math.pi)
        pizzaSection = int(theta//(math.pi/4))
        if pizzaSection == 0 or pizzaSection == 7:
            self.orientation = 'east'
        if pizzaSection == 1 or pizzaSection == 2:
            self.orientation = 'north'
        if pizzaSection == 3 or pizzaSection == 4:
            self.orientation = 'west'
        if pizzaSection == 5 or pizzaSection == 6:
            self.orientation = 'south'
        return copy(self.state)

# Functions for the continuous world
class Controller():
    def __init__(self, robot):
        self.robot = robot
    def move(self,direction):           #direction: up, down, left, right
        if self.robot.isCentered:
            if direction == 'up':
                self.robot.targetNode[1] += 1
                self.robot.isCentered = False
            if direction == 'down':
                self.robot.targetNode[1] -= 1
                self.robot.isCentered = False
            if direction == 'left':
                self.robot.targetNode[0] -= 1
                self.robot.isCentered = False
            if direction == 'right':
                self.robot.targetNode[0] += 1
                self.robot.isCentered = False
    def getThetaRef(self, dX, dY): # calculate reference angle for control loop
        state = copy(self.robot.state)
        thetaRef = math.atan2(dY, dX)
        if thetaRef < 0: thetaRef += 2*math.pi
        while state[2] - thetaRef > math.pi:
            thetaRef += 2*math.pi
        while state[2] - thetaRef < -math.pi:
            thetaRef -= 2*math.pi
        return thetaRef
    def setControlValue(self): # sets control value calculated by P-controller
        if self.robot.measurements.collision == True:
            self.robot.targetNode = copy(self.robot.currentNode)
        self.currentState = self.robot.getState()
        dX = self.robot.targetNode[0]*2 - self.currentState[0]
        dY = self.robot.targetNode[1]*2 - self.currentState[1]
        thetaRef = self.getThetaRef(dX, dY)
        delta = math.sqrt(dX**2 + dY**2)        #distance to target node
        u = self.calcControlValue(thetaRef, delta)
        self.robot.motorAction(u)
    def calcControlValue(self, thetaRef, delta):    # P-controller
        errorDelta = -delta                         # distance error
        P_theta = 0.15                    # proportional action
        P_delta = 0.30                    # proportional action
        theta = self.robot.state[2]
        errorTheta = theta - thetaRef               # angular error
        ab = abs(errorTheta)*180/math.pi
        if ab > 5 and ab <= 9:
            P_delta = P_delta * (-0.25*ab + 9/4)
        if ab > 9:              # don't go forward if angular error is too high
            P_delta = 0
        if abs(errorDelta) < 0.20:       # node tolerance
            P_delta = 0                 # stop moving
            modulo = math.fmod(theta, 2*math.pi)
            while modulo < 0: modulo += 2*math.pi
            # align robot to closest general direction:
            if self.robot.orientation == 'north':
                errorTheta = modulo - math.pi/2
            if self.robot.orientation == 'east':
                errorTheta = modulo
                if errorTheta > math.pi: errorTheta -= 2*math.pi
            if self.robot.orientation == 'south':
                errorTheta = modulo - 3*math.pi/2
            if self.robot.orientation == 'west':
                errorTheta = modulo - math.pi
            # when distance and angular error small: target reached
            if abs(errorTheta) < 4*math.pi/180:
                errorTheta = 0
                self.robot.currentNode = copy(self.robot.targetNode)
                self.robot.isCentered = True
        # motor control values calculated by two P-Loops:
        unlimitedCtrl = errorDelta*P_delta*(-1)
        limitedCtrl = unlimitedCtrl if abs(unlimitedCtrl) < 0.12 else numpy.sign(unlimitedCtrl)*0.12
        rightWheel = limitedCtrl + errorTheta*P_theta*(-1)
        leftWheel  = limitedCtrl - errorTheta*P_theta*(-1)

        rightWheel = rightWheel if abs(rightWheel) < 0.15 else numpy.sign(rightWheel)*0.15
        leftWheel = leftWheel if abs(leftWheel) < 0.15 else numpy.sign(leftWheel)*0.15
        u = [rightWheel, leftWheel]
        return u
class SystemModel():
    def __init__(self):
        self.A = numpy.array([[1.0,0,0,0,0],[0,1,0,0,0],[0,0,1,0.5,-0.5],[0,0,0,0.5,0],[0,0,0,0,0.5]])
        self.B = numpy.array([[0.0,0],[0,0],[0.5,-0.5],[0.5,0],[0,0.5]])
        self.R = numpy.array([[0.0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0]])
        self.C = numpy.array([[0.0,1,0,0,0],[1,0,0,0,0],[0,1,0,0,0],[1,0,0,0,0],[0,0,1,0,0]])
class Kalman():
    def __init__(self, initialBelief, systemModel):
        self.systemModel = systemModel
        self.belief = numpy.append(numpy.asarray(initialBelief), numpy.array([0, 0]))
        self.belief = self.belief.reshape(5,1)
        self.Sigma =  numpy.array([[0.0,0,0,0,0],[0,0,0,0,0],[0,0,20.0,0,0],[0,0,0,0,0],[0,0,0,0,0]])
        self.u = numpy.array([[0.0],[0.0]])
        self.updateSystem()
    def updateSystem(self):
        s = math.sin(self.belief[2,0])/4
        c = math.cos(self.belief[2,0])/4
        m = numpy.array([[c, c],[s, s]])
        self.systemModel.A[0:2, 3:5] = m
        self.systemModel.B[0:2, 0:2] = m
        self.systemModel.R[3,3] = 0.015*abs(0.5*self.u[0,0] + 0.5*self.belief[3,0])
        self.systemModel.R[4,4] = 0.015*abs(0.5*self.u[1,0] + 0.5*self.belief[4,0])

    def kalmanStep(self, measurements, irStd):
        z, tempC, Q= self.calcz(measurements, irStd)
        belPre =        numpy.add(
                            numpy.matmul(self.systemModel.A, self.belief),
                            numpy.matmul(self.systemModel.B, self.u)
                            )
        actionNoise =   numpy.matmul(
                            numpy.matmul(   self.systemModel.A, self.Sigma),
                            numpy.transpose(self.systemModel.A)
                                    )
        sigmaPre = numpy.add( actionNoise,  self.systemModel.R )

        A = numpy.matmul(   sigmaPre, numpy.transpose(tempC) )
        B = numpy.add(      numpy.matmul(tempC, A), Q  )
        KalmanGain = numpy.matmul( A, numpy.linalg.inv(B) )

        C = numpy.subtract(
                            z, numpy.matmul( tempC, belPre )
                            )
        belief = numpy.add(
                            belPre, numpy.matmul(  KalmanGain, C )
                            )

        D = numpy.matmul(KalmanGain, tempC)
        Sigma = numpy.matmul(
                            numpy.subtract( numpy.eye(5), D ), sigmaPre
                            )
        self.Sigma = Sigma
        self.belief = belief
        self.updateSystem()
        #if node == [3,0] and orientation == 'north' and numpy.shape(tempC)[0]==2:
        #    pdb.set_trace()
        belief = belief.reshape(1,5)[0]
        belief = belief[0:3].tolist()
        return belief
    def setU(self, u):
        u = numpy.asarray(u)
        u = u.reshape(2,1)
        self.u = u
    def calcz(self, measurements, irStd):
        compass = numpy.asarray(measurements.compass)
        irSensor = numpy.asarray([x for x in measurements.irSensor if x!=[]])
        validSenses = [1 if x!=[] else 0 for x in measurements.irSensor]
        z = numpy.append([irSensor], [compass])
        z = z.reshape(numpy.shape(z)[0],1)
        tempC = self.systemModel.C
        currentRow = 0
        for x in validSenses:
            if x == 0:
                tempC = numpy.delete(tempC, currentRow, 0)
            else:
                currentRow += 1
        std = [irStd[x] + 0.04 for x in range(4) if validSenses[x]==1]
        std.append(2*math.pi/180)
        Q = numpy.diag(numpy.asarray(std))

        return (z, tempC, Q)

###### functions for the discrete world ########

class Map():
    def __init__(self):
        self.nodes=0
        self.map = []
        self.contDeletedX=[0,0] #cont to know how many lines from left/left were deleted
        self.contDeletedY=[0,0] #cont to know how many lines from up/down were deleted
        self.basePoint = (6,13)
        for y in range(6,-7,-1):
            xList= []
            for x in range(-13,14):
                node = Node((x,y))
                xList.append(node)
            self.map.append(xList)
        self.targetIndex = (len(self.map)-1,len(self.map[0])-1)
        self.lastMinimunDist = 1000
        self.performingAStar=False
        self.NodesOfAStart = []
        self.closedSet=[]
        self.inClosestNode=False
        self.indexNodeOfAStar = -1
        self.historyPathNode=[]
        self.cheeseCoord=None
        self.subStateExploring=0
    def putWalls(self,currentNodeCoord,walls):

        elementLocation = self.getElementLocation(currentNodeCoord)
        #add walls to the current position of the robot
        self.map[elementLocation[0]][elementLocation[1]].walls = walls

        #add walls to the nodes near this walls
        if walls[0]==1:
            tempNodeAboveLocation = self.getElementLocation(tuple(map(operator.add, (0,1), currentNodeCoord)))
            if tempNodeAboveLocation[0] < len(self.map[0]) & tempNodeAboveLocation[1] < len(self.map):
                self.map[tempNodeAboveLocation[0]][tempNodeAboveLocation[1]].walls[2]=1
        if walls[1]==1:
            tempNodeEastLocation = self.getElementLocation(tuple(tuple(map(operator.add, (1,0), currentNodeCoord))))
            if tempNodeEastLocation[0] < len(self.map[0]) & tempNodeEastLocation[1] < len(self.map):
                self.map[tempNodeEastLocation[0]][tempNodeEastLocation[1]].walls[3]=1
        if walls[2]==1:
            tempNodeBelowLocation = self.getElementLocation(tuple(tuple(map(operator.add, (0,-1), currentNodeCoord))))
            if tempNodeBelowLocation[0] < len(self.map[0]) & tempNodeBelowLocation[1] < len(self.map):
                self.map[tempNodeBelowLocation[0]][tempNodeBelowLocation[1]].walls[0]=1
                #self.map[tempNodeBelowLocation[0]][tempNodeBelowLocation[1]]=node
        if walls[3]==1:
            tempNodeWestLocation = self.getElementLocation(tuple(tuple(map(operator.add, (-1,0), currentNodeCoord))))
            if tempNodeWestLocation[0] < len(self.map[0]) & tempNodeWestLocation[1] < len(self.map):
                self.map[tempNodeWestLocation[0]][tempNodeWestLocation[1]].walls[1]=1


    def getElementLocation(self, currentNodeCoord):
        currentNodeCoord=currentNodeCoord[::-1]
        currentNodeCoord= tuple(map(operator.mul, (-1,1), currentNodeCoord))
        return tuple(map(operator.add, self.basePoint, currentNodeCoord))

    def getNeighbors(self,currentNodeCoord, walls):
        neightborNodesCoord=[]
        neighbors=[(0,1,walls[0],'north','up'),(1,0,walls[1],'east','right'),(0,-1,walls[2],'south','down'),(-1,0,walls[3],'west','left')]
        for n in neighbors:
            if n[2]!=1:
                neighborCoord = tuple(tuple(map(operator.add, (n[0],n[1]), currentNodeCoord)))
                neightborNodesCoord.append((neighborCoord[0],neighborCoord[1],n[3],n[4]))
        return neightborNodesCoord

    def getKnownNeighbors(self,neightborNodesCoord):
        knownNeighbors=[]
        for n in neightborNodesCoord:
            nLocation =self.getElementLocation((n[0],n[1]))
            if self.map[nLocation[0]][nLocation[1]].walls.count(-1)==0:
                knownNeighbors.append(n)
        return knownNeighbors

    def getUnknownNeighbors(self,neightborNodesCoord):
        unknownNeighbors=[]
        for n in neightborNodesCoord:
            nLocation =self.getElementLocation((n[0],n[1]))
            if self.map[nLocation[0]][nLocation[1]].walls.count(-1)>1:
                unknownNeighbors.append(n)

        return unknownNeighbors
    def getPath(self,startingNode,targetNode):
        path=[]
        currentNode=targetNode

        while currentNode.pos!=startingNode.pos:
            path.append(currentNode)
            print(currentNode.pos)
            currentNode = currentNode.parent
        print("#getpath()")
        print("path: "+str(path))
        return path

    def checkIfNodeIsInList(self,list,nodeToCheck,currentNode):
        for node in list:
            if not (node.parent is None):
                if node.pos == nodeToCheck.pos and node.parent.pos==currentNode.pos:
                    return True
        return False
    def removeNodeFromList(self,list,nodeToDelete):
        for node in list:
            if not (node.parent is None):
                if node.pos == nodeToDelete.pos and node.parent==nodeToDelete.parent:
                    nodeToDelete = node
                    break
        list.remove(nodeToDelete)
        return list
    def performAStar(self,startingNodeCoord,targetNodeCoord,saveClosedSet=False):
        print("PerforminAStart")
        print("startingNodeCoord: "+str(startingNodeCoord)+"targetNodeCoord: "+str(targetNodeCoord))
        #ensure that AStar is reset
        self.resetAStar("soft")
        openSet = []
        closedSet = []
        currentElementLocation = self.getElementLocation(startingNodeCoord)
        targetElementLocation = self.getElementLocation(targetNodeCoord)
        startingNode = self.map[currentElementLocation[0]][currentElementLocation[1]]

        openSet.append(deepcopy(startingNode))
        targetNode = self.map[targetElementLocation[0]][targetElementLocation[1]]

        while len(openSet)>0:
            currentNode = openSet[0]

            for n in openSet:
                if n.getFcost() < currentNode.getFcost() or (n.getFcost() == currentNode.getFcost() and n.hCost < currentNode.hCost):
                    currentNode = n

            openSet = self.removeNodeFromList(openSet,currentNode)
            #openSet.remove(currentNode)
            closedSet.append(currentNode)

            if currentNode.pos==targetNode.pos:
                if saveClosedSet:
                    self.closedSet = deepcopy(closedSet)
                return self.getPath(startingNode,currentNode)
            #print(openSet)
            #print(currentNode)
            #if currentNode.pos == (6,2) and openSet[0].pos==(3,2):
            #    pdb.set_trace()
            for neightborCoor in self.getKnownNeighbors(self.getNeighbors(currentNode.pos,currentNode.walls)):
                neighIndex = self.getElementLocation((neightborCoor[0],neightborCoor[1]))
                neighborNode = deepcopy(self.map[neighIndex[0]][neighIndex[1]])

                if not self.checkIfNodeIsInList(closedSet,neighborNode,currentNode):
                #if not (neighborNode in closedSet):
                    newMovementCostToNeigh = currentNode.gCost + 1
                    if newMovementCostToNeigh < neighborNode.gCost or not self.checkIfNodeIsInList(openSet,neighborNode,currentNode):
                    #if newMovementCostToNeigh < neighborNode.gCost or not ( neighborNode in openSet):
                        neighborNode.gCost= newMovementCostToNeigh
                        neighborNode.hCost = self.manhattanDistanceTwoPoints(neighborNode.pos,targetNodeCoord)
                        neighborNode.parent=currentNode
                        #neighborNode.unknownNeigh = self.getUnknownNeighbors(self.getNeighbors(neighborNode.pos,neighborNode.walls))
                        if not self.checkIfNodeIsInList(openSet,neighborNode,currentNode):
                            openSet.append(neighborNode)


    def resetAStar(self,mode):
        if mode=="hard":
            print("hard reset to AStar")
            self.indexNodeOfAStar=-1
            self.NodesOfAStart=[]
            self.closedSet=[]
            self.performingAStar=False
            for xList in self.map:
                for node in xList:
                    node.reset()
        else:
            for xList in self.map:
                for node in xList:
                    node.reset()

    def getDirectionOfAStar(self,currentNodeCoord):
        print(self.NodesOfAStart)
        print(self.indexNodeOfAStar)
        print(len(self.NodesOfAStart))
        nodeToGo = self.NodesOfAStart[self.indexNodeOfAStar]
        print(self.NodesOfAStart[self.indexNodeOfAStar])
        if self.indexNodeOfAStar==-len(self.NodesOfAStart):
            self.resetAStar("hard")
        else:
            self.indexNodeOfAStar-=1

        if nodeToGo.pos[0]==currentNodeCoord[0]:
            if nodeToGo.pos[1]<currentNodeCoord[1]:
                return "down"
            else:
                return "up"
        else:
            if nodeToGo.pos[0]<currentNodeCoord[0]:
                return "left"
            else:
                return "right"

    def calculateTargetNodeToAStar(self):
         for node in reversed(self.historyPathNode):
             neighbors = self.getNeighbors(node.pos,node.walls)
             for neight in neighbors:
                 nodeLocation = self.getElementLocation((neight[0],neight[1]))
                 if self.map[nodeLocation[0]][nodeLocation[1]].walls.count(-1)>0:
                     return (node.pos)
        #if somothing fails
        #return self.historyPathNode[0].pos
    def getDirectionOfNodeToGo(self,currentNodeCoord,walls,orientation):
        if self.performingAStar:
            return self.getDirectionOfAStar(currentNodeCoord)

        else:
            unknownNeightborNodes=[]

            unknownNeightborNodes = self.getUnknownNeighbors(self.getNeighbors(currentNodeCoord,walls))
            if len(unknownNeightborNodes)==0:
                self.NodesOfAStart=self.performAStar(currentNodeCoord,self.calculateTargetNodeToAStar())
                self.performingAStar=True
                return self.getDirectionOfAStar(currentNodeCoord)
                #do A* to closest unknown node?
            print(unknownNeightborNodes)
            #priority to follow the same orientation
            for n in unknownNeightborNodes:
                #if n[2]==orientation:
                if n[2]=='north':
                    return n[3]

            return unknownNeightborNodes[0][3]

    def calculateNodeToExplore(self,path):
        for node in reversed(path):
            unknowkNeighbors= self.getUnknownNeighbors(self.getNeighbors(node.pos,node.walls))
            if len(unknowkNeighbors)>0:
                return node

    def getMovementDirectionStateExploringMap(self, currentNodeCoord, orientation):
        elementLocation = self.getElementLocation(currentNodeCoord)
        currentNode = self.map[elementLocation[0]][elementLocation[1]]
        self.historyPathNode.append(currentNode)
        walls = currentNode.walls
        directionNodeToGo = self.getDirectionOfNodeToGo(currentNodeCoord,walls,orientation)

        return directionNodeToGo

    def findPathOfClosestUnknownNode(self,currentNodeCoord):
        #unknownNodes = []
        minFcost=1000;
        minHCost=1000;
        for n in self.closedSet:
        #    print(n.getFcost())
        #    print(self.getUnknownNeighbors(self.getNeighbors(n.pos,n.walls)))
            if len(self.getUnknownNeighbors(self.getNeighbors(n.pos,n.walls)))>0:
                if n.getFcost() <= minFcost:
                    if n.hCost < minHCost:
                        minFcost, minHCost,nodeMinCost = n.getFcost(), n.hCost, n
        #        unknownNodes.append(n)

        bestPath=self.performAStar(currentNodeCoord,nodeMinCost.pos)
        #print(unknownNodes)
#        bestPath = self.performAStar(currentNodeCoord,unknownNodes[0].pos)
#        closestDistance, closestNode = len(bestPath),unknownNodes[0]
#
#        for i in range(1,len(unknownNodes)):
#            print(unknownNodes[i])
#            path = self.performAStar(currentNodeCoord,unknownNodes[i].pos)
#            if len(path) < closestDistance:
#                closestDistance, closestNode, bestPath = len(path), unknownNodes[i], path
#        print("BEST!!")
#        print((closestDistance, closestNode, bestPath))
        return bestPath

    def checkIfCurrentCoordIsInClosedSetUnknown(self,currentNodeCoord):
        print("?????????????????????????????????? checkIfCheeseCordIsInClosedSet ??????????????????????????????????")
        print(self.closedSet)
        print(self.cheeseCoord)
        for n in self.closedSet:
            if n.pos == (currentNodeCoord[0],currentNodeCoord[1]):
                if len(self.getUnknownNeighbors(self.getNeighbors(n.pos,n.walls)))>0:
                    return True
        return False

    def getMovementDirectionToFindBestPath(self,currentNodeCoord,orientation):
        print("$$$$$$$$$$$$$$$$$$$$$$$$ getMovementDirectionToFindBestPath $$$$$$$$$$$$$$$$$$$$$$$$$$$$")
        if self.performingAStar:
            return self.getDirectionOfAStar(currentNodeCoord)
        else:
            print("getMovementDirectionToFindBestPath")
            if not self.inClosestNode:
                path = self.performAStar(self.cheeseCoord,[0,0],True)
                #if not self.checkIfCurrentCoordIsInClosedSetUnknown(currentNodeCoord):
                print("not self.checkIfCheeseCordIsInClosedSet()")
                path = self.findPathOfClosestUnknownNode(currentNodeCoord)
                self.inClosestNode=True
                if len(path)!=0:
                    self.NodesOfAStart = path
                    self.performingAStar=True
                    return self.getDirectionOfAStar(currentNodeCoord)


            elementLocation = self.getElementLocation(currentNodeCoord)
            currentNode = self.map[elementLocation[0]][elementLocation[1]]
            #self.historyPathNode.append(currentNode)
            walls = currentNode.walls

            unknownNeightborNodes=[]

            unknownNeightborNodes = self.getUnknownNeighbors(self.getNeighbors(currentNodeCoord,walls))
            minimunManhantanDistance=1000
            for n in unknownNeightborNodes:
                manhantanDistance = self.manhattanDistanceTwoPoints((n[0],n[1]),[0,0])
                if manhantanDistance<minimunManhantanDistance:
                    minimunManhantanDistance,nodeWithMinimunDistance = manhantanDistance,n
            print("Closest NeightBoor Unknown -> "+str(nodeWithMinimunDistance))
            self.inClosestNode=False
            return nodeWithMinimunDistance[3]

        #    if len(unknownNeightborNodes)==0:
        #        knownNeighbors = self.getNeighbors(currentNodeCoord,walls)
        #        for n in knownNeighbors:
        #            if n[2]==orientation:
        #                return n[3]
        #        return knownNeighbors[0]
        #    else:
        #        return unknownNeightborNodes[0][3]

#    def getMovementDirectionStateExploringMapAfterCheese(self, currentNodeCoord, orientation):
#        if self.performingAStar:
#            return self.getDirectionOfAStar(currentNodeCoord)
#        else:
#            if self.subStateExploring==0: #state to check if all the nodes are knwo around best path of known nodes
#                path = self.performAStar(self.cheeseCoord,[0,0])
#                if not self.checkIfAllNeightboursOfPathAreKnow(path):
#                    nodeToGo = self.calculateNodeToExplore(path)
#                    self.NodesOfAStart = self.performAStar(currentNodeCoord,nodeToGo.pos)
#                    self.performingAStar=True
#                    self.subStateExploring=1
#                    return self.getDirectionOfAStar(currentNodeCoord)
#                else:
#                    self.NodesOfAStart = self.performAStar(currentNodeCoord,self.cheeseCoord)
#                    self.performingAStar=True
#                    self.subStateExploring=2
#                    return self.getDirectionOfAStar(currentNodeCoord)
#            if self.subStateExploring==1:
#                elementLocation = self.getElementLocation(currentNodeCoord)
#                currentNode = self.map[elementLocation[0]][elementLocation[1]]
#                self.historyPathNode.append(currentNode)
#                walls = currentNode.walls
#                #done only for horizontal paths
#                unknownNeighbors = self.getUnknownNeighbors(self.getNeighbors(currentNode.pos, currentNode.walls))
#                if len(unknownNeighbors)>0:
#                    (minimunDistance,unknownNeigh)=self.manhattanDistanceTwoPoints([0,0],[unknownNeighbors[0][0],unknownNeighbors[0][1]]),unknownNeighbors[0]
#
#                    for n in unknownNeighbors:
#                         if self.manhattanDistanceTwoPoints([0,0], [n[0],n[1]])< minimunDistance:
#                             (minimunDistance,unknownNeigh)=self.manhattanDistanceTwoPoints([0,0],[n[0],n[1]]),n
#
#                    return unknownNeigh[3]
#                else:
#                    path = self.performAStar(self.cheeseCoord,[0,0])
#                    if len(path) == self.manhattanDistanceTwoPoints(self.cheeseCoord,[0,0]):
#                        self.NodesOfAStart = self.performAStar(currentNodeCoord,self.cheeseCoord)
#                        self.performingAStar=True
#                        self.subStateExploring=2
#                        return self.getDirectionOfAStar(currentNodeCoord)
#                    if not self.checkIfAllNeightboursOfPathAreKnow(path):
#                        nodeToGo = self.calculateNodeToExplore(path)
#                        self.NodesOfAStart = self.performAStar(currentNodeCoord,nodeToGo.pos)
#                        self.performingAStar=True
#                        self.subStateExploring=1
#                        return self.getDirectionOfAStar(currentNodeCoord)
#                    else:
#                        print("###################GOING TO CHEESE#############################")
#                        self.NodesOfAStart = self.performAStar(currentNodeCoord,self.cheeseCoord)
#                        self.performingAStar=True
#                        self.subStateExploring=2
#                        return self.getDirectionOfAStar(currentNodeCoord)

    def getMovementDirectionToGoToCheese(self, currentNodeCoord, orientation):
        print("getMovementDirectionToGoToCheese")
        if self.performingAStar:
            return self.getDirectionOfAStar(currentNodeCoord)
        self.NodesOfAStart = self.performAStar(currentNodeCoord,self.cheeseCoord)
        self.performingAStar=True
        return self.getDirectionOfAStar(currentNodeCoord)

    def getMovementDirectionFinal(self, currentNodeCoord, orientation):
        print("getMovementDirectionFinal")
        if self.performingAStar:
            return self.getDirectionOfAStar(currentNodeCoord)
        self.NodesOfAStart = self.performAStar(currentNodeCoord,[0,0])
        self.performingAStar=True
        return self.getDirectionOfAStar(currentNodeCoord)

    def manhattanDistanceTwoPoints(self,p,q):
        return abs(p[0]-q[0])+abs(p[1]-q[1])

    def updateSizeMap(self, currentNodeCoord):
        if currentNodeCoord[0] < 0 :
            if -1*currentNodeCoord[0] > self.contDeletedX[0]:
                self.removeMapLine("x","right")
                self.contDeletedX[0]=self.contDeletedX[0]+1
        else:
            if currentNodeCoord[0] > self.contDeletedX[1]:
                self.removeMapLine("x","left")
                self.contDeletedX[1]=self.contDeletedX[1]+1
        if currentNodeCoord[1] < 0 :
            if -1*currentNodeCoord[1] > self.contDeletedY[0]:
                self.removeMapLine("y","up")
                self.contDeletedY[0]=self.contDeletedY[0]+1
        else:
            if currentNodeCoord[1] > self.contDeletedY[1]:
                self.removeMapLine("y","down")
                self.contDeletedY[1]=self.contDeletedY[1]+1

    def removeMapLine(self,coordinate,side):
        if coordinate=="x":
            if side == "left":
                #self.map =
                self.basePoint = tuple(tuple(map(operator.add, (0,-1), self.basePoint)))
                [xList.pop(0) for xList in self.map ]
            else:
                #self.map =
                 [xList.pop() for xList in self.map ]
        else:
            if side == "up":
                self.basePoint = tuple(tuple(map(operator.add, (-1,0), self.basePoint)))
                self.map.pop(0)
            else:
                self.map.pop()

    def checkIfAllClosedNodesAreKnown(self):
        print("checkIfAllClosedNodesAreKnown")
        print(self.closedSet)
        print(len(self.closedSet))
        for n in self.closedSet:
            if len(self.getUnknownNeighbors(self.getNeighbors(n.pos,n.walls)))>0:
                return False
        return True

    def checkIfClosestNodeWithLowerCostIsBase(self):
        print("checkIfClosestNodeWithLowerCostIsBase")
        minFcost=1000;
        minHCost=1000;
        for n in self.closedSet:
        #    print(n.getFcost())
        #    print(self.getUnknownNeighbors(self.getNeighbors(n.pos,n.walls)))
            if len(self.getUnknownNeighbors(self.getNeighbors(n.pos,n.walls)))>0:
                if n.getFcost() <= minFcost:
                    if n.hCost < minHCost:
                        minFcost, minHCost,nodeMinCost = n.getFcost(), n.hCost, n
        print(nodeMinCost)
        if nodeMinCost.pos == (0,0):
            return True
        return False

    def checkIfBestPathIsAvailable(self,currentNodeCoord):
         finalNodeCoord=[0,0]
         path = self.performAStar(self.cheeseCoord,finalNodeCoord,True)
         print("checkIfBestPathIsAvailable")
         #print(len(path),self.manhattanDistanceTwoPoints(self.cheeseCoord,finalNodeCoord))
         print(path)
         if len(path) == self.manhattanDistanceTwoPoints(self.cheeseCoord,finalNodeCoord):
             self.NodesOfAStart=path
             self.performingAStar=True
             print("manhattanDistanceTwoPoints")
             return True

         if self.checkIfAllClosedNodesAreKnown():
             print("checkIfAllClosedNodesAreKnown")
             return True

         if self.checkIfClosestNodeWithLowerCostIsBase():
             return True
    #     if self.checkIfMapIsExploredBetweenTwoNodes(initialNodeCoord,finalNodeCoord):
    #         self.NodesOfAStart=path
    #         self.performingAStar=True
    #         return True

    #     if self.checkIfAllNeightboursOfPathAreKnow(path):
    #         return True

         return False

    def saveCheeseCoord(self,currentNodeCoord):
        self.cheeseCoord = currentNodeCoord

    def __str__(self):
        string = ""
        for x in self.map:
            #string.join(str(x))
            string+=str(x)
            string+="\n"
            #string.join("\n")
        return string


class Node():
    def __init__(self, pos, walls = None):
        self.pos = pos
        if walls is None:
            self.walls = [-1,-1,-1,-1]
        else:
            self.walls=walls
        self.gCost = 0
        self.hCost = 0
        self.parent = None
        #self.unknownNeigh = None
    def getFcost(self):
        return self.hCost + self.gCost
    def reset(self):
        self.gCost = 0
        self.hCost = 0
        self.parent = None
        #self.unknownNeigh = None
    def __str__(self):
     return ""+str(self.pos)+"->"+str(self.walls)
    def __repr__(self):
     return ""+str(self.pos)+"->"+str(self.walls)
