import socket, sys, numpy, math
from croblink import CRobLinkAngs
from copy import copy
import operator
import pdb
from random import randint

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
        self.indexNodeOfAStar = -1
        self.historyPathNode=[]
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

#    def updateTargetNode(self,mode=None):
#        if self.targetIndex[0] > len(self.map)-1 or self.targetIndex[1] > len(self.map[0])-1:
#            self.targetIndex = (len(self.map)-1,len(self.map[0])-1)
#        if mode=="getPointInZoneLessKnown":
    #        self.targetIndex = self.getPointInZoneLessKnown()
    #    print(self.map[self.targetIndex[0]][self.targetIndex[1]])
#        #if mode=="opposite":


#    def getPointInZoneLessKnown(self):
#        gridContList=[0,0,0,0,0,0]
#        grid=0
#        mapSizeX = len(self.map[0])
#        mapSizeY = len(self.map)
##        gridXSize=mapSizeX/3
#        gridYSize=mapSizeY/2
#        grid1Size = (mapSizeX-gridXSize*3,mapSizeY-gridYSize*2)
#        sumgridX=gridXSize
#        sumgridY=gridYSize
#

#        for y in range(0,2):
#            if y==1:
#                gridMap = self.map[gridYSize*y:(gridYSize+grid1Size[1])*(y+1)]
#            else:
#                gridMap = self.map[gridYSize*y:gridYSize*(y+1)]
#            for x in range(0,3):
#                for xList in gridMap:
#                    if x==3:
#                        gridXList = xList[gridXSize*x:(gridXSize+grid1Size[0])*(x+1)]
#                    else:
#                        gridXList = xList[gridXSize*x:gridXSize*(x+1)]
#                    for n in gridXList:
#                        if n.walls.count(-1)>0:
#                            gridContList[grid]+=1

#                grid+=1
#        gridLessKnown = gridContList.index(max(gridContList))
        #pdb.set_trace()
#        if gridLessKnown==0:
#            return (randint(0, gridYSize),randint(0, gridXSize))
#        if gridLessKnown==1:
#            return (randint(0, gridYSize),randint(gridXSize, gridXSize*2))
#        if gridLessKnown==2:
#            return (randint(0, gridYSize), randint(gridXSize*2, gridXSize*3))
#        if gridLessKnown==3:
#            return (randint(gridYSize, gridYSize*2),randint(0, gridXSize))
#        if gridLessKnown==4:
#            return (randint(gridYSize, gridYSize*2),randint(gridXSize, gridXSize*2))
#        if gridLessKnown==5:
#            return (randint(gridYSize, gridYSize*2),randint(gridXSize*2, gridXSize*3))




#    def getDistanceOfEachNearNodeToTarget(self,walls,currentNode):
#        dist=[1000,1000,1000,1000]
#        targetNodeCoord= self.map[self.targetIndex[0]][self.targetIndex[1]].pos
#
#        if walls[0]==0:
#            dist[0]= self.manhattanDistanceTwoPoints(tuple(tuple(map(operator.add, (0,1), currentNode))),targetNodeCoord)
#        if walls[1]==0:
#            dist[1]= self.manhattanDistanceTwoPoints(tuple(tuple(map(operator.add, (1,0), currentNode))),targetNodeCoord)
#        if walls[2]==0:
#            dist[2]= self.manhattanDistanceTwoPoints(tuple(tuple(map(operator.add, (0,-1), currentNode))),targetNodeCoord)
#        if walls[3]==0:
#            dist[3]= self.manhattanDistanceTwoPoints(tuple(tuple(map(operator.add, (-1,0), currentNode))),targetNodeCoord)
#        return dist

#    def getMovement(self, currentNode):
#        elementLocation = self.getElementLocation(currentNode)
#        walls = self.map[elementLocation[0]][elementLocation[1]].walls
#        dist = self.getDistanceOfEachNearNodeToTarget(walls,currentNode)
#
#        if min(dist)> self.lastMinimunDist:
#            self.updateTargetNode("getPointInZoneLessKnown")
#
#        minimunIndex= dist.index(min(dist))
#
#        self.lastMinimunDist = min(dist)
#        print(dist)
#        print(self.targetIndex)

##        print(self.map[self.targetIndex[0]][self.targetIndex[1]])
#        if minimunIndex == 0:
#            return "up"
#        if minimunIndex == 1:
#            return "right"
#        if minimunIndex == 2:
#            return "down"
#        return "left"
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

    def getPath(self,startingNode,targetNode):
        path=[]
        currentNode=targetNode

        while currentNode.pos!=startingNode.pos:
            path.append(currentNode)
            print(currentNode.pos)
            currentNode = currentNode.parent
        print("#getpath()")
        print("path: "+str(path))
        self.NodesOfAStart=path
        self.performingAStar=True


    def performAStart(self,startingNodeCoord,targetNodeCoord):
        print("PerforminAStart")
        print("startingNodeCoord: "+str(startingNodeCoord)+"targetNodeCoord: "+str(targetNodeCoord))
        openSet = []
        closedSet = []
        currentElementLocation = self.getElementLocation(startingNodeCoord)
        targetElementLocation = self.getElementLocation(targetNodeCoord)
        startingNode = self.map[currentElementLocation[0]][currentElementLocation[1]]
        openSet.append(startingNode)
        targetNode = self.map[targetElementLocation[0]][targetElementLocation[1]]

        while len(openSet)>0:
            currentNode = openSet[0]

            for n in openSet:
                if n.getFcost() < currentNode.getFcost() or n.getFcost() == currentNode.getFcost() and n.hCost < currentNode.hCost:
                    currentNode = n
            openSet.remove(n)
            closedSet.append(n)

            if currentNode.pos==targetNode.pos:
                self.getPath(startingNode,currentNode)

            for neightborCoor in self.getKnownNeighbors(self.getNeighbors(currentNode.pos,currentNode.walls)):
                neighIndex = self.getElementLocation((neightborCoor[0],neightborCoor[1]))
                neighborNode = self.map[neighIndex[0]][neighIndex[1]]
                if not (neighborNode in closedSet):
                    newMovementCostToNeigh = currentNode.gCost + 1
                    if newMovementCostToNeigh < neighborNode.gCost or not ( neighborNode in openSet):
                        neighborNode.gCost= newMovementCostToNeigh
                        neighborNode.hCost = self.manhattanDistanceTwoPoints(neighborNode.pos,targetNodeCoord)
                        neighborNode.parent=currentNode

                        if not (neighborNode in openSet):
                            openSet.append(neighborNode)


    def followingAStar(self,currentNodeCoord):
        print(self.NodesOfAStart)
        print(self.indexNodeOfAStar)
        print(len(self.NodesOfAStart))
        nodeToGo = self.NodesOfAStart[self.indexNodeOfAStar]

        if self.indexNodeOfAStar==-len(self.NodesOfAStart):
            self.indexNodeOfAStar=-1
            self.NodesOfAStart=[]
            self.performingAStar=False
            for xList in self.map:
                for node in xList:
                    node.reset()
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
            return self.followingAStar(currentNodeCoord)

        else:
            unknownNeightborNodes=[]

            neightborNodes = self.getNeighbors(currentNodeCoord,walls)
            for n in neightborNodes:
                nIndexLoc = self.getElementLocation((n[0],n[1]))
                if self.map[nIndexLoc[0]][nIndexLoc[1]].walls.count(-1)>0:
                    unknownNeightborNodes.append(n)

            if len(unknownNeightborNodes)==0:
                self.performAStart(currentNodeCoord,self.calculateTargetNodeToAStar())
                return self.followingAStar(currentNodeCoord)
                #do A* to closest unknown node?

            #priority to follow the same orientation
            for n in unknownNeightborNodes:
                if n[2]==orientation:
                    return n[3]
            return unknownNeightborNodes[0][3]


    def getMovementDirection(self, currentNodeCoord, orientation):
         elementLocation = self.getElementLocation(currentNodeCoord)
         currentNode = self.map[elementLocation[0]][elementLocation[1]]
         self.historyPathNode.append(currentNode)
         walls = currentNode.walls
         directionNodeToGo = self.getDirectionOfNodeToGo(currentNodeCoord,walls,orientation)
         return directionNodeToGo

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

    def getMap(self):
        return self.map
    def __str__(self):
        string = ""
        for x in self.map:
            #string.join(str(x))
            string+=str(x)
            string+="\n"
            #string.join("\n")
        return string

class NodeA():
    def __init__(self, pos, walls = None):
        self.pos = pos
        if walls is None:
            self.walls = [-1,-1,-1,-1]
        else:
            self.walls=walls


    def __str__(self):
     return ""+str(self.pos)+"->"+str(self.walls)
    def __repr__(self):
     return ""+str(self.pos)+"->"+str(self.walls)

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
    def getFcost(self):
        return self.hCost + self.gCost
    def reset(self):
        self.gCost = 0
        self.hCost = 0
        self.parent = None
    def __str__(self):
     return ""+str(self.pos)+"->"+str(self.walls)
    def __repr__(self):
     return ""+str(self.pos)+"->"+str(self.walls)
