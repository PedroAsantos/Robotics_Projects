import socket, sys, numpy, math
from croblink import CRobLinkAngs
from copy import copy
import operator
import pdb

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
        irSensor[0] = -irSensor[0] + nearestY + 0.45
        irSensor[1] = -irSensor[1] + nearestX + 0.45
        irSensor[2] =  irSensor[2] + nearestY - 0.45
        irSensor[3] =  irSensor[3] + nearestX - 0.45
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
        self.systemModel.R[3,3] = 0.015*(0.5*self.u[0,0] + 0.5*self.belief[3,0])
        self.systemModel.R[4,4] = 0.015*(0.5*self.u[1,0] + 0.5*self.belief[4,0])

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
        xList= []
        for y in range(6,-7,-1):
            for x in range(-13,14):
                node = Node((x,y))
                xList.append(node)
            self.map.append(xList)
            xList = []
    def putWalls(self,currentNode,walls):
        elementLocation= getElementLocation(currentNode)
        #add walls to the current position of the robot
        self.map[elementLocation[0]][elementLocation[1]].walls = walls
        #add walls to the nodes near this walls
        if walls[0]==1:
            tempNodeAboveLocation = tuple(map(operator.add, elementLocation, (0,1)))
            if tempNodeAboveLocation[0] < len(self.map[0]) & tempNodeAboveLocation[1] < len(self.map):
                walls = self.map[tempNodeAboveLocation[0]][tempNodeAboveLocation[1]].walls
                walls[2]=1
                self.map[tempNodeAboveLocation[0]][tempNodeAboveLocation[1]].walls=walls
        if walls[1]==1:
            tempNodeEastLocation = tuple(map(operator.add, elementLocation, (1,0)))
            if tempNodeEastLocation[0] < len(self.map[0]) & tempNodeEastLocation[1] < len(self.map):
                walls = self.map[tempNodeEastLocation[0]][tempNodeEastLocation[1]].walls
                walls[3]=1
                self.map[tempNodeEastLocation[0]][tempNodeEastLocation[1]].walls=walls
        if walls[2]==1:
            tempNodeBelowLocation = tuple(map(operator.add, elementLocation, (0,-1)))
            if tempNodeBelowLocation[0] < len(self.map[0]) & tempNodeBelowLocation[1] < len(self.map):
                walls = self.map[tempNodeBelowLocation[0]][tempNodeBelowLocation[1]].walls
                walls[0]=1
                self.map[tempNodeBelowLocation[0]][tempNodeBelowLocation[1]].walls=walls
        if walls[3]==1:
            tempNodeWestLocation = tuple(map(operator.add, elementLocation, (-1,0)))
            if tempNodeWestLocation[0] < len(self.map[0]) & tempNodeWestLocation[1] < len(self.map):
                walls = self.map[tempNodeWestLocation[0]][tempNodeWestLocation[1]].walls
                walls[1]=1
                self.map[tempNodeWestLocation[0]][tempNodeWestLocation[1]].walls=walls




    def getElementLocation(self, currentNode):
        basePoint=(5,12)
        return tuple(map(operator.add, basePoint, currentNode))


    def removeMapLine(self,coordinate,side):
        if coordinate=="x":
            if side == "left":
                #self.map =
                [xList.pop(0) for xList in self.map ]
            else:
                #self.map =
                 [xList.pop() for xList in self.map ]
        else:
            if side == "up":
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
class Node():
    def __init__(self, pos, walls = [0, 0, 0, 0]):
        self.pos = pos
        self.walls = walls
    def __str__(self):
     return ""+str(self.pos)
    def __repr__(self):
     return ""+str(self.pos)
