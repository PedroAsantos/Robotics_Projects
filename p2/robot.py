import socket, sys, numpy, math
from croblink import CRobLink
from croblink import CRobLinkAngs
from copy import copy

class Robot():
    def __init__(self, interface, systemModel):
        self.interface = interface
        self.state = [0, 0, 0]              #[x y theta]
        self.walls = [0, 0, 0, 0]           #[north east south west] -1: unknown
        self.isCentered = True              #robot reached target node
        self.currentNode = [0, 0]           #[n_x, n_y]
        self.targetNode = [0, 0]            #[n_x, n_y]
        self.Kalman = Kalman(copy(self.state), systemModel)
        self.map = Map(self.currentNode)
        self.orientation = ''               #general robot direction (e.g. east)
        self.getMeasurements()
    def motorAction(self, u):               #apply control input u
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
        irSensor[0] = -irSensor[0] + nearestY + 0.5
        irSensor[1] = -irSensor[1] + nearestX + 0.5
        irSensor[2] =  irSensor[2] + nearestY - 0.5
        irSensor[3] =  irSensor[3] + nearestX - 0.5
        irSensor = [irSensor[x] if walls[x] ==1 else [] for x in range(4)]
        # empty entry: no valid measurement
        return (walls, irSensor)
    def getState(self):
        self.state = self.Kalman.kalmanStep(self.getMeasurements())
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
        if abs(errorDelta) < 0.25:       # node tolerance
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
        u = [rightWheel, leftWheel]
        return u

class Kalman():
    nada=[]

        # functions for the discrete world:

class Map():
    def __init__(self, startingPos):
        self.nodes=0

class Node():
    def __init__(self, pos, walls = [0, 0, 0, 0]):
        self.pos = pos
        self.walls = walls
