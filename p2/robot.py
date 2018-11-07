import socket, sys, numpy, math
from croblink import CRobLink
from croblink import CRobLinkAngs
from copy import copy

class Robot():
    def __init__(self, interface, systemModel):
        self.interface = interface
        self.state = [0, 0, 0]            #[x y theta]
        self.walls = [0, 0, 0, 0]          #[up down left right]
        self.isCentered = True         #robot reached target node
        self.currentNode = [0, 0]        #[n_x, n_y]
        self.targetNode = [0, 0]            #[n_x, n_y]
        self.Kalman = Kalman(copy(self.state), systemModel)
        self.map = Map(self.currentNode)
        self.orientation = ''       #general robot direction
        self.getMeasurements()
    def motorAction(self, u):
        self.Kalman.setU(u)
        self.interface.driveMotors(u[1],u[0])
    def getMeasurements(self):
        self.interface.readSensors()
        self.measurements = copy(self.interface.measures)
        compass = copy(self.measurements.compass)
        compass = compass*math.pi/180
        if compass < 0: compass += 2*math.pi
        while self.state[2] - compass > math.pi:
            compass += 2*math.pi
        while self.state[2] - compass < -math.pi:
            compass -= 2*math.pi
        self.measurements.compass = compass
        irSensor = copy(self.measurements.irSensor)
        irSensor = [x if x<1/2 else -1.0 for x in irSensor]
        if self.orientation == 'north':
            irSensor = [irSensor[3]]   + irSensor[0:3]
        if self.orientation == 'west':
            irSensor = irSensor[2:4] + irSensor[0:1]
        if self.orientation == 'south':
            irSensor = irSensor[1:4] + [irSensor[0]]
        nearestX = 2 * round(float(self.state[0])/2)
        nearestY = 2 * round(float(self.state[1])/2)
        irSensor[0] -= nearestY + 0.5
        irSensor[1] -= nearestX + 0.5
        irSensor[2] += nearestY - 0.5
        irSensor[3] += nearestX - 0.5
        self.measurements.irSensor = irSensor
        return copy(self.measurements)
    def getState(self):
        self.state = self.Kalman.kalmanStep(self.getMeasurements())
        theta = self.state[2]
        while theta < 0:
            theta += 2*math.pi
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
    def __init__(self, mood, robot):
        self.mood = mood                # 0.66: normal 1: aggressive; 1.25: max;
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
    def getThetaRef(self, dX, dY):
        state = copy(self.robot.state)
        thetaRef = math.atan2(dY, dX)
        if thetaRef < 0: thetaRef += 2*math.pi
        while state[2] - thetaRef > math.pi:
            thetaRef += 2*math.pi
        while state[2] - thetaRef < -math.pi:
            thetaRef -= 2*math.pi
        return thetaRef
    def setControlValue(self):
        if self.robot.measurements.collision == True:
            self.robot.targetNode = copy(self.robot.currentNode)
        self.currentState = self.robot.getState()
        dX = self.robot.targetNode[0]*2 - self.currentState[0]
        dY = self.robot.targetNode[1]*2 - self.currentState[1]
        thetaRef = self.getThetaRef(dX, dY)
        delta = math.sqrt(dX**2 + dY**2)
        u = self.calcControlValue(thetaRef, delta)
        self.robot.motorAction(u)
    def calcControlValue(self, thetaRef, delta):
        errorDelta = -delta
        P_theta = self.mood*0.15
        P_delta = self.mood*0.30
        theta = self.robot.state[2]
        errorTheta = theta - thetaRef
        ab = abs(errorTheta)*180/math.pi
        if ab > 5 and ab <= 9:
            P_delta = P_delta * (-0.25*ab + 9/4)
        if ab > 9:
            P_delta = 0
        if abs(errorDelta) < 0.3:
            P_delta = 0
            modulo = math.fmod(theta, 2*math.pi)
            while modulo < 0: modulo += 2*math.pi
            if self.robot.orientation == 'north':
                errorTheta = modulo - math.pi/2
            if self.robot.orientation == 'east':
                errorTheta = modulo
                if errorTheta > math.pi: errorTheta -= 2*math.pi
            if self.robot.orientation == 'south':
                errorTheta = modulo - 3*math.pi/2
            if self.robot.orientation == 'west':
                errorTheta = modulo - math.pi
            if abs(errorTheta) < 4*math.pi/180:
                errorTheta = 0
                self.robot.currentNode = copy(self.robot.targetNode)
                self.robot.isCentered = True
        rightWheel = errorDelta*P_delta*(-1) + errorTheta*P_theta*(-1)
        leftWheel  = errorDelta*P_delta*(-1) - errorTheta*P_theta*(-1)
        u = [rightWheel, leftWheel]
        return u

class Kalman():
    def __init__(self, initialBelief, systemModel):
        self.systemModel = systemModel
        self.belief = initialBelief
    def updateSystemMatrix(self,state):
        x=0
    def kalmanStep(self, measurements):
        x=0
        return belief

class Map():
    def __init__(self, startingPos):
        self.nodes=0

class Node():
    def __init__(self, pos, walls = [0, 0, 0, 0]):
        self.pos = pos
        self.walls = walls
