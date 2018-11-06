import socket, sys, numpy, math
from croblink import CRobLink
from croblink import CRobLinkAngs

class Robot():
    def __init__(self, interface, systemModel):
        self.interface = interface
        self.state = [0, 0, 0]            #[x y theta]
        self.walls = [0, 0, 0, 0]          #[up down left right]
        self.isCentered = True         #robot reached target node
        self.currentNode = [0, 0]        #[n_x, n_y]
        self.targetNode = [0, 0]            #[n_x, n_y]
        self.Kalman = Kalman(self.state, systemModel)
        self.map = Map(currentNode)
        self.measurements = cif.measurements
        self.d = 1
        self.orientation = 'east'       #general robot direction
    def motorAction(self, u):
        self.kalman.setU(u)
        self.interface.driveMotors(u[1],u[0])
    def getMeasurements(self):
        self.interface.readSensors()
        self.measurements = self.interface.measurements
        compass = self.measurements.compass
        compass = compass*math.pi/180
        if compass < 0: compass += 2*math.pi
        while self.state[2] - compass > math.pi:
            compass += 2*math.pi
        while self.state[2] - compass < -math.pi:
            compass -= 2*math.pi
        self.measurements.compass = compass
        irSensor = self.measurements.irSensor
        irSensor = [x if x<self.d/2 else -1.0 for x in irSensor]
        if self.state == 'north':
            irSensor = irSensor[3]   + irSensor[0:2]
        if self.state == 'west':
            irSensor = irSensor[2:3] + irSensor[0:1]
        if self.state == 'south':
            irSensor = irSensor[1:3] + irSensor[0]

        return self.measurements
    def getState(self):
        self.state = self.Kalman.kalmanStep(self.getMeasurements())
        theta = self.state[2]
        theta = math.fmod(theta, 2*math.pi)
        pizzaSection = theta//(math.pi/4)
        if pizzaSection = 0 or pizzaSection = 7:
            self.orientation = 'east'
        if pizzaSection = 1 or pizzaSection = 2:
            self.orientation = 'north'
        if pizzaSection = 3 or pizzaSection = 4:
            self.orientation = 'west'
        if pizzaSection = 5 or pizzaSection = 6:
            self.orientation = 'south'
        return self.state

# Functions for the continuous world
class Controller():
    def __init__(self, mood, robot):
        self.mood = mood                #0: asleep; 0.66: normal 1: aggressive; 10: cocaine;
        self.robot = robot
        self.currentState = robot.state
    def move(self,direction):           #direction: up, down, left, right
        if robot.isCentered:
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
        state = self.currentState
        thetaRef = math.atan2(dY, dX)
        if thetaRef < 0: thetaRef += 2*math.pi
        while state[2] - thetaRef > math.pi:
            thetaRef += 2*math.pi
        while state[2] - thetaRef < -math.pi:
            compass -= 2*math.pi
        return thetaRef
    def setControlValue(self):
        if self.robot.measurements.collision = True:
            self.robot.targetNode = self.robot.currentNode
        self.currentState = self.robot.getState()
        dX = self.robot.targetNode[0]*2*self.robot.d - state[0]
        dY = self.robot.targetNode[1]*2*self.robot.d - state[1]
        thetaRef = getThetaRef(dX, dY)
        delta = math.sqrt(dX** + dY**)
        u = calcControlValue(thetaRef, delta)
        self.robot.motorAction(u)
    def calcControlValue(self, thetaRef, delta):
        P_theta = self.mood*0.15
        P_delta = self.mood*0.30
        theta = self.robot.state[2]
        errorTheta = theta - thetaRef
        if errorTheta > 5 and errorTheta <= 9:
            P_delta = P_delta * (-0.25*errorTheta + 9/4)
        if errorTheta > 9:
            P_delta = 0
        if delta < 0.3:
            P_delta = 0
            if self.robot.orientation == 'north':
                errorTheta = math.fmod(theta, 2*math.pi) - math.pi/2
            if self.robot.orientation == 'east':
                errorTheta = math.fmod(theta, 2*math.pi)
                if errorTheta > math.pi:
                    errorTheta -= 2*math.pi
            if self.robot.orientation == 'south':
                errorTheta = math.fmod(theta, 2*math.pi) - 3*math.pi/2
            if self.robot.orientation == 'west':
                errorTheta = math.fmod(theta, 2*math.pi) - math.pi
            if errorTheta < 4*math.pi/180:
                self.robot.currentNode = self.robot.targetNode
                self.robot.isCentered = True
        rightWheel = delta*P_delta + errorTheta*P_theta
        leftWheel  = delta*P_delta - errorTheta*P_theta
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
        self.nodes={startingPos: Node(startingPos, walls)}

class Node():
    def __init__(self, pos, walls = [0, 0, 0, 0]):
        self.pos = pos
        self.walls = walls
