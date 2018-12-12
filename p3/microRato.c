#include "rmi-mr32.h"
#include "rmi-mr32.c"
#include <math.h>
#include <string.h>
#include <stdlib.h>



enum states { EXPLORINGMAP, FINDINGBESTPATH, GOINGTOTOKEN, GOINGTOBASE } state;
enum directions { NORTH, EAST, SOUTH, WEST} dir;
enum movementstate { MOVING, EXPECTINGCOMMAND, CENTERING } movstate;
double x, y, t, dx, dy;
const double d = 12.5; //grid distance
const int forwardSpeed = 40;
const int slowTurn     =  5;
const int fastTurn     = 15;


void rotateRel(int maxVel, double deltaAngle);
void motorCommand(int comR, int comL, int *buffer);
int stabledetection(int *sensorBuffer, int state);
void updateMap();

int main(void)
{
    int groundSensor;
    initPIC32();
    closedLoopControl( true );
    setVel2(0, 0);
    printf("MicroRato, robot %d\n\n\n", ROBOT);

    while(1)
    {
        printf("Press start to continue\n");
        while(!startButton());
        //Initial state
        enableObstSens();
        state    = EXPLORINGMAP;
        movstate = MOVING;
        int     motorBuffer [2][6] = { {0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0} }; //TODO: move
        int     sensorBuffer   [5] = { 0, 0, 0, 0, 0 }; //TODO: move
        int     node [2]           = {0, 0};
        int     paths              = 0b1000;    //bits: north, east, south, west
        int     groundStable       = 0b00000;
        double  beaconX            = 0.0;
        double  beaconY            = 0.0;
        bool    updateAvailable    = true;

        do
        {
            waitTick40ms();						// Wait for next 40ms tick
            readAnalogSensors();				// Fill in "analogSensors" structure
            groundSensor = readLineSensors(0);	// Read ground sensor
            //get robot position
            getRobotPos(&x, &y, &t);
            dx = x - beaconX;
            dy = y - beaconY;
            // update ground sensor buffer
            for(i=0; i<=3; i++){
              sensorBuffer[i] = sensorBuffer[i+1];
            }
            sensorBuffer[4] = groundSensor;

            switch (movstate){
              case MOVING:
                if (fmax( abs(dx), abs(dy) ) >= d + 1.0){ //detect straight path
                  updateAvailable = true;
                  paths = 0b1010;
                  nextNode(*node, dir);
                  beaconX = x;
                  beaconY = y;
                  if(abs(dx) > abs(dy)){    //correct offset of 1.0
                    beaconX += -sgn(dx);
                  }else{
                    beaconY += -sgn(dy);
                  }
                }
                groundStable = stabledetection(*sensorBuffer, 1);
                if (groundStable & 0b10001){      //detect intersection
                  movstate = CENTERING;
                  beaconX = x;
                  beaconY = y;
                }
                // line following algorithm
                switch(groundSensor & 0b01110){      //lookup table for behavior
                  case 0b01110:
                    comL = forwardSpeed;
                    comR = forwardSpeed;
                  break;
                  case 0b00100:
                    comL = forwardSpeed;
                    comR = forwardSpeed;
                  break;
                  case 0b01100:
                    comL = forwardSpeed - slowTurn;
                    comR = forwardSpeed + slowTurn;
                  break;
                  case 0b00110:
                    comL = forwardSpeed + slowTurn;
                    comR = forwardSpeed - slowTurn;
                  break;
                  case 0b01000:
                    comL = forwardSpeed - fastTurn;
                    comR = forwardSpeed + fastTurn;
                  break;
                  case 0b00010:
                    comL = forwardSpeed + fastTurn;
                    comR = forwardSpeed - fastTurn;
                  break;
              break;
              case CENTERING:
                comL = forwardSpeed/2;
                comR = forwardSpeed/2;
                groundStable = stabledetection(*sensorBuffer, 0);

                //move forward until center
                //analyse sensor data ->create node
                //check for finish line
                //if edge turn and move forward ->MOVING
                //if intersection -> EXPECTINGCOMMAND + zero velocity twice
              break;
              case EXPECTINGCOMMAND:
                comL = 0;
                comR = 0;
              break;
            }
            motorCommand(comR, comL, *motorBuffer);

            if(updateAvailable){
              updateMap();
              updateAvailable = false;
            }

            if(movstate == EXPECTINGCOMMAND){
                switch (state) {
                  case EXPLORINGMAP:

                  break;
                  case FINDINGBESTPATH:

                  break;
                  case GOINGTOTOKEN:

                  break;
                  case GOINGTOBASE:

                  break;
                }
            }


        } while(!stopButton());
        disableObstSens();
        setVel2(0, 0);
    }
    return 0;
}



void updateMap(){

}

int stabledetection(int *sensorBuffer, int state){
return 0;
}

void motorCommand(int newValue[2], int *buffer){

setVel2(0, 0);
}


void rotateRel(int maxVel, double deltaAngle){
    double x, y, t;
    double targetAngle;
    double error;
    double integral = 0;
    double KP_ROT = 50;
    double KI_ROT	= 0;
    int cmdVel;

    getRobotPos(&x, &y, &t);
    targetAngle = normalizeAngle(t + deltaAngle);
    do
    {
        waitTick40ms();
        getRobotPos(&x, &y, &t);
        error = normalizeAngle(targetAngle - t);

        integral += error;
        integral = integral > PI / 2 ? PI / 2: integral;
        integral = integral < -PI / 2 ? -PI / 2: integral;

        cmdVel = (int)((KP_ROT * error) + (integral * KI_ROT));
        cmdVel = cmdVel > maxVel ? maxVel : cmdVel;
        cmdVel = cmdVel < -maxVel ? -maxVel : cmdVel;

        setVel2(-cmdVel, cmdVel);
    } while (fabs(error) > 0.01);
    setVel2(0, 0);
}
