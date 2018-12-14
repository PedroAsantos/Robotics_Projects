//#include "rmi-mr32.h"
#include "rmi-mr32.c"
#include "bluetooth_comm.h"
#include "bluetooth_comm.c"
#include <math.h>
#include <string.h>
#include <stdlib.h>
// TODO: Implement functions, implement bluetooth, draw state machine

enum states { EXPLORINGMAP, FINDINGBESTPATH, GOINGTOTOKEN, GOINGTOBASE } state;
enum directions { NORTH = 8, EAST = 4, SOUTH = 2, WEST = 1} dir;
enum movementstate { MOVING, EXPECTINGCOMMAND, CENTERING } movstate;
enum relative { FRONT, BACK, LEFT, RIGHT} relDir;
double x, y, t, dx, dy;
const double d = 125; //grid distance in [mm]
const double robotRadius = 100;       //TODO: measure
const int forwardSpeed = 45;
const int slowTurn     =  4;
const int fastTurn     = 10;


void rotateRel(int maxVel, double deltaAngle);
void motorCommand(int comR, int comL, int *buffer);
int stabledetection(int groundSensor);
int flipdir( int dir, int turnAngle);
void nextNode(int *node, int dir);
void updateMap();
int sumofbits(int number);
void moverel(int dir, int currentDir);

int main(void)
{
    int groundSensor;
    initPIC32();
    configBTUart(3, 115200); // Configure Bluetooth UART
    bt_on();     // enable bluetooth channel; printf
                // is now redirected to the bluetooth UART
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
        dir      = NORTH;
        int     motorBuffer [2][6] = {{0}}; //TODO: move
        int     node [2]           = {0};
        int     paths              = 0b1000;    //bits: north, east, south, west
        int     groundStable       = 0b00000;   //debounced groundSensor
        int     groundLR           = 0b00000;   //Tigger left/right sensor
        int     comL               = 0;         //left motor command
        int     comR               = 0;         //right motor command
        double  beaconX            = 0.0;       //beacon coords
        double  beaconY            = 0.0;
        double  distToBeacon       = 0.0;
        bool    updateAvailable    = true;      //true when new node discovered


        do
        {
            waitTick40ms();						// Wait for next 40ms tick
            readAnalogSensors();				// Fill in "analogSensors" structure
            groundSensor = readLineSensors(0);	// Read ground sensor
            //get robot position
            getRobotPos(&x, &y, &t);
            dx = x - beaconX;
            dy = y - beaconY;
            groundStable = stabledetection(groundSensor);
            switch (movstate){

              case MOVING:
                if (fmax( abs(dx), abs(dy) ) >= d + 10.0){ //straight node detected
                  updateAvailable = true;
                  paths = dir | flipdir(dir, BACK);  //update available paths
                  nextNode(node, dir);
                  beaconX = x;              //set beacon to current position
                  beaconY = y;
                  if(abs(dx) > abs(dy)){    //correct offset of 10 mm
                    beaconX -= 10*sgn(dx);
                  }else{
                    beaconY -= 10*sgn(dy);
                  }
                }
                if (groundStable == 0){    //dead end detected
                  updateAvailable = true;
                  paths = flipdir(dir, BACK);
                  nextNode(node, dir);
                  beaconX = x;
                  beaconY = y;
                  if(abs(dx) > abs(dy)){    //correct offset
                    beaconX += (robotRadius - 10.0)*sgn(dx);
                  }else{
                    beaconY += (robotRadius - 10.0)*sgn(dy);
                  }
                  moverel(BACK, dir);
                  dir = flipdir(dir, BACK);
                  break;
                }
                if (groundStable & 0b10001){      //detect intersection
                  movstate = CENTERING;
                  groundLR = 0;
                  comL = 0;
                  comR = 0;
                  beaconX = x;
                  beaconY = y;
                  break;
                }
                // line following algorithm
                switch(groundSensor & 0b01110){      //lookup table for movement behavior
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
                }
              break;

              case CENTERING:           //navigate to center of intersection
                comL         = forwardSpeed/2;
                comR         = forwardSpeed/2;
                groundLR     = groundLR | (groundStable & 0b10001);
                distToBeacon = 0;
                if ((groundStable & 0b10001) == 0){
                  // distance the robot travelled while measuring black
                  distToBeacon = fmax( abs(dx), abs(dy) );
                }

                if (fmax( abs(dx), abs(dy) ) >= - 25.0 + robotRadius){ //center reached
                  comL = 0;
                  comR = 0;
                  beaconX = x;
                  beaconY = y;
                  updateAvailable = true;
                  nextNode(node, dir);
                  if( (distToBeacon >= 40) || (groundStable & 0b10001) ){
                    paths = 0b10000 | flipdir(dir, BACK); //finish line detected
                    movstate = EXPECTINGCOMMAND;
                    break;
                  }else{
                    paths = flipdir(dir, BACK);
                    if (groundStable & 0b00100){
                      paths = paths | dir;
                    }
                    if (groundLR & 0b10000){
                      paths = paths | flipdir(dir, LEFT);
                    }
                    if (groundLR & 0b00001){
                      paths = paths | flipdir(dir, RIGHT);
                    }
                  }
                  if (sumofbits(paths & 0b01111) == 2){   //no intersection, just turn
                    movstate = MOVING;
                    if ( paths & flipdir(dir, LEFT) ){    //left turn only option
                      moverel(LEFT, dir);
                      dir = flipdir(dir, LEFT);
                    }else if ( paths & flipdir(dir, RIGHT) ){ //right turn only option
                      moverel(RIGHT, dir);
                      dir = flipdir(dir, RIGHT);
                    }
                  }else{        //intersection
                    movstate = EXPECTINGCOMMAND;
                  }
                }
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

int stabledetection(int groundSensor){  //TODO    //TODO: dump buffer after turning

  static int sensorBuffer[5] = { 0, 0, 0, 0, 0 };
  // update ground sensor buffer
  int i;
  for( i=0; i<=3; i++){
    sensorBuffer[i] = sensorBuffer[i+1];
  }
  sensorBuffer[4] = groundSensor;
  return 0;
}

void motorCommand(int comR, int comL, int *buffer){     //TODO

setVel2(0, 0);
}

void nextNode(int *node, int dir){          //TODO

}

int sumofbits(int number){              //TODO
  return 0;
}

int flipdir( int dir, int turnAngle){     //TODO
  return 0;
}

void moverel(int dir, int currentDir){
  switch (dir){
    case 1:

    break;
    case 2:

    break;
    case 3:

    break;

    case 4:

    break;
  }
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
