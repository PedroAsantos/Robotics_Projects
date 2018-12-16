//#include "rmi-mr32.h"
#include "rmi-mr32.c"
#include "bluetooth_comm.h"
#include "bluetooth_comm.c"
#include <math.h>
#include <string.h>
#include <stdlib.h>
// TODO: Implement functions,  draw state machine, measure robot radius

typedef enum { EXPLORINGMAP, FINDINGBESTPATH, GOINGTOTOKEN, GOINGTOBASE } states;
typedef enum { NORTH = 8, EAST = 4, SOUTH = 2, WEST = 1} directions;
typedef enum { MOVING, EXPECTINGCOMMAND, CENTERING } movementstate;
typedef enum { FRONT = 0, BACK = 2, LEFT = 1, RIGHT = 3} relative;
double x, y, t, dx, dy;
const double d = 125; //grid distance in [mm]
const double robotRadius = 100;
const int    forwardSpeed = 45;
const int    slowTurn     =  4;
const int    fastTurn     = 10;


void rotateRel(int maxVel, double deltaAngle);
void motorCommand(int comR, int comL);
int stabledetection(int groundSensor, bool dump);
int flipdir( directions dir, relative turn);
void nextNode(int *node, directions dir);
void updateMap(int *node, int paths);
int sumofbits(int number);
void moverel(directions *dir, relative turn);
void moveabs(directions *dir, directions absolute);

int main(void)
{
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
        //enableObstSens();         //Not needed
        states        state    = EXPLORINGMAP;
        movementstate movstate = MOVING;
        directions    dir      = NORTH;
        int     node [2]           = {0};
        int     paths              = 0b1000;    //bits: north, east, south, west
        int     groundSensor       = 0b00000;
        int     groundStable       = 0b00000;   //debounced groundSensor
        int     groundLR           = 0b00000;   //Tigger left/right sensor
        int     comL               = 0;         //left motor command
        int     comR               = 0;         //right motor command
        double  beaconX            = 0.0;       //beacon coords
        double  beaconY            = 0.0;
        double  distToBeacon       = 0.0;
        bool    updateAvailable    = true;      //true when new node discovered

        groundSensor = stabledetection(groundSensor, true);

        do
        {
            waitTick40ms();						// Wait for next 40ms tick
            readAnalogSensors();				// Fill in "analogSensors" structure
            groundSensor = readLineSensors(0);	// Read ground sensor
            //get robot position
            getRobotPos(&x, &y, &t);
            dx = x - beaconX;
            dy = y - beaconY;
            groundStable = stabledetection(groundSensor, false);
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
                if ( (groundStable == 0) && (fmax( abs(dx), abs(dy) ) >= d - robotRadius) ){    //dead end detected
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
                  moverel(&dir, BACK);
                  dir = flipdir(dir, BACK);
                  break;
                }
                if (groundStable & 0b10001){      //intersection detected
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
                      moverel(&dir, LEFT);
                      dir = flipdir(dir, LEFT);
                    }else if ( paths & flipdir(dir, RIGHT) ){ //right turn only option
                      moverel(&dir, RIGHT);
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

            motorCommand(comR, comL);
            if(updateAvailable){
              updateMap(node, paths);
              updateAvailable = false;
            }

            if(movstate == EXPECTINGCOMMAND){
                switch (state) {
                  case EXPLORINGMAP:
                    //algorithm for exploring north
                    if(paths & 0b010000){
                      moveabs(&dir, NORTH);
                    }else if(paths & 0b001000){
                      moveabs(&dir, EAST);
                    }else if(paths & 0b010000){
                      moveabs(&dir, SOUTH);
                    }else{
                      moveabs(&dir, WEST);
                    }
                    printf("Dir %d; state: %d; dist: %.0f; %.0f; node: %d; %d; grSt: %d; update: %d\n", dir, movstate, dx, dy, node[0], node[1], groundStable, updateAvailable);
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


void updateMap(int *node, int paths){

}

int stabledetection(int groundSensor, bool dump){
  const int l = 5;
  static int sensorBuffer[5] = {0};
  static int state = 0b00100;
  int i, j, curState;
  bool trigger;

  for( i = 0; i < l-1; i++){     //update ground sensor buffer
    if (dump){
      sensorBuffer[i] = 0;
    }else{
      sensorBuffer[i] = sensorBuffer[i+1];
    }
  }
  sensorBuffer[l-1] = groundSensor;
  if (dump){
    state = 0b00100;        //reset state after dump
  }
  for (i = 0; i < l; i++){     //check for stable signal
    curState = (state >> i) & 0b00001;
    trigger = true;
    for (j = 0; j < l; j++) {
      if ( (sensorBuffer[j] >> i) & (0b00001 == curState) ){
        trigger = false;
      }
      if (trigger){
        state ^= (0b00001 << i);       //toggle sensor bit
      }
    }
  }
  return state;
}

void motorCommand(int comR, int comL){
  const int len = 6;
  static int motorBuffer[2][6] = {{0}};
  int l, r, i;

  for( i = 0; i < len-1; i++){     //update motor buffer
      motorBuffer[0][i] = motorBuffer[0][i+1];
      motorBuffer[1][i] = motorBuffer[1][i+1];
  }
  motorBuffer[0][len-1] = comR;
  motorBuffer[0][len-1] = comL;
  l = 0;
  r = 0;
  for( i = 0; i < len-1; i++){     //calculate mean values of buffer
      r += motorBuffer[0][i]/len;
      l += motorBuffer[1][i]/len;
  }
  setVel2(r, l);
}

void nextNode(int *node, directions dir){
  int incrementX = 0;
  int incrementY = 0;

  switch (dir){
    case NORTH:
      incrementX += 1;
    break;
    case EAST:
      incrementY -= 1;
    break;
    case SOUTH:
      incrementX -= 1;
    break;
    case WEST:
      incrementY += 1;
    break;
    }
    node[0] += incrementX;
    node[1] += incrementY;
}

int sumofbits(int number){
  int sum = 0;

  while(number > 0){
    sum += number&0b000001;
    number >>= 1;
  }
  return sum;
}

int flipdir( directions dir, relative turn){
int temp = dir;
int upper, lower;

temp <<= turn;
upper = temp >> 4;
lower = temp & 0b1111;
return (upper | lower);
}

void moverel(directions *dir, relative turn){
  switch (turn){
    case FRONT:
      //do nothing
    break;
    case RIGHT:
      rotateRel(70, -M_PI/2);
    break;
    case BACK:
      rotateRel(70, M_PI);
    break;
    case LEFT:
      rotateRel(70, M_PI/2);
    break;
  }
  stabledetection(0b00000, true);
  *dir = flipdir(*dir, turn);
}

void moveabs(directions *dir, directions absolute){
  int continuous = (*dir) | (*dir>>4);
  if ( continuous & (absolute>>0) ){
    moverel(dir, FRONT);
  }else if( continuous & (absolute>>1) ){
    moverel(dir, RIGHT);
  }else if( continuous & (absolute>>2) ){
    moverel(dir, BACK);
  }else if( continuous & (absolute>>3) ){
    moverel(dir, LEFT);
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
