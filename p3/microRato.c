#include "rmi-mr32.h"
#include "rmi-mr32.c"
#include "bluetooth_comm.h"
#include "bluetooth_comm.c"
#include <math.h>
#include <string.h>
#include <stdlib.h>
// TODO: draw state machine, measure robot radius

typedef enum { EXPLORINGMAP, FINDINGBESTPATH, GOINGTOTOKEN, GOINGTOBASE } states;
typedef enum { NORTH = 8, EAST = 4, SOUTH = 2, WEST = 1} directions;
typedef enum { MOVING, EXPECTINGCOMMAND, CENTERING } movementstate;
typedef enum { FRONT = 0, LEFT = 1, BACK = 2, RIGHT = 3} relative;
const double d = 125; //grid distance in [mm]
const double robotRadius = 100;
const int    forwardSpeed = 35;
const int    slowTurn     =  4;
const int    fastTurn     = 10;
double x, y, t;
bool switcher;

void    rotateRel(int maxVel, double deltaAngle);
void    motorCommand(int comR, int comL);
int     stabledetection(int groundSensor, bool dump);
int     flipdir( directions dir, relative turn);
void    nextNode(int *node, directions dir);
void    updateMap(int *node, int paths);
int     sumofbits(int number);
void    moverel(directions *dir, relative turn);
void    moveabs(directions *dir, directions absolute);
double  fmax(double v1, double v2);
int     sgn(double v);

int main(void)
{
    initPIC32();
    //configBTUart(3, 115200); // Configure Bluetooth UART
    //bt_on();   // enable bluetooth; printf is now redirected to bluetooth UART
    closedLoopControl( true );
    setVel2(0, 0);
    printf("MicroRato, robot %d\n\n\n", ROBOT);

    while(1)
    {
        printf("Press start to continue\n");
        while(!startButton());
        //Initialize default state
        states        state        = EXPLORINGMAP;
        movementstate movstate     = MOVING;
        directions    dir          = NORTH;
        int     node [2]           = {0};
        //bits of path variable: finish line detected, north, east, south, west
        int     paths              = 0b1000;
        int     groundSensor       = 0b00000;
        int     groundStable       = 0b00000;   //stabilized groundSensor
        int     groundLR           = 0b00000;   //trigger left/right sensor
        int     comL               = 0;         //left motor command
        int     comR               = 0;         //right motor command
        double  distToBeacon       = 0.0;
        bool    updateAvailable    = true;      //true when new node discovered

        enableObstSens();
        groundStable = stabledetection(groundSensor, true);  //dump buffer
        setRobotPos(0.0, 0.0, 0.0);
        led(0,0);
        led(1,0);

        do
        {
            waitTick40ms();						    // Wait for next 40ms tick
            readAnalogSensors();				  // Fill in "analogSensors" structure
            groundSensor = readLineSensors(0);	// Read ground sensor
            getRobotPos(&x, &y, &t);      //get robot position
            groundStable = stabledetection(groundSensor, false);

            switch (movstate){
              case MOVING:
                if ( x  >= d+10.0){//straight node detected
                  updateAvailable = true;
                  paths = dir | flipdir(dir, BACK);     //update available paths
                  nextNode(node, dir);
                  setRobotPos(10.0, 0.0, 0.0);   //set beacon to current position
                }
                if ( (groundStable == 0) &&             //dead end detected
                    ( x  >= d - robotRadius) ){
                  updateAvailable = true;
                  paths = flipdir(dir, BACK);           //only path leads back
                  nextNode(node, dir);                  //update node and beacon
                  moverel(&dir, BACK);                  //turn back
                  setRobotPos(-10.0 + robotRadius, 0.0, 0.0);
                  break;
                }
                if (groundStable & 0b10001){            //intersection detected
                  movstate = CENTERING;
                  //initialize centering process by resetting variables
                  groundLR = 0;
                  comL     = 0;
                  comR     = 0;
                  switcher = true;
                  distToBeacon = 0;
                  setRobotPos(0.0, 0.0, 0.0);
                  break;
                }
                //lookup table for closed-loop line following behavior:
                switch(groundSensor & 0b01110){
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
                if ( ((groundStable & 0b10001) == 0) && switcher ){
                  // distance the robot travelled while measuring black
                  distToBeacon = x;
                  switcher = false;
                }
                if ( x >= robotRadius - 25.0){  //centered
                  comL    = 0;
                  comR    = 0;
                  setRobotPos(0.0, 0.0, 0.0);
                  updateAvailable = true;
                  nextNode(node, dir);
                  if( (distToBeacon >= 35.0) || (groundStable & 0b10001) ){
                    paths = 0b10000 | flipdir(dir, BACK); //finish line detected
                    led(0,1);
                    led(1,1);
                    movstate = EXPECTINGCOMMAND;
                    break;
                  }else{
                    paths = flipdir(dir, BACK);
                    if (groundStable & 0b00100){
                      paths = paths | dir;      //path leading straight ahead
                    }
                    if (groundLR & 0b10000){
                      paths = paths | flipdir(dir, LEFT);  //path to the left
                    }
                    if (groundLR & 0b00001){
                      paths = paths | flipdir(dir, RIGHT); //path to the right
                    }
                  }
                  if (sumofbits(paths & 0b01111) == 2){
                    movstate = MOVING;            //no intersection, just turn
                    if ( paths & flipdir(dir, LEFT) ){
                      moverel(&dir, LEFT);        //left turn only option
                      setRobotPos(0.0, 0.0, 0.0);
                      printf("left turn detected\n");
                    }else if ( paths & flipdir(dir, RIGHT) ){
                      moverel(&dir, RIGHT);       //right turn only option
                      setRobotPos(0.0, 0.0, 0.0);
                      printf("right turn detected\n");
                    }
                  }else{
                    movstate = EXPECTINGCOMMAND;  //intersection
                    printf("intersection detected\n");
                  }
                }
              break;

              case EXPECTINGCOMMAND:
                comL = 0;
                comR = 0;
              break;
            }
            motorCommand(comR, comL);   //execute motor command with buffer
            //setVel2(comL, comR);      //motor without buffer
            //debug message:
            printf("Dir %d; state: %d; dist: %.0f; node: %d; %d; update: %d;", dir, movstate, x, node[0], node[1], updateAvailable);
            printf(" ground: ");
            printInt(groundSensor, 2 | 5 << 16);
            printf("; grStable: ");
            printInt(groundStable, 2 | 5 << 16);
            printf("; paths: ");
            printInt(paths, 2 | 5 << 16);
            printf("\n");

            if(updateAvailable){
              updateMap(node, paths);
              updateAvailable = false;
            }

            if(movstate == EXPECTINGCOMMAND){
                switch (state) {
                  case EXPLORINGMAP:
                    //algorithm for exploring north (stupid and simple - delete later)
                    if(paths & 0b01000){
                      moveabs(&dir, NORTH);
                      printf("north\n");
                    }else if(paths & 0b00100){
                      moveabs(&dir, EAST);
                      printf("east\n");
                    }else if(paths & 0b0010){
                      moveabs(&dir, SOUTH);
                      printf("south\n");
                    }else{
                      moveabs(&dir, WEST);
                      printf("west\n");
                    }
                  break;
                  case FINDINGBESTPATH:

                  break;
                  case GOINGTOTOKEN:

                  break;
                  case GOINGTOBASE:

                  break;
                }
                movstate = MOVING;
            }


        } while(!stopButton());
        disableObstSens();
        setVel2(0, 0);
    }
    return 0;
}


void updateMap(int *node, int paths){

}

double fmax(double v1, double v2){
  if(v1 > v2){
    return v1;
  }else{
    return v2;
  }
}

int sgn(double v){
  if (v==0.0){
    return 0;
  }else if(v < 0.0){
    return -1;
  }else{
    return 1;
  }
}

int stabledetection(int groundSensor, bool dump){
  const  int l = 3;
  static int sensorBuffer[3] = {4, 4, 4};
  static int state = 0b00100;
  int i, j, curState;
  bool trigger;

  for( i = 0; i < l-1; i++){                //update ground sensor buffer
    if (dump){
      sensorBuffer[i] = 0b00100;
    }else{
      sensorBuffer[i] = sensorBuffer[i+1];
    }
  }
  sensorBuffer[l-1] = groundSensor;
  if (dump){
    state = 0b00100;                        //reset state after dump
  }
  for (i = 0; i < 5; i++){                  //check for stable signal
    curState = (state >> i) & 0b00001;
    trigger = true;
    for (j = 0; j < l; j++) {
      if ( ( (sensorBuffer[j] >> i) & 0b00001 ) == curState ){
        trigger = false;      //all buffer bits have to differ from state bit
      }
    }
    if (trigger){
      state ^= (0b00001 << i);            //toggle sensor bit
    }
  }
  return state;
}

void motorCommand(int comR, int comL){
  const  int len = 3;
  static int motorBuffer[2][3] = {{0}};
  int l, r, i;

  for( i = 0; i < len-1; i++){              //update motor buffer
      motorBuffer[0][i] = motorBuffer[0][i+1];
      motorBuffer[1][i] = motorBuffer[1][i+1];
  }
  motorBuffer[0][len-1] = comR;
  motorBuffer[1][len-1] = comL;
  l = 0;
  r = 0;
  for( i = 0; i < len-1; i++ ){             //calculate mean values of buffer
      r += motorBuffer[0][i]/len;
      l += motorBuffer[1][i]/len;
  }
  setVel2(l, r);                            //apply commands to motor
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
      rotateRel(70,  M_PI);
    break;
    case LEFT:
      rotateRel(70,  M_PI/2);
    break;
  }
  stabledetection(0b00000, true);   //dump sensor buffer to avoid artifacts
  *dir = flipdir(*dir, turn);
  setRobotPos(0.0, 0.0, 0.0);
}

void moveabs(directions *dir, directions absolute){
  int continuous = (*dir) | (*dir<<4);
  if      ( continuous & (absolute<<0) ){
    moverel(dir, FRONT);
  }else if( continuous & (absolute<<1) ){
    moverel(dir, RIGHT);
  }else if( continuous & (absolute<<2) ){
    moverel(dir, BACK);
  }else if( continuous & (absolute<<3) ){
    moverel(dir, LEFT);
  }
}

void rotateRel(int maxVel, double deltaAngle){
    double x, y, t;
    double targetAngle;
    double error;
    double integral = 0;
    double KP_ROT = 30;
    double KI_ROT	= 6;
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
