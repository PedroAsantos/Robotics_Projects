#include "rmi-mr32.h"
#include "rmi-mr32.c"
#include <math.h>
#include <string.h>
#include <stdlib.h>

void findTransition(int turning, int highTurn);
void rotateRel(int maxVel, double deltaAngle);
bool detectStartLine(int *counter, int groundSensor, int firstSense, bool reset);
bool checkCircle(int groudSensor);
bool detectFreeRoad(int groundSensorObjectDetected);

int main(void)
{
    int groundSensor = 0;      //stores sensor values
    int groundSensorObjectDetected=0;
    int turning = 0;           //0: robot turns left; 1: robot turns right
    int enableBasicNav = true; //enables basic transition following algorithm
    int leftWheel = 0;         //left wheel speed
    int rightWheel = 0;        //right wheel speed
    int counter = 0;               //number of completed laps
    bool slowZone=false;
    bool circleFound=false;
    bool oldCircleFound=circleFound;

    const int maxSpeedNormal=40;
    const int minSpeedNormal=24;
    const int lowTurnNormal=4;
    const int highTurnNormal=34;

    const int maxSpeedSlowZone=20;
    const int minSpeedSlowZone=12;
    const int lowTurnSlowZone=2;
    const int highTurnSlowZone=16;

    int maxSpeed=maxSpeedNormal;  //maximum speed
    int minSpeed=minSpeedNormal;  //minimum speed
    int lowTurn=lowTurnNormal;
    int highTurn=highTurnNormal;

    int firstSense = 0;        //first value of right ground sensor
    bool finished;     //Has the robot finished?
    int contCycleObjeDect=0;
    int i=0;
    int contSensorRead=0;
    initPIC32();
    closedLoopControl(true);
    setVel2(0, 0);

    printf("Welcome, I am robot %d\n\n\n", ROBOT);

    while(1){
      printf("Press start to continue\n");
      while(!startButton());

      enableObstSens();
      detectStartLine(&counter, groundSensor, firstSense, true);   //reset lap count
      groundSensor = readLineSensors(0);	// Read ground sensor
      firstSense = groundSensor&0x01; //first value of right ground sensor
      finished = false;
    //  printf("=>");
    //  printInt(groundSensor, 2 | 5 << 16);

      do{
        waitTick40ms();						   // Wait for next 40ms tick
        readAnalogSensors();				 // Fill in "analogSensors" structure
        groundSensor = readLineSensors(0);	// Read ground sensor

        //readLineSensors

        printf("Obstacle Sensor=>");
        printf("%03d %03d %03d\n" ,  analogSensors.obstSensLeft, analogSensors.obstSensFront,  analogSensors.obstSensRight);

        if(detectStartLine(&counter, groundSensor, firstSense ,false)){
            switch (counter){     //decides what to do after lap finishes
              case 0:
                // go on
                break;
              case 1:
                // go on
                break;
              case 2:
                rotateRel(highTurn, M_PI);          //rotate 180 degrees
                firstSense = (~firstSense) & 0x01;  //invert firstSense
  ;              break;
              default:
                finished = true;
                break;
            }
        }


        //TODO: LEDs on and change speed when circle detected
        oldCircleFound = circleFound;
        circleFound = checkCircle(groundSensor);

        if(circleFound!=oldCircleFound && circleFound==false){   //check if the values are different and check if it is in the exit of th circle.
          slowZone=!slowZone;
          if(slowZone){
            maxSpeed=maxSpeedSlowZone;
            minSpeed=minSpeedSlowZone;
            lowTurn=lowTurnSlowZone;
            highTurn=highTurnSlowZone;
            for(i = 0;i<=3;i++){
              led(i,1);
            }
          }else{
            maxSpeed=maxSpeedNormal;
            minSpeed=minSpeedNormal;
            lowTurn=lowTurnNormal;
            highTurn=highTurnNormal;
            for(i = 0;i<=3;i++){
              led(i,0);
            }
          }
        }

        //TODO: Obstacle detection and avoidance

        enableBasicNav = detectFreeRoad(groundSensor);


        if(enableBasicNav){           //basic transition finding algorithm
           switch(groundSensor){      //lookup table for behavior
             case 0x18:               //turn slightly left
                leftWheel = maxSpeed - lowTurn;
                rightWheel = maxSpeed + lowTurn;
                //printf("Turning slightly left\n");
                turning = 0; break;
             case 0x07:               //turn slightly left
                leftWheel = maxSpeed - lowTurn;
                rightWheel = maxSpeed + lowTurn;
                //printf("Turning slightly left\n");
                turning = 0; break;
             case 0x03:               //turn slightly right
                leftWheel = maxSpeed + lowTurn;
                rightWheel = maxSpeed - lowTurn;
                //printf("Turning slightly right\n");
                turning = 1; break;
             case 0x1c:               //turn slightly right
                leftWheel = maxSpeed + lowTurn;
                rightWheel = maxSpeed - lowTurn;
                //printf("Turning slightly right\n");
                turning = 1; break;
             case 0x01:               //turn Right
                leftWheel = minSpeed + highTurn;
                rightWheel = minSpeed - highTurn;
                //printf("Turning right\n");
                turning = 1; break;
             case 0x1e:               //turn Right
                leftWheel = minSpeed + highTurn;
                rightWheel = minSpeed - highTurn;
                //printf("Turning right\n");
                turning = 1; break;
             case 0x10:               //turn left
                leftWheel = minSpeed - highTurn;
                rightWheel = minSpeed + highTurn;
                //printf("Turning left\n");
                turning = 0; break;
             case 0x0f:               //turn left
                leftWheel = minSpeed - highTurn;
                rightWheel = minSpeed + highTurn;
                //printf("Turning left\n");
                turning = 0; break;
             case 0x1f:               //stop and Turn
                findTransition(turning,highTurn);
                printf("Uh Oh, I am lost\n");
                break;
             case 0x00:               //stop and Turn
                findTransition(turning,highTurn);
                printf("Uh Oh, I am lost\n");
                break;
             default:                 //keep going straight
                leftWheel = maxSpeed;
                rightWheel = maxSpeed;
                //printf("Going straight ahead\n");
                break;
             }
             setVel2(leftWheel,rightWheel);
          }else{
            groundSensorObjectDetected=groundSensor;
            switch (groundSensor&0x01) {
              case 0x01:
                printf("0x00000000000000000000000000000000000000000001\n");
                contCycleObjeDect=0;
                do{
                  contCycleObjeDect++;
                  rotateRel(highTurn, -0.1-M_PI/2);          //rotate 90 degrees to go inside the circuit

                  //cycle to go straighforawd until not find the object
                  do{
                    contSensorRead=0;
                    for(i=0;i<5;i++){
                       readAnalogSensors();
                       if(analogSensors.obstSensLeft<50){
                         contSensorRead++;
                       }
                     }
                     setVel2(maxSpeedSlowZone,maxSpeedSlowZone);
                     printf("Get far away from the object!\n" );
                     printf("Left Sensor=>");
                       printf("%03d\n" , analogSensors.obstSensLeft);
                  }while(contSensorRead>=3);
                   //check if it is the cycle number one -> if it is the first time we know that at least it is necessary to travel the robot radius
                  if(contCycleObjeDect==1){
                    //delay and speed to go the radius of the robot
                    setVel2(maxSpeedSlowZone,maxSpeedSlowZone);
                    delay(19000);
                   }else{
                    setVel2(maxSpeedSlowZone,maxSpeedSlowZone);
                    delay(5000);
                   }

                //rotate to the direction of the circuit
                rotateRel(highTurn, M_PI/2);
                //end of cycle to verify if there is no object
              }while(!detectFreeRoad(groundSensorObjectDetected));
              //speed and delay to go the diameter of the robot
              setVel2(maxSpeedSlowZone,maxSpeedSlowZone);
              delay(30000);
              //cycle to go until not find the object
              do{
                contSensorRead=0;
                for(i=0;i<5;i++){
                   readAnalogSensors();
                   if(analogSensors.obstSensLeft<50){
                     contSensorRead++;
                   }
                 }
                 setVel2(maxSpeedSlowZone,maxSpeedSlowZone);
                 printf("Get2 far away from the object!\n" );

              }while(contSensorRead>=3);
              //TODO: we need to check if the robot enter several times in the loop.

              //rotate to the circuit 45 degrees
              rotateRel(highTurn, M_PI/4);

              setVel2(maxSpeedSlowZone,maxSpeedSlowZone);
              //go until find white surface
              groundSensor = readLineSensors(0);
              while(groundSensor!=0x00){
                  groundSensor = readLineSensors(0);
                  printf("Serching for 0x00\n");
              }
                setVel2(0,0);
              //rotate until find the path
              groundSensor = readLineSensors(0);
              while((groundSensor&0x01)!=0x01){
                groundSensor = readLineSensors(0);
                printf("Serching for 0x10\n");
                rotateRel(highTurn,-0.25);
              }
                break;
              case 0x00:

                printf("0x00000000000000000000000000000000000000000000\n");
                contCycleObjeDect=0;
                do{
                contCycleObjeDect++;
                rotateRel(highTurn, M_PI/2+0.25);          //rotate 90 degrees to go inside the circuit
                //cycle to go straighforawd until not find the object
                do{
                  contSensorRead=0;
                  for(i=0;i<5;i++){
                     readAnalogSensors();
                     if(analogSensors.obstSensRight<50){
                       contSensorRead++;
                     }
                   }
                   setVel2(maxSpeedSlowZone,maxSpeedSlowZone);
                   printf("Get far away from the object!\n" );
                   printf("Right Sensor=>");
                   printf("%03d\n" , analogSensors.obstSensRight);
                }while(contSensorRead>=3);
                   //check if it is the cycle number one -> if it is the first time we know that at least it is necessary to travel the robot radius
                   if(contCycleObjeDect==1){
                     //delay and speed to go the radius of the robot
                     setVel2(maxSpeedSlowZone,maxSpeedSlowZone);
                     delay(19000);
                   }else{
                     setVel2(maxSpeedSlowZone,maxSpeedSlowZone);
                     delay(5000);
                   }
                //rotate to the direction of the circuit
                rotateRel(highTurn, -M_PI/2);
                //end of cycle to verify if there is no object
              }while(!detectFreeRoad(groundSensorObjectDetected));
              //speed and delay to go the diameter of the robot
              setVel2(maxSpeedSlowZone,maxSpeedSlowZone);
              delay(30000);
              //cycle to go until not find the object
              do{
                contSensorRead=0;
                for(i=0;i<5;i++){
                   readAnalogSensors();
                   if(analogSensors.obstSensRight<50){
                     contSensorRead++;
                   }
                 }
                 setVel2(maxSpeedSlowZone,maxSpeedSlowZone);
                 printf("Get2 far away from the object!\n" );

              }while(contSensorRead>=3);
              //TODO: we need to check if the robot enter several times in the loop.

              //rotate to the circuit 45 degrees
              rotateRel(highTurn, -M_PI/4);

              setVel2(maxSpeedSlowZone,maxSpeedSlowZone);

              groundSensor = readLineSensors(0);
              while(groundSensor!=0x00){
                  groundSensor = readLineSensors(0);
                  printf("Searching for 0x00\n");
              }
                setVel2(0,0);
              groundSensor = readLineSensors(0);
              while((groundSensor&0x10)!=0x10){
                groundSensor = readLineSensors(0);
                printf("Searching for 0x10\n");
                rotateRel(highTurn,0.25);
              }

              break;
            }


          }
        }while( (!stopButton()) && (!finished) );
        //reset robot:
        counter = 0;
        finished = false;
        setVel2(0, 0);
        disableObstSens();
    }
    return 0;
}

bool detectFreeRoad(int groundSensorObjectDetected){
  int cont=0;
  int contR=0;
  int contL=0;
  int i;
  for(i=0;i<20;i++){
      readAnalogSensors();
      if(analogSensors.obstSensFront < 20){
        cont++;
      }
  }

  if((groundSensorObjectDetected&0x01)==0x01){
    printf("checking left\n");

    for(i=0;i<20;i++){
        readAnalogSensors();
        if(analogSensors.obstSensLeft < 20){
          contL++;
        }
    }
    printf("Left Sensor=>");
    printf("%03d\n" , analogSensors.obstSensLeft );
    if(cont>=15 || contL>=15){
      return false;
    }

  }else{
    printf("checking right\n" );
    for(i=0;i<20;i++){
        readAnalogSensors();
        if(analogSensors.obstSensRight < 20){
          contR++;
        }
    }
    printf("Right Sensor=>");
    printf("%03d\n" , analogSensors.obstSensRight );
    if(cont>=15 || contR>=15){
      return false;
    }
  }


  printf("center Sensor=>");
  printf("%03d\n" , analogSensors.obstSensFront );
//  printf("%d\n", obstacleSensor());



  printf("Free Road Ahead!\n" );
  return true;

}

void findTransition(int turning, int highTurn){  //turns the robot until it finds a transition again
    int groundSensor;
    do{
      groundSensor = readLineSensors(0);	// Read ground sensor
        if(turning==0 ){
          setVel2(-highTurn, highTurn);
        }else{
          setVel2(highTurn, -highTurn);
        }
    }while((groundSensor==0x00 || groundSensor==0x1f) && (!stopButton()));
}

bool checkCircle(int groundSensor){

    switch (groundSensor) {
      case 0x19: //11001
      case 0x1D: //11101

        return true;

        break;
      case 0x13: // 10011
      case 0x17: // 10111

        return true;

        break;
      default:
        return false;
        break;
    }


}

bool detectStartLine(int *counter, int groundSensor, int firstSense, bool reset){
    double x, y, t;
    double tolerance = 250;  //tolerance for enabling detection in [mm]
    double distx;           //stores distance to beacon in x-direction
    double disty;           //stores distance to beacon in y-direction
    double dist;            //distance to beacon
    bool withinTol;         //is robot inside tolerance radius?
    bool transition;        //detects black/white transition
    bool detected;          //start line detected?
    static bool locked = true;         //locks/unlocks start line detection
    static double beacon[2] = {0,0};  //Start line beacon

    getRobotPos(&x, &y, &t);
    //resets internal states and returns:
    if(reset){
      locked = true;
      beacon[0] = x;
      beacon[1] = y;
      return false;
    }
    //calculate distance to beacon:
    distx = x - beacon[0];
    disty = y - beacon[1];
    dist = sqrt(distx*distx + disty*disty);
    //only unlock line detection if robot is far away from start line:
    if (locked && dist>800){
        locked = false;
    }
    //example: if firstSense(right sensor) is 1, 'detected' triggers when
    //groundSensor is 1 x x x x 0  (x values don't matter)
    transition = (groundSensor&0x11)  ==  ((firstSense<<4) | ((~firstSense)&0x01));
    withinTol = dist <= tolerance;

    if (*counter < 2){
      //first two rounds: look for transition
      detected = withinTol && transition && !locked;
    }else{
      //third round: just check for tolerance
      detected = withinTol && !locked;
    }
    printf("Distance: %.0f Transition: %d Locked: %d Detected: %d Lap: %d\n", dist, transition, locked, detected, *counter);

    if(detected){
       *counter  += 1;   //one lap was completed. Increase counter
       beacon[0] = x;    //update beacon with new value
       beacon[1] = y;    //update beacon with new value
       locked = true;    //lock detection to prevent multiple detection
       return true;      //return that line was found
    }else{
      return false;
    }
}

void rotateRel(int maxVel, double deltaAngle){
    double x, y, t;
    double targetAngle;
    double error;
    double integral = 0;
    double KP_ROT = 40;
    double KI_ROT	= 5;
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
