#include "rmi-mr32.h"
#include "rmi-mr32.c"
#include <math.h>
#include <string.h>
#include <stdlib.h>



enum states { EXPLORINGMAP, FINDINGBESTPATH, GOINGTOTOKEN, GOINGTOBASE };


void rotateRel_basic(int speed, double deltaAngle);
void rotateRel(int maxVel, double deltaAngle);
void updateMap();

int main(void)
{
    int groundSensor;

    initPIC32();
    closedLoopControl( true );
    setVel2(0, 0);
    //initial state
    enum states state = EXPLORINGMAP;
    bool updateAvailable=false;
    bool isIntersection=false;

    printf("RMI-example, robot %d\n\n\n", ROBOT);

    while(1)
    {
        printf("Press start to continue\n");
        while(!startButton());
        enableObstSens();

        rotateRel(100, M_PI / 2);
        //      setVel2(70, 70);
        do
        {
            waitTick40ms();						// Wait for next 40ms tick
            readAnalogSensors();				// Fill in "analogSensors" structure
            groundSensor = readLineSensors(0);	// Read ground sensor
            printf("Obst_left=%03d, Obst_center=%03d, Obst_right=%03d, Bat_voltage=%03d, Ground_sens=",
                    analogSensors.obstSensLeft,
                    analogSensors.obstSensFront,
                    analogSensors.obstSensRight,
                    analogSensors.batteryVoltage);

            printInt(groundSensor, 2 | 5 << 16);	// System call
            printf("\n");


            

            if(updateAvailable){
              updateMap();
              updateAvailable=false;
            }
            if(isIntersection){
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
        rotateRel(100, -M_PI / 2);
        //      setVel2(0, 0);
    }
    return 0;
}



void updateMap(){

}


#define KP_ROT	40
#define KI_ROT	5

// deltaAngle in radians
void rotateRel(int maxVel, double deltaAngle)
{
    double x, y, t;
    double targetAngle;
    double error;
    double integral = 0;
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


void rotateRel_basic(int speed, double deltaAngle)
{
    double x, y, t;
    double targetAngle;
    double error;
    int cmdVel, errorSignOri;

    getRobotPos(&x, &y, &t);
    targetAngle = normalizeAngle(t + deltaAngle);
    error = normalizeAngle(targetAngle - t);
    errorSignOri = error < 0 ? -1 : 1;

    cmdVel = error < 0 ? -speed : speed;
    setVel2(-cmdVel, cmdVel);

    do
    {
        getRobotPos(&x, &y, &t);
        error = normalizeAngle(targetAngle - t);
    } while (fabs(error) > 0.01 && errorSignOri * error > 0);
    setVel2(0, 0);
}
