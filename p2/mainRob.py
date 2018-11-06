
import socket, sys
from croblink import CRobLink
from croblink import CRobLinkAngs

def wander(cif):
     if cif.measures.irSensor[0]> 5.0\
        or cif.measures.irSensor[1]> 5.0\
        or cif.measures.irSensor[2]> 5.0\
        or cif.measures.irSensor[3]> 5.0:
     #    print "Rotate"
         cif.driveMotors(-0.1,+0.1)
     elif cif.measures.irSensor[1]> 0.7:
         cif.driveMotors(0.1,0.0)
     elif cif.measures.irSensor[2]> 0.7:
         cif.driveMotors(0.0,0.1)
     else:
     #    print "Go"
         cif.driveMotors(0.1,0.1)

RUN=1
WAIT=2
RETURN=3


def main(argv):
    pos = 0
    host = "localhost"
    robname = "pClient"

    for i in range(1, len(argv), 2):
        if argv[i] == "--pos":
            pos = int(argv[i+1])
        elif argv[i] == "--host":
            host = argv[i+1]
        elif argv[i] == "--robname":
            robname = argv[i+1]
        else:
            print("Unkown argument", argv[i])
            return 1

    #cif = CRobLink(robname, pos, host)   # use this for default obstacle sensor algular positions
    cif = CRobLinkAngs(robname, pos, [0.0,-60.0,60.0,90.0], host)    # use this to control (through 3rd argument) the angular position of obstacle sensors

    if cif.status!=0:
        print "Connection refused or error"
        quit()

    state = RUN

    while 1:
         #print "READ"
         cif.readSensors()
         #print "IRS "+ str(cif.measures.irSensor[0]) +" " + str(cif.measures.irSensor[1])\
         #         +" "+ str(cif.measures.irSensor[2]) +" "+ str(cif.measures.irSensor[3])
         #print "Drive"
         if state==RUN:
             if cif.measures.visitingLed==1:
                 state=WAIT
             if cif.measures.ground==0:
                 cif.setVisitingLed(1);
             wander(cif)
         elif state==WAIT:
            cif.setReturningLed(1)
            if cif.measures.visitingLed==1:
                cif.setVisitingLed(0)
            if cif.measures.returningLed==1:
                 state=RETURN
            cif.driveMotors(0.0,0.0)
         elif state==RETURN:
            if cif.measures.visitingLed==1:
                cif.setVisitingLed(0)
            if cif.measures.returningLed==1:
                cif.setReturningLed(0)
            wander(cif)
         print cif.measures.irSensor

if __name__ == "__main__":
    main(sys.argv)
