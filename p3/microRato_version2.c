#include "rmi-mr32.h"
#include "rmi-mr32.c"
#include "bluetooth_comm.h"
#include "bluetooth_comm.c"
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include "array.c"


// TODO: draw state machine, measure robot radius





typedef enum { STARTING, EXPLORINGMAP, FINDINGBESTPATH, GOINGTOTOKEN, GOINGTOBASE } states;
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





const int INITIAL_SIZE_X_MAP = 37;
const int INITIAL_SIZE_Y_MAP = 37;
int SIZE_X_MAP = 37;
int SIZE_Y_MAP = 37;
// why error using struct  nodeMap map[INITIAL_SIZE_Y_MAP][INITIAL_SIZE_X_MAP];
Node map[37][37];
ArrayUnknownCoord unknownNodesHistory;
Array aStarPath;
Array closedSet;
bool performingAStar=false;
int     cheeseCoord [2]           = {0};
bool closedNodeMode = true;
Node *lastNodeToGo;
int baseNodeIndex[2] = {18,18};
void    rotateRel(int maxVel, double deltaAngle);
void    motorCommand(int comR, int comL);
int     stabledetection(int groundSensor, bool dump);
int     flipdir( directions dir, relative turn);
void    nextNode(int *node, directions dir);
void    updateMapPaths(int *node, int paths);
void    updateMap(int *node, int paths);
int     sumofbits(int number);
void    moverel(directions *dir, relative turn);
void    moveabs(directions *dir, directions absolute);
double  fmax(double v1, double v2);
int     sgn(double v);

void initializeMap();
void getNodeMapIndex(int* currentNodeCoord, int* index);
void updateMapSize(int* currentNodeCoord);
directions getDirectionToExploreMap(int* currentNode, directions currentDirection);
void performAStar(Node *initialNode, Node *finalNode, bool toFollow);
int GetManhattanDistance(Node *nodeA, Node *nodeB);
void saveAStartPath(Node *head,bool toFollow);
void updateUnknownNodesHistory(int *nodeCoord);
directions followAStar(int* currentNodeCoord);
bool checkIfBestPathIsAvailable();
directions getDirectionToFindBestPath(int* currentCoord, bool previousStateFinding);
void cleanAStar();
void copyArrayInt(int* array_source,int* array_dest ,int size);
bool checkIfNodeHasUnknownNeighbours(Node *node);
directions getDirectionToGoToToken(int* currentCoord);
directions getDirectionToGoToInitialPoint(int* currentCoord, states previousState);

int main(void)
{
    initPIC32();
//    configBTUart(3, 115200); // Configure Bluetooth UART
//    bt_on();   // enable bluetooth; ////printf is now redirected to bluetooth UART
    closedLoopControl( true );
    setVel2(0, 0);
    ////printf("MicroRato, robot %d\n\n\n", ROBOT);
    initializeMap();

    unknownNodesHistory.used = 0;



    while(1)
    {
        ////printf("Press start to continue\n");
        while(!startButton());
        //Initialize default state
        states        state        = EXPLORINGMAP;
        states        previousState = STARTING;
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
                if ( (x  >= d) && (groundStable & 0b00100) ){//straight node detected
                  updateAvailable = true;
                  paths = dir | flipdir(dir, BACK);     //update available paths
                  nextNode(node, dir);
                  setRobotPos(0.0, 0.0, 0.0);   //set beacon to current position
                  movstate = EXPECTINGCOMMAND;

                  ////printf("ACCPETING COMMAND ###############################\n" );
                  movstate = EXPECTINGCOMMAND;

                }
                if ( (groundStable == 0) && ( x  >= d ) ){ //dead end detected
                  updateAvailable = true;
                  paths = flipdir(dir, BACK);           //only path leads back
                  nextNode(node, dir);                  //update node and beacon
                  setRobotPos(0.0, 0.0, 0.0);
                  movstate = EXPECTINGCOMMAND;
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
                    movstate = EXPECTINGCOMMAND;  //intersection
                }
              break;

              case EXPECTINGCOMMAND:
                comL = 0;
                comR = 0;
              break;
            }
            motorCommand(comR, comL);   //execute motor command with buffer
            //setVel2(comL, comR);      //motor without buffer

            if(updateAvailable){
              printf("Dir %d; state: %d; dist: %.0f; node: %d; %d;", dir, movstate, x, node[0], node[1]);
              printf(" ground: ");
              printInt(groundSensor, 2 | 5 << 16);
              printf("; grStable: ");
              printInt(groundStable, 2 | 5 << 16);
              printf("; paths: ");
              printInt(paths, 2 | 5 << 16);
              printf("\n");

              updateAvailable = false;
              //update historyPathNode
              //check if node has more than one unknown neighbour. if yes, than add to the array.

              //set visited current node
        //      int nodeIndex[2] = {0};
        //      getNodeMapIndex(node,nodeIndex);
        //      map[nodeIndex[0]][nodeIndex[1]].visited = true;
            }

            if(movstate == EXPECTINGCOMMAND){
              if(state==EXPLORINGMAP){
                updateMap(node, paths);
                updateUnknownNodesHistory(node);
                printf("EXPLORING MAP\n");
              //  ////printf("####################### %d\n",getDirectionToExploreMap(node,NORTH));
                // check if it found the cheese
                if((paths & 0b10000) == 0b10000){
                  printf("#################### CHEESE FOUND ###########################\n" );
                  cheeseCoord[0] = node[0];
                  cheeseCoord[1] = node[1];
                  if(checkIfBestPathIsAvailable()){
                    //printf("Best path available\n");
                    state=GOINGTOBASE;
                  }else{
                    previousState= EXPLORINGMAP;
                    state=FINDINGBESTPATH;
                  }
                }else{
                  moveabs(&dir,getDirectionToExploreMap(node,NORTH));
                }
              }
              if(state==FINDINGBESTPATH){
                updateMap(node, paths);
                printf("FINDING BEST PATH\n");
                bool previousStateFinding;
                ////printf("Exploring MAP = %d\n", EXPLORINGMAP);
                if(previousState == EXPLORINGMAP){
                  previousStateFinding = false;
                }else{
                  previousStateFinding = true;
                }

                if( previousStateFinding && !performingAStar && checkIfBestPathIsAvailable()){
                  state=GOINGTOTOKEN;
                }else{
                    moveabs(&dir,getDirectionToFindBestPath(node,previousStateFinding));
                }
                previousState = FINDINGBESTPATH;
              }
              if(state==GOINGTOTOKEN){
                printf("GOING TO TOKEN\n");
                if(node[0]==cheeseCoord[0] && node[1]==cheeseCoord[1]){
                  state= GOINGTOBASE;
                  previousState = GOINGTOTOKEN;
                }else{
                  moveabs(&dir,getDirectionToGoToToken(node));
                }
              }
              if(state==GOINGTOBASE){
                printf("GOING TO BASE\n");
                if(node[0]==0 && node[1]==0){
                  ////printf("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& END &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&\n");
                  setVel2(0, 0);
                  for(i=0;i<SIZE_Y_MAP;i++){
                    for(j=0;j<SIZE_X_MAP;j++){
                        printf("(%d,", map[i][j].coor_x);
                        printf("%d,", map[i][j].coor_y);
                        for(c=0;c<4;c++){
                          printf("%d,", map[i][j].paths[c]);
                        }
                        printf(")");
                     }
                  }
                  printf("\n");
                  printf("BestPath->");
                  for(i=0;i<aStarPath.size;i++){
                    printf("(%d,%d) ",aStarPath.array[i]);
                  }
                  while(true){

                  }
                }else{
                  moveabs(&dir,getDirectionToGoToInitialPoint(node,previousState));
                }
                previousState = GOINGTOBASE;
              }

                movstate = MOVING;
            }


        } while(!stopButton());
        ////printf("###########################MAP######################################\n");

        disableObstSens();
        setVel2(0, 0);
    }
    return 0;
}

// function to return direciton when exploring map
directions getDirectionToGoToInitialPoint(int* currentCoord, states previousState){
  //printf("getDirectionToGoToInitialPoint\n" );
  if(previousState == GOINGTOBASE){
    if(performingAStar){
      return followAStar(currentCoord);
    }else{
      int nodeIndex[2] = {0};
      int nodePoinIndex[2] ={0};
      int initialPoint[2] = {0};
      getNodeMapIndex(initialPoint,nodePoinIndex);
      getNodeMapIndex(currentCoord,nodeIndex);
      Node *currentNode = &map[nodeIndex[0]][nodeIndex[1]];
      Node *initialNode = &map[nodePoinIndex[0]][nodePoinIndex[1]];


      performAStar(currentNode,initialNode,true);

      if(performingAStar){
          return followAStar(currentCoord);
      }
    }
  }else{
    int nodeIndex[2] = {0};
    getNodeMapIndex(currentCoord, nodeIndex);

    int c;
    for(c=0;c<4;c++){
      if(map[nodeIndex[0]][nodeIndex[1]].paths[c]==1){
        if(c==0){
          ////printf("Going NORTH\n");
          return NORTH;
        }else if(c==1){
          ////printf("Going EAST\n");
          return EAST;
        }else if(c==2){
          ////printf("Going SOUTH\n");
          return SOUTH;
        }else if(c==3){
          ////printf("Going WEST\n");
          return WEST;
        }
      }
    }
  }
  ////printf("Finding getDirectionToGoToInitialPoint path return null\n" );
  //hard coded invert of the current dir.
  return -1;
}

//function to return direction when is going to token
directions getDirectionToGoToToken(int* currentCoord){
  //printf("getDirectionToGoToToken\n");
  if(performingAStar){
    return followAStar(currentCoord);
  }else{
    int nodeIndex[2] = {0};
    int nodeTokenIndex[2] ={0};

    getNodeMapIndex(cheeseCoord,nodeTokenIndex);
    getNodeMapIndex(currentCoord,nodeIndex);
    Node *currentNode = &map[nodeIndex[0]][nodeIndex[1]];
    Node *tokenNode = &map[nodeTokenIndex[0]][nodeTokenIndex[1]];


    performAStar(currentNode,tokenNode,true);

    if(performingAStar){
        return followAStar(currentCoord);
    }
  }
  ////printf("Direction to token not found\n");
  return -1;
}
//function to return the direction of an unkwnon neighbour
directions getDirectionToUnknownNeighbor(int* currentCoord){
  //printf("getDirectionToUnknownNeighbor() \n");
  int neighbourdNode[2] = {0};
  int neighbourIndex[2] = {0};
  int nodeIndex[2] = {0};
  getNodeMapIndex(currentCoord,nodeIndex);
  int c;
  for(c=0;c<4;c++){
    if(map[nodeIndex[0]][nodeIndex[1]].paths[c]==1){
        ////printf("c=%d\n",c);
        copyArrayInt(currentCoord,neighbourdNode,sizeof(neighbourdNode)/sizeof(neighbourdNode[0]));
        if(c==0){
            neighbourdNode[0]=neighbourdNode[0]+1;
        }else if(c==1){
          neighbourdNode[1]=neighbourdNode[1]-1;
        }else if(c==2){
          neighbourdNode[0]=neighbourdNode[0]-1;
        }else if(c==3){
          neighbourdNode[1]=neighbourdNode[1]+1;
        }

        //neighbourIndex =
        getNodeMapIndex(neighbourdNode,neighbourIndex);
        int cc;
        for(cc=0;cc<4;cc++){
        //  ////printf("map[neighbourIndex[0]][neighbourIndex[1]].paths[%d] = %d\n ", cc, map[neighbourIndex[0]][neighbourIndex[1]].paths[cc]);
          if(map[neighbourIndex[0]][neighbourIndex[1]].paths[cc]==-1){
        //    ////printf("neightbour of current Node test (%d,%d): \n", map[neighbourIndex[0]][neighbourIndex[1]].coor_x, map[neighbourIndex[0]][neighbourIndex[1]].coor_y);
            if(c==0){
              lastNodeToGo=&map[neighbourIndex[0]][neighbourIndex[1]];
              //printf("Going NORTH\n");
              return NORTH;
            }else if(c==1){
              lastNodeToGo=&map[neighbourIndex[0]][neighbourIndex[1]];
              //printf("Going EAST\n");
              return EAST;
            }else if(c==2){
              lastNodeToGo=&map[neighbourIndex[0]][neighbourIndex[1]];
              //printf("Going SOUTH\n");
              return SOUTH;
            }else if(c==3){
              lastNodeToGo=&map[neighbourIndex[0]][neighbourIndex[1]];
              //printf("Going WEST\n");
              return WEST;
            }
          }
      }
    }
  }
  ////printf("getDirectionToUnknownNeighbor() returnin null");
  return 1;
}
//function to return direction when it is finding the best path
directions getDirectionToFindBestPath(int* currentCoord, bool previousStateFinding){
  //printf("getDirectionToFindBestPath()\n" );
  ////printf("previousStateFinding %d\n", previousStateFinding);
  if(previousStateFinding){
    if(performingAStar){
      return followAStar(currentCoord);
    }else{
      int nodeIndex[2] = {0};
      getNodeMapIndex(currentCoord,nodeIndex);
      Node *currentNode = &map[nodeIndex[0]][nodeIndex[1]];

      if(!checkIfNodeHasUnknownNeighbours(currentNode) || closedNodeMode){
        int initialNodeCoord[2] = {0};
        int initialNodeIndex[2] = {0};
        int cheeseNodeIndex[2] = {0};
        getNodeMapIndex(initialNodeCoord,initialNodeIndex);
        getNodeMapIndex(cheeseCoord,cheeseNodeIndex);
        // Node *nodeCheese = &map[cheeseNodeIndex[0]][cheeseNodeIndex[1]];
        // Node *initial = &map[initialNodeIndex[0]][initialNodeIndex[1]];

        //performAStar(nodeCheese,initial,false);
        int i;
        int minhCost=1000;
        int minFcost=1000;
        int indexOfNode=0;
        bool searchMoreLastNode=false;
        int fcostArray;
        printf("ln %d,%d", lastNodeToGo->coor_x, lastNodeToGo->coor_y);
        for(i=0;i<closedSet.used;i++){
          if(checkIfNodeHasUnknownNeighbours(closedSet.array[i])){
            fcostArray = closedSet.array[i]->hCost + closedSet.array[i]->ggCost;
            printf("Unknown neigh:  %d,%d = %d - %d\n",closedSet.array[i]->coor_x, closedSet.array[i]->coor_y, closedSet.array[i]->hCost, closedSet.array[i]->ggCost);
            if((fcostArray < minFcost) || ((fcostArray == minFcost) && (closedSet.array[i]->hCost < minhCost)) ){
              minhCost=closedSet.array[i]->hCost;
              minFcost=fcostArray;
              indexOfNode=i;
            }
            if(lastNodeToGo == closedSet.array[i]){
              printf("searchMoreLastNode\n");
              searchMoreLastNode = true;
            }
          }
        }
        int nodeIndex[2] = {0};
        getNodeMapIndex(currentCoord, nodeIndex);
        Node *currentNode = &map[nodeIndex[0]][nodeIndex[1]];

        if( searchMoreLastNode || (currentNode->coor_x == closedSet.array[indexOfNode]->coor_x && currentNode->coor_y == closedSet.array[indexOfNode]->coor_y)){
          return getDirectionToUnknownNeighbor(currentCoord);
        }

        performAStar(currentNode,closedSet.array[indexOfNode],true);
        // closedSet.array[indexOfNode];
        if(performingAStar){
            closedNodeMode=false;
            return followAStar(currentCoord);
        }
      }else{
        closedNodeMode=true;
        int neighbourdNode[2] = {0};
        int neighbourIndex[2] = {0};
        int c;
        for(c=0;c<4;c++){
          if(map[nodeIndex[0]][nodeIndex[1]].paths[c]==1){
              //printf("c=%d\n",c);
              copyArrayInt(currentCoord,neighbourdNode,sizeof(neighbourdNode)/sizeof(neighbourdNode[0]));
              if(c==0){
                  neighbourdNode[0]=neighbourdNode[0]+1;
              }else if(c==1){
                neighbourdNode[1]=neighbourdNode[1]-1;
              }else if(c==2){
                neighbourdNode[0]=neighbourdNode[0]-1;
              }else if(c==3){
                neighbourdNode[1]=neighbourdNode[1]+1;
              }

              //neighbourIndex =
              getNodeMapIndex(neighbourdNode,neighbourIndex);
              int cc;
              for(cc=0;cc<4;cc++){
              //  ////printf("map[neighbourIndex[0]][neighbourIndex[1]].paths[%d] = %d\n ", cc, map[neighbourIndex[0]][neighbourIndex[1]].paths[cc]);
                if(map[neighbourIndex[0]][neighbourIndex[1]].paths[cc]==-1){
                  lastNodeToGo = &map[neighbourIndex[0]][neighbourIndex[1]];
              //    ////printf("neightbour of current Node test (%d,%d): \n", map[neighbourIndex[0]][neighbourIndex[1]].coor_x, map[neighbourIndex[0]][neighbourIndex[1]].coor_y);
                  if(c==0){
                    lastNodeToGo = &map[neighbourIndex[0]][neighbourIndex[1]];
        //            //printf("Going NORTH\n");
                    return NORTH;
                  }else if(c==1){
                    lastNodeToGo = &map[neighbourIndex[0]][neighbourIndex[1]];
        //            //printf("Going EAST\n");
                    return EAST;
                  }else if(c==2){
                    lastNodeToGo = &map[neighbourIndex[0]][neighbourIndex[1]];
        //            //printf("Going SOUTH\n");
                    return SOUTH;
                  }else if(c==3){
                    lastNodeToGo = &map[neighbourIndex[0]][neighbourIndex[1]];
        //            //printf("Going WEST\n");
                    return WEST;
                  }
                }
            }
        }
      }

     }
   }
 }

 int nodeIndex[2] = {0};
 getNodeMapIndex(currentCoord, nodeIndex);
 lastNodeToGo = &map[nodeIndex[0]][nodeIndex[1]];
 int c;
 for(c=0;c<4;c++){
   if(map[nodeIndex[0]][nodeIndex[1]].paths[c]==1){
     if(c==0){
       //printf("Going NORTH\n");
       return NORTH;
     }else if(c==1){
       //printf("Going EAST\n");
       return EAST;
     }else if(c==2){
       //printf("Going SOUTH\n");
       return SOUTH;
     }else if(c==3){
       //printf("Going WEST\n");
       return WEST;
     }
   }
 }
  ////printf("Finding best path return null\n" );
  //hard coded invert of the current dir.
  return -1;

}
// function to check if the best path is available
bool checkIfBestPathIsAvailable(){
  //printf(" checkIfBestPathIsAvailable() \n");
  int initialNodeCoord[2] = {0};
  int initialNodeIndex[2] = {0};
  int cheeseNodeIndex[2] = {0};
  getNodeMapIndex(initialNodeCoord,initialNodeIndex);
  getNodeMapIndex(cheeseCoord,cheeseNodeIndex);
  Node *nodeCheese = &map[cheeseNodeIndex[0]][cheeseNodeIndex[1]];
  Node *initial = &map[initialNodeIndex[0]][initialNodeIndex[1]];
  performAStar(nodeCheese,initial,false);

//  ////printf("aStarPath.size  = %d,  GetManhattanDistance(initial,nodeCheese) %d\n",(int)aStarPath.size,GetManhattanDistance(initial,nodeCheese));
  if(aStarPath.size == GetManhattanDistance(initial,nodeCheese)){
    return true;
  }else{
      int i;
      for(i=0;i<closedSet.used;i++){
        if(checkIfNodeHasUnknownNeighbours(closedSet.array[i])){
          return false;
        }
      }
  }

  return true;

}
//function to verify if the node has argument has unknwon nieghbours
bool checkIfNodeHasUnknownNeighbours(Node *node){
  int neighbourdNode[2] = {0};
  int neighbourIndex[2] = {0};
  int nodeCoord[2] = {0};
  nodeCoord[0] = node->coor_x;
  nodeCoord[1] = node->coor_y;

  int c;
  for(c=0;c<4;c++){
      if(node->paths[c]==1){
        copyArrayInt(nodeCoord,neighbourdNode,sizeof(neighbourIndex)/sizeof(neighbourIndex [0]));
        if(c==0){
            neighbourdNode[0]=node->coor_x+1;
        }else if(c==1){
          neighbourdNode[1]=node->coor_y-1;
        }else if(c==2){
          neighbourdNode[0]=node->coor_x-1;
        }else if(c==3){
          neighbourdNode[1]=node->coor_y+1;
        }

        getNodeMapIndex(neighbourdNode,neighbourIndex);
        int cc;
        Node *neighbour = &map[neighbourIndex[0]][neighbourIndex[1]];

        for(cc=0;cc<4;cc++){
          if(neighbour->paths[cc]== -1){
             return true;
          }
        }

      }
    }
    return false;
}

//function to copy array of ints
void copyArrayInt(int* array_source,int* array_dest ,int size){
    int i;
    for(i=0;i<size;i++){
      array_dest[i]=array_source[i];
    }

}
//function to follow A*
directions followAStar(int* currentNodeCoord){
//    //printf("followAStar() \n");
//  ////printf("aStarPath.used = %d\n",aStarPath.used);
//  //printf("array[used-1] = (%d, %d)\n", aStarPath.array[aStarPath.used-1]->coor_x,aStarPath.array[aStarPath.used-1]->coor_y);
  Node *nextNode = aStarPath.array[aStarPath.used-1];
  removeElementFromArray(&aStarPath, nextNode);
  if(aStarPath.used==0){
    performingAStar=false;
  }

  int diff[2] = {0};
  diff[0]= nextNode->coor_x-currentNodeCoord[0];
  diff[1]= nextNode->coor_y-currentNodeCoord[1];
  if(abs(diff[0])>0){
    if(diff[0]>0){
      ////printf("NORTH\n");
      return NORTH;
    }else{
      ////printf("SOUTH\n");
      return SOUTH;
    }
  }else{
    if(diff[1]>0){
      ////printf("WEST\n");
      return WEST;
    }else{
      ////printf("EAST\n");
      return EAST;
    }
  }


}
//fucntion that returns the direction when exploring map
directions getDirectionToExploreMap(int* currentNodeCoord, directions currentDirection){

  if(performingAStar){
    return followAStar(currentNodeCoord);
  }else{
    int nodeIndex[2] = {0};
    getNodeMapIndex(currentNodeCoord,nodeIndex);
    int neighbourdNode[2] = {0};
    int neighbourIndex[2] = {0};

    int c;
    int sumofpaths=0;
    for(c=0;c<4;c++){
      if(map[nodeIndex[0]][nodeIndex[1]].paths[c]==1){
        copyArrayInt(currentNodeCoord,neighbourdNode,sizeof(neighbourdNode)/sizeof(neighbourdNode[0]));
        if(c==0){
            neighbourdNode[0]=neighbourdNode[0]+1;
        }else if(c==1){
          neighbourdNode[1]=neighbourdNode[1]-1;
        }else if(c==2){
          neighbourdNode[0]=neighbourdNode[0]-1;
        }else if(c==3){
          neighbourdNode[1]=neighbourdNode[1]+1;
        }

        getNodeMapIndex(neighbourdNode,neighbourIndex);
        sumofpaths=0;
        int cc;
        if(map[neighbourIndex[0]][neighbourIndex[1]].visited==false){
          for(cc=0;cc<4;cc++){
        //    ////printf("map[neighbourIndex[0]][neighbourIndex[1]].paths[%d] = %d\n ", cc, map[neighbourIndex[0]][neighbourIndex[1]].paths[cc]);
            if(map[neighbourIndex[0]][neighbourIndex[1]].paths[cc]==-1){
              sumofpaths=-10;
            }
            if(sumofpaths!=-10){
              sumofpaths+=map[neighbourIndex[0]][neighbourIndex[1]].paths[cc];
            }
          }

          if(sumofpaths==1){
            if(c==0){
              //printf("Going NORTH\n");
              return NORTH;
            }else if(c==1){
              //printf("Going EAST\n");
              return EAST;
            }else if(c==2){
              //printf("Going SOUTH\n");
              return SOUTH;
            }else if(c==3){
              //printf("Going WEST\n");
              return WEST;
            }
          }
        }
      }
    }


    for(c=0;c<4;c++){
      if(map[nodeIndex[0]][nodeIndex[1]].paths[c]==1){
  //        ////printf("c=%d\n",c);
          copyArrayInt(currentNodeCoord,neighbourdNode,sizeof(neighbourdNode)/sizeof(neighbourdNode[0]));
          if(c==0){
              neighbourdNode[0]=neighbourdNode[0]+1;
          }else if(c==1){
            neighbourdNode[1]=neighbourdNode[1]-1;
          }else if(c==2){
            neighbourdNode[0]=neighbourdNode[0]-1;
          }else if(c==3){
            neighbourdNode[1]=neighbourdNode[1]+1;
          }

          //neighbourIndex =
          getNodeMapIndex(neighbourdNode,neighbourIndex);
          int cc;
          for(cc=0;cc<4;cc++){
        //    ////printf("map[neighbourIndex[0]][neighbourIndex[1]].paths[%d] = %d\n ", cc, map[neighbourIndex[0]][neighbourIndex[1]].paths[cc]);
            if(map[neighbourIndex[0]][neighbourIndex[1]].paths[cc]==-1){
        //  ////printf("neightbour of current Node test (%d,%d): \n", map[neighbourIndex[0]][neighbourIndex[1]].coor_x, map[neighbourIndex[0]][neighbourIndex[1]].coor_y);
              if(c==0){
//                //printf("Going NORTH\n");
                return NORTH;
              }else if(c==1){
//                //printf("Going EAST\n");
                return EAST;
              }else if(c==2){
//                //printf("Going SOUTH\n");
                return SOUTH;
              }else if(c==3){
//                //printf("Going WEST\n");
                return WEST;
              }
            }
          }
      }
    }
    ////printf("Explorating MAP  A* ? \n");
  // necessary to create code to go to the nearest unbknown node
  //usar lista de memoria e depois fazer a* para o node da lista mais recente desconhecido
    ////printf("Size of unknoen nodes histoyry %d\n",(int)unknownNodesHistory.used);
    if(unknownNodesHistory.used>0){  //what to do if it is equal to 0 -- is it possible to be????
//      ////printf("Unknown Nodes history !=0 \n");
      Node *currentNode = &map[nodeIndex[0]][nodeIndex[1]];
      /*int i;
      for(i=0;i<unknownNodesHistory.used;i++){
        ////printf("unknownNodesHistory[i]=(%d,%d)\n",unknownNodesHistory.array[i]->coor_x,unknownNodesHistory.array[i]->coor_y );
      }*/
      int nodeTargetIndex[2] = {0};
      getNodeMapIndex(unknownNodesHistory.intArray[unknownNodesHistory.used-1],nodeTargetIndex);
      Node *targetNode = &map[nodeTargetIndex[0]][nodeTargetIndex[1]];
      performAStar(currentNode,targetNode,true);
      if(performingAStar){
        return followAStar(currentNodeCoord);
      }

    }
    ////printf("Explorating map return null\n");
    return -1;
  }
}



//function to initialize map
void initializeMap(){
  int i, j, c;
  const int PATHS_SIZE= 4;
  for(i=0; i<INITIAL_SIZE_Y_MAP; i++) {
      for(j=0;j<INITIAL_SIZE_X_MAP;j++) {
        map[i][j].coor_x = j-INITIAL_SIZE_X_MAP/2;
        map[i][j].coor_y = INITIAL_SIZE_Y_MAP/2-i;
        map[i][j].ggCost = 0 ;
        map[i][j].hCost = 0;
        map[i][j].visited = false;
        for(c=0;c<PATHS_SIZE;c++){
            map[i][j].paths[c] = -1;
        }
      }
   }
  /* int nodeIndex[2] = {0};
  // int initialCoord[2] = {0};

   getNodeMapIndex(initialCoord,nodeIndex);
   map[nodeIndex[0]][nodeIndex[1]].visited = true;*/
}
//function that receives a pointer to an array with the coordinates and it will write in the pointer of the second argument the index
void getNodeMapIndex(int* currentNodeCoord, int* index){
  //static int index[2] = {0};
  index[0] = currentNodeCoord[1]*(-1)+baseNodeIndex[0];
  index[1] = currentNodeCoord[0]+baseNodeIndex[1];
//  return index;
}
//function to update map
void updateMap(int *node, int paths){
  updateMapPaths(node, paths);
  updateMapSize(node);
}
//function to calculate f cost of node
int getFcostOfNode(Node *node){
  return node->ggCost + node->hCost;
}

//function to write in array of nodes the kwnon neighbours of a node
void getKnownNeighborsToArray(Array *a, Node *node){
  int neighbourdNode[2] = {0};
  int nodeCoord[] = {node->coor_x, node->coor_y};
  int neighbourIndex[2] = {0};

  bool knownNeighbours=false;
  int c;
  for(c=0;c<4;c++){
      if(node->paths[c]==1){
        copyArrayInt(nodeCoord,neighbourdNode,sizeof(neighbourIndex)/sizeof(neighbourIndex [0]));
        if(c==0){
            neighbourdNode[0]=node->coor_x+1;
        }else if(c==1){
          neighbourdNode[1]=node->coor_y-1;
        }else if(c==2){
          neighbourdNode[0]=node->coor_x-1;
        }else if(c==3){
          neighbourdNode[1]=node->coor_y+1;
        }

        getNodeMapIndex(neighbourdNode,neighbourIndex);
        int cc;
        Node *neighbour = &map[neighbourIndex[0]][neighbourIndex[1]];
        knownNeighbours=true;
        for(cc=0;cc<4;cc++){
          if(neighbour->paths[cc]== -1){
             knownNeighbours = false;
             break;
          }
        }
        if(knownNeighbours){
          insertArray(a, neighbour);
        }
      }
  }
}

//function to update the array with unknwon neighbours
void updateUnknownNodesHistory(int *nodeCoord){
  ////printf("updateUnknownNodesHistory \n");
  int neighbourdNode[2] = {0};
  int neighbourIndex[2] = {0};
  int contUnknownneighbours = 0;
  int nodeIndex[2] = {0};
  getNodeMapIndex(nodeCoord,nodeIndex);
  Node *currentNode = &map[nodeIndex[0]][nodeIndex[1]];

  int c;
  for(c=0;c<4;c++){
      if(currentNode->paths[c]==1){
        copyArrayInt(nodeCoord,neighbourdNode,sizeof(neighbourIndex)/sizeof(neighbourIndex [0]));
        if(c==0){
            neighbourdNode[0]=currentNode->coor_x+1;
        }else if(c==1){
          neighbourdNode[1]=currentNode->coor_y-1;
        }else if(c==2){
          neighbourdNode[0]=currentNode->coor_x-1;
        }else if(c==3){
          neighbourdNode[1]=currentNode->coor_y+1;
        }

        getNodeMapIndex(neighbourdNode,neighbourIndex);
        int cc;
        Node *neighbour = &map[neighbourIndex[0]][neighbourIndex[1]];

        for(cc=0;cc<4;cc++){
          if(neighbour->paths[cc]== -1){
             contUnknownneighbours++;
             break;
          }
        }

      }
  }
  if(contUnknownneighbours>1){
    if(elementIsInArrayInt(&unknownNodesHistory,nodeCoord)){
      removeElementFromArrayInt(&unknownNodesHistory,nodeCoord);
    }
    insertArrayInt(&unknownNodesHistory, nodeCoord);
    ////printf("added element to unknown nodes list (%d,%d)",currentNode->coor_x, currentNode->coor_y);
    ////printf(" unknownNodesHistory.used= %d \n", (int) unknownNodesHistory.used);
  }else{
    if(elementIsInArrayInt(&unknownNodesHistory,nodeCoord)){
      removeElementFromArrayInt(&unknownNodesHistory,nodeCoord);
      ////printf("Removed element to unknown nodes list. unknownNodesHistory.used= %d \n", (int) unknownNodesHistory.used);
    }
  }

}

//function that returns manhantan distance
int GetManhattanDistance(Node *nodeA, Node *nodeB){
  return abs(nodeA->coor_x-nodeB->coor_x)+abs(nodeA->coor_y-nodeB->coor_y);
}

//function to perform A star
void performAStar(Node *initialNode, Node *finalNode, bool toFollow){
  printf("performAStar (%d,%d)->(%d,%d)\n",initialNode->coor_x,initialNode->coor_y,finalNode->coor_x, finalNode->coor_y);
  cleanAStar();
  Array openSet;
  openSet.used = 0;
  Node *nodesOpenSet[(INITIAL_SIZE_X_MAP/2+1) * (INITIAL_SIZE_Y_MAP/2+1)];
  openSet.array = nodesOpenSet;

  Node *nodesClosedSet[(INITIAL_SIZE_X_MAP/2+1) * (INITIAL_SIZE_Y_MAP/2+1)];
  closedSet.used = 0;
  closedSet.array = nodesClosedSet;

  insertArray(&openSet, initialNode);

  ////printf("%d, %d\n", openSet.array[0]->coor_x, openSet.array[0]->coor_y );
  while(openSet.used>0){
//    ////printf("WHILE BEGIN\n");
    Node *currentNode = openSet.array[0];

    int fCostCurrentNode = 0;
    fCostCurrentNode = currentNode->ggCost + currentNode->hCost;

//    ////printf("f cost current Node: %d\n", fCostCurrentNode);
//    ////printf("gcost: %d, hcost: %d", currentNode->ggCost, currentNode->hCost);
    int i;
    int fCostArray=0;
    for(i=0;i<openSet.used;i++){
      fCostArray = openSet.array[i]->ggCost + openSet.array[i]->hCost;
  //    ////printf("(%d,%d)- F cost: %d\n",openSet.array[i]->coor_x,openSet.array[i]->coor_y , fCostArray);
      if((fCostArray < fCostCurrentNode) || (fCostArray == fCostCurrentNode && openSet.array[i]->hCost < currentNode->hCost )){
        currentNode = openSet.array[i];
        fCostCurrentNode = fCostArray;
      }
    }
//    ////printf("current node : %d,%d,%d\n",currentNode->coor_x, currentNode->coor_y, fCostCurrentNode);

    removeElementFromArray(&openSet,currentNode);
    insertArray(&closedSet, currentNode);

    if(currentNode->coor_x == finalNode->coor_x && currentNode->coor_y == finalNode->coor_y){
       ////printf("TARGET REACH\n");
//      ////printf("%d, %d \n", currentNode->coor_x, currentNode->coor_y );
      saveAStartPath(currentNode, toFollow);
      return;
    }

    Array neighboursOfCurrentNode;
    Node *nodesNeighbours[4];
    neighboursOfCurrentNode.used = 0;
    neighboursOfCurrentNode.array = nodesNeighbours;

    getKnownNeighborsToArray(&neighboursOfCurrentNode, currentNode);

    int n;
    for(n=0;n<neighboursOfCurrentNode.used;n++){
//      ////printf("neigh: %d,%d\n",neighboursOfCurrentNode.array[n]->coor_x, neighboursOfCurrentNode.array[n]->coor_y);
      if(!elementIsInArray(&closedSet,neighboursOfCurrentNode.array[n])){
        int newMovementCostToNeigh = currentNode->ggCost + 1;
        if(newMovementCostToNeigh < neighboursOfCurrentNode.array[n]->ggCost || !elementIsInArray(&openSet,neighboursOfCurrentNode.array[n])){
          neighboursOfCurrentNode.array[n]->ggCost = newMovementCostToNeigh;
          neighboursOfCurrentNode.array[n]->hCost = GetManhattanDistance(neighboursOfCurrentNode.array[n],finalNode);
      //    ////printf("add neigh : %d, %d \n", neighboursOfCurrentNode.array[n]->coor_x, neighboursOfCurrentNode.array[n]->coor_y);

          (neighboursOfCurrentNode.array[n])->parent = currentNode;  //verificar se está bem.

          if(!elementIsInArray(&openSet,neighboursOfCurrentNode.array[n])){
            insertArray(&openSet,neighboursOfCurrentNode.array[n]);
          }
        }
      }
    }

  }

}
//function to save path a start
void saveAStartPath(Node *head, bool toFollow) {
//   //printf("save A Start\n" );
   Node *aStarNodes[(INITIAL_SIZE_X_MAP/2+1) * (INITIAL_SIZE_Y_MAP/2+1)];
   aStarPath.array=aStarNodes;
   aStarPath.used=0;
   //used to count the number of points
   aStarPath.size=0;
   Node *current_node = head;
   Node *previousNode = NULL;
   	while ( current_node != NULL) {
        if(current_node->parent != NULL){
        //  ////printf("(%d, %d) ",current_node->coor_x, current_node->coor_y );
          aStarPath.size++;
          insertArray(&aStarPath, current_node);
      //    ////printf("\n");

        }
        current_node = current_node->parent;
        previousNode = current_node;
    }

    if(aStarPath.used==0 || toFollow==false){
      performingAStar = false;
    }else{
      performingAStar = true;
    }

}
//fucntion to clean a start
void cleanAStar(){
  //clean A Start Nodes

  int i, j;
  for(i=0; i<INITIAL_SIZE_Y_MAP; i++) {
      for(j=0;j<INITIAL_SIZE_X_MAP;j++) {
        map[i][j].ggCost = 0 ;
        map[i][j].hCost = 0;
        map[i][j].parent = NULL;
      }
   }
}

//function to update map paths
void updateMapPaths(int *node, int paths){
  int nodeIndex[2] = {0};
  getNodeMapIndex(node,nodeIndex);
  int neighbourdNode[2] = {0};

  int c;
  for(c=0;c<4;c++){
    map[nodeIndex[0]][nodeIndex[1]].paths[c]=0;
  }
  map[nodeIndex[0]][nodeIndex[1]].visited = true;
  int nodeIndexNeighbour[2] = {0};

  //West
  copyArrayInt(node,neighbourdNode,sizeof(nodeIndex)/sizeof(nodeIndex[0]));
  neighbourdNode[1]=neighbourdNode[1]+1;
  getNodeMapIndex(neighbourdNode,nodeIndexNeighbour);
  if((paths & 0b0001) == 0b0001){
    map[nodeIndex[0]][nodeIndex[1]].paths[3]=1;
    map[nodeIndexNeighbour[0]][nodeIndexNeighbour[1]].paths[1]=1;
  }else{
    map[nodeIndexNeighbour[0]][nodeIndexNeighbour[1]].paths[1]=0;
  }
  //South
  copyArrayInt(node,neighbourdNode,sizeof(nodeIndex)/sizeof(nodeIndex[0]));
  neighbourdNode[0]=neighbourdNode[0]-1;
  getNodeMapIndex(neighbourdNode,nodeIndexNeighbour);
  if((paths & 0b0010) == 0b0010){
    map[nodeIndex[0]][nodeIndex[1]].paths[2]=1;
    map[nodeIndexNeighbour[0]][nodeIndexNeighbour[1]].paths[0]=1;
  }else{
    map[nodeIndexNeighbour[0]][nodeIndexNeighbour[1]].paths[0]=0;
  }
  //EAST
  copyArrayInt(node,neighbourdNode,sizeof(nodeIndex)/sizeof(nodeIndex[0]));
  neighbourdNode[1]=neighbourdNode[1]-1;
  getNodeMapIndex(neighbourdNode,nodeIndexNeighbour);
  if((paths & 0b0100) == 0b0100){
    map[nodeIndex[0]][nodeIndex[1]].paths[1]=1;
    map[nodeIndexNeighbour[0]][nodeIndexNeighbour[1]].paths[3]=1;
  }else{
    map[nodeIndexNeighbour[0]][nodeIndexNeighbour[1]].paths[3]=0;
  }
  //north
  copyArrayInt(node,neighbourdNode,sizeof(nodeIndex)/sizeof(nodeIndex[0]));
  neighbourdNode[0]=neighbourdNode[0]+1;
  getNodeMapIndex(neighbourdNode,nodeIndexNeighbour);
  if((paths & 0b1000) == 0b1000){
    map[nodeIndex[0]][nodeIndex[1]].paths[0]=1;
    map[nodeIndexNeighbour[0]][nodeIndexNeighbour[1]].paths[2]=1;
  }else{
    map[nodeIndexNeighbour[0]][nodeIndexNeighbour[1]].paths[2]=0;
  }

}
//function to update map size
void updateMapSize(int* currentNodeCoord){
    if(currentNodeCoord[0]>0){
      if(INITIAL_SIZE_X_MAP/2 + map[0][0].coor_x < currentNodeCoord[0]){
        baseNodeIndex[1] = baseNodeIndex[1] - 1; //not sure
        SIZE_X_MAP=SIZE_X_MAP-1;
        int j,i;
        for(i=0;i<SIZE_Y_MAP;i++){
          for(j=0;j<SIZE_X_MAP;j++){
            map[i][j] = map[i][j+1];
          }
        }
      }
    }else{
      if(INITIAL_SIZE_X_MAP/2 - map[0][SIZE_X_MAP-1].coor_x < -1*currentNodeCoord[0]){
        SIZE_X_MAP=SIZE_X_MAP-1;
      }
    }

    if(currentNodeCoord[1]>0){
      if(INITIAL_SIZE_Y_MAP/2 + map[SIZE_Y_MAP-1][0].coor_y < currentNodeCoord[1]){
        SIZE_Y_MAP=SIZE_Y_MAP-1;
      }
    }else{
      if(INITIAL_SIZE_Y_MAP/2 - map[0][0].coor_y < -1*currentNodeCoord[1]){
        baseNodeIndex[0] = baseNodeIndex[0] - 1; //not sure
        SIZE_Y_MAP=SIZE_Y_MAP-1;
        int j,i;
        for(i=0;i<SIZE_Y_MAP;i++){
          for(j=0;j<SIZE_X_MAP;j++){
            map[i][j] = map[i+1][j];
          }
        }
      }
    }

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
