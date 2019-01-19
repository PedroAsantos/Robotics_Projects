#include <string.h>
#include <stdlib.h>
//#include "vector.h"


typedef struct nodeMap
{
    int coor_x;
    int coor_y;
    int paths[4];
    int ggCost;
    int hCost;
    bool visited;
    struct Node *parent;
} Node;


typedef struct {
  Node **array;
  size_t used;
  size_t size;
} Array;

typedef struct {
  int intArray[19*19][2];
  size_t used;
  size_t size;
} ArrayUnknownCoord;

void initArray(Array *a, size_t initialSize) {
//  a->array = (Node *) malloc(initialSize * sizeof(Node));
  /*printf("teste\n");
  a->array = (Node *) malloc(initialSize * sizeof(Node));
  printf("malloc array %d\n", a->array);
  a->used = 0;
  a->size = initialSize;*/

}

void insertArray(Array *a, Node *element) {
  // a->used is the number of used entries, because a->array[a->used++] updates a->used only *after* the array has been accessed.
  // Therefore a->used can go up to a->size
  /*if (a->used == a->size) {
    a->size *= 2;
    a->array = (Node *)realloc(a->array, a->size * sizeof(Node));
  }*/
  a->array[a->used++] = element;
}

void insertArrayInt(ArrayUnknownCoord *a, int *element) {
//  printf("insertArrayInt() (%d,%d) \n", element[0],element[1]);
  // a->used is the number of used entries, because a->array[a->used++] updates a->used only *after* the array has been accessed.
  // Therefore a->used can go up to a->size
  /*if (a->used == a->size) {
    a->size *= 2;
    a->array = (Node *)realloc(a->array, a->size * sizeof(Node));
  }*/
  a->intArray[a->used][0] = element[0];
  a->intArray[a->used][1] = element[1];
  a->used++;
}


bool removeElementFromArrayInt(ArrayUnknownCoord *a, int *elementToRemove){
  int i;

  for(i=0;i<a->used;i++){
    if(a->intArray[i][0] == elementToRemove[0] && a->intArray[i][1] == elementToRemove[1]){
      for ( ; i < a->used - 1; i++)
      {
        // Assign the next element to current location.
        a->intArray[i][0] = a->intArray[i + 1][0];
        a->intArray[i][1] = a->intArray[i + 1][1];
      }
    //  a->array[a->used-1]= ;
      a->used = a-> used - 1;
    }
  }

  return false;
}

bool removeElementFromArray(Array *a, Node *elementToRemove){
  int i;

  for(i=0;i<a->used;i++){
    if(a->array[i] == elementToRemove){
      for ( ; i < a->used - 1; i++)
      {
        // Assign the next element to current location.
        a->array[i] = a->array[i + 1];
      }
    //  a->array[a->used-1]= ;
      a->used = a-> used - 1;
    }
  }
  return false;
}

bool elementIsInArray(Array *a, Node *elementToCheck){
  int i;

  for(i=0;i<a->used;i++){
    if(a->array[i] == elementToCheck){
      return true;
    }
  }

  return false;

}

bool elementIsInArrayInt(ArrayUnknownCoord *a, int *elementToCheck){
//  printf("elementIsInArrayInt() \n");
  int i;
//  printf("a->used %d\n",(int) a->used);
  for(i=0;i<a->used;i++){
    if(a->intArray[i][0] == elementToCheck[0] && a->intArray[i][1] == elementToCheck[1]){
      return true;
    }
  }

  return false;

}




void freeArray(Array *a) {
  free(a->array);
  a->array = NULL;
  a->used = a->size = 0;
}
