#include <string.h>
#include <stdlib.h>
//#include "vector.h"


typedef struct nodeMap
{
    int coor_x;
    int coor_y;
    int paths[4];
    int gCost;
    int hCost;
    //struct Node *parent;
    int indexArrayParent;
} Node;


typedef struct {
  Node *array;
  size_t used;
  size_t size;
} Array;

void initArray(Array *a, size_t initialSize) {
//  a->array = (Node *) malloc(initialSize * sizeof(Node));
  printf("teste\n");
  a->array = (Node *) malloc(initialSize * sizeof(Node));
  printf("malloc array %d\n", a->array);
  a->used = 0;
  a->size = initialSize;

}

void insertArray(Array *a, Node element) {
  // a->used is the number of used entries, because a->array[a->used++] updates a->used only *after* the array has been accessed.
  // Therefore a->used can go up to a->size
  /*if (a->used == a->size) {
    a->size *= 2;
    a->array = (Node *)realloc(a->array, a->size * sizeof(Node));
  }*/
  a->array[a->used++] = element;
}
bool removeElementFromArray(Array *a, Node elementToRemove){
  int i;

  for(i=0;i<a->used;i++){
    if(a->array[i].coor_x == elementToRemove.coor_x && a->array[i].coor_y == elementToRemove.coor_y){
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

bool elementIsInArray(Array *a, Node elementToCheck){
  int i;

  for(i=0;i<a->used;i++){
    if(a->array[i].coor_x == elementToCheck.coor_x && a->array[i].coor_y == elementToCheck.coor_y){
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
