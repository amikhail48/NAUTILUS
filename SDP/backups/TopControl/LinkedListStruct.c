#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <string.h>
#include "LinkedListStruct.h"
// Node ADT
typedef struct NodeObj{
  char * block;
  char startpoint;
  struct NodeObj* next;
} NodeObj;

typedef NodeObj* Node;

// newNode()
// constructor of the Node type
Node newNode(char* data, char start) {
  //printf("malloc Node\n");
  Node N = malloc(sizeof(NodeObj));
  assert(N!=NULL);
  //printf("malloc block\n");
  N->block = malloc(sizeof(char)*30);
  assert(N->block != NULL);
  memcpy(N->block,data,30);
  //printf("memcpy\n");
  /*int i;
  for (i=0;i<30;i++){
    *((N->block)+sizeof(char)*i) = *(data+sizeof(char)*i);
  }*/
  N->startpoint = start;
  N->next = NULL;
  //printf("done\n");
  return(N);
}

// freeNode()
// destructor for the Node type
void freeNode(Node* pN){
    if( pN!=NULL && *pN!=NULL ){
	free((*pN)->block);
        free(*pN);
        *pN = NULL;
   }
}

// emptyNode()
void emptyNode(Node N){
    if(N != NULL){
    	emptyNode(N->next);
        N->next = NULL;
	//N->block = NULL;
        freeNode(&N);
   }
}


typedef struct LinkedListObj{
  Node head;
  int numItems;
} LinkedListObj;

typedef struct LinkedListObj* LinkedList;

LinkedList newLinkedList(void){
  LinkedList L = (struct LinkedListObj*)malloc(sizeof(LinkedListObj));
  assert(L!=NULL);
  L->head = NULL;
  L->numItems = 0;
  return(L);
}

void emptyList(LinkedList L) {
  emptyNode(L->head);
  L->head=NULL;
  L->numItems=0;
}

void freeLinkedList(LinkedList* pL) {
    if( pL!=NULL && *pL!=NULL ){
    free(*pL);
      *pL = NULL;
    }
}

void addNode(LinkedList L, char* data, char start){
  if (L->head == NULL){
    //printf("head\n");
    L->head = newNode(data,start);
  } else {
    Node N = L->head;
    while (N->next != NULL){
      N=N->next;
    }
    N->next = newNode(data,start);
  }
  //printf("done addnode\n");
  L->numItems++;
}

char popNode(LinkedList L, char *buf) {
	Node N;
	char s;
	
        if (L->numItems>0){
		N = L->head;
		s = N->startpoint;
		memcpy(buf,N->block,30);
		/*for (i=0; i<30; i++){
			*(buf+sizeof(char)*i)= *(N->block+sizeof(char)*i);
		}*/	
		L->head=N->next;
		L->numItems--;
		freeNode(&N);
		return s;
	} else {
		return -1;
	}
	
}

/*
char popBlock(LinkedList L, char * buf) {
	//char * block;
	
	NodeObj N;
	if (L->numItems > 0) {
		N = popNode(L);
		buf = N.block;
		return N.startpoint;
	} else {
		return -1;
	}
}*/
/*void removeNode(LinkedList L, char* k) {
  if(searchList(L, k)){
    Node N = L->head;
    if(strcmp(N->key, k) == 0){
      L->head=N->next;
      freeNode(&N);
    } else {
      for (int i=0; i<(L->numItems-1); i++){
        if (strcmp(N->next->key, k) != 0){
          N=N->next;
        } else {
          break;
        }
      }
      Node M = N->next;
      if (M->next == NULL) {
        N->next = NULL;
      } else {
        N->next = M->next;
        M->next = NULL;
      }
    freeNode(&M);
    }
    L->numItems--;
  }
}*/


