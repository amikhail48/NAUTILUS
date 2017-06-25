//-----------------------------------------------------------------------------
// Dictionary.h
// Header file for the Dictionary ADT
//-----------------------------------------------------------------------------

#ifndef _LINKLIST_H_INCLUDE_
#define _LINKLIST_H_INCLUDE_


// Dictionary
// Exported reference type
typedef struct DictionaryObj* Dictionary;
typedef struct LinkedListObj* LinkedList;
typedef struct NodeObj* Node;

// newDictionary()
// constructor for the Dictionary type
LinkedList newLinkedList(void);

// freeDictionary()
// destructor for the Dictionary type
void freeLinkedList(LinkedList* pD);
void emptyList(LinkedList L);
// isEmpty()
// returns 1 (true) if S is empty, 0 (false) otherwise
// pre: none
//int isEmpty(Dictionary D);

// size()
// returns the number of (key, value) pairs in D
// pre: none
//int size(Dictionary D);

// lookup()
// returns the value v such that (k, v) is in D, or returns NULL if no 
// such value v exists.
// pre: none
//char* lookup(Dictionary D, char* k);

// insert()
// inserts new (key,value) pair into D
// pre: lookup(D, k)==NULL
//void insert(Dictionary D, char* data);

// delete()
// deletes pair with the key k
// pre: lookup(D, k)!=NULL
//void delete(Dictionary D, char* k);

// makeEmpty()
// re-sets D to the empty state.
// pre: none
//void makeEmpty(Dictionary D);
void addNode(LinkedList L, char* data,char start);

char popNode(LinkedList L, char* buf);
//char blockBlock(LinkedList L, char * buf);
// printDictionary()
// pre: none
// prints a text representation of D to the file pointed to by out
//void printDictionary(FILE* out, Dictionary D);

#endif
