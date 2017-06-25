#ifndef LSM6DS3_COLLECTION_H
#define LSM6DS3_COLLECTION_H


#include <stdint.h>
#include <time.h>
#define PI	3.14159265359
struct pCollection_args {
	
	uint8_t BLOCK_READY;
	uint8_t CAL_COMPLETE;
	double x;
	double y;
	double z;
	double xl_x;
	double xl_y;
	double xl_z;
};

extern struct pCollection_args *gArgs;
extern struct pCollection_args *gDeltaArgs;
extern struct CoordinateAxis *OrdinalAxis;
void *runCollection(void *bp);

#endif
