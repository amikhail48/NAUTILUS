
#ifndef MS5803_COLLECTION_H
#define MS5803_COLLECTION_H

#include "ms5803.h"

struct presCollection_args {

	float pres;
	float depth;
	float temp;
};

struct presCollection_args *psArgs;

void *runPresCollection(void *vp);


#endif //MS5803_COLLECTION_H
