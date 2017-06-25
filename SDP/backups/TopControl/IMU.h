#ifndef IMU_H
#define IMU_H

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include "LSM6DS3_Collection.h"

struct CoordinateAxis {

	double X;
	double Y;
	double Z;

};


struct AbsolutePositionStruct {
	double roll;
	double pitch;
	double yaw;
	double depth;
	double accX;
	double accY;
	double accZ;
	double vX;
	double vY;
	double vZ;
};

struct KalmanFilter {
	double angle;
	double rate;
	double gy_bias;
	double P[2][2];
	double Q_a;
	double Q_b;
	double R_meas;
};
struct KalmanFilter *NewKalmanFilter(double angle, double bias);
double getAngleKalman( struct KalmanFilter *ThisAxis, double thisAngle, double thisRate, double deltaT );
void rotateBtoI(struct pCollection_args angle, struct CoordinateAxis *axis);
void updateXLCoordinateAxis(struct pCollection_args angle, struct CoordinateAxis *axis);


#endif
