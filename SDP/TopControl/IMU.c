#include "IMU.h"
#include <math.h>
#include "LSM6DS3_Collection.h"

/*double bias = 0.0;
double Q_a = 0.001;
double Q_b = 0.003;
double R_meas = 0.03;*/

struct KalmanFilter *NewKalmanFilter(double angle, double bias){
	struct KalmanFilter *pKalmanFilter;
	pKalmanFilter = malloc(sizeof(struct KalmanFilter));
	pKalmanFilter->angle = angle;
	pKalmanFilter->gy_bias = bias;
	pKalmanFilter->P[0][0] = 0.0;
	pKalmanFilter->P[0][1] = 0.0;
	pKalmanFilter->P[1][0] = 0.0;
	pKalmanFilter->P[1][1] = 0.0;
	pKalmanFilter->Q_a = 0.001;
	pKalmanFilter->Q_b = 0.003;
	pKalmanFilter->R_meas=0.03;	
	return pKalmanFilter;
}

double getAngleKalman(struct KalmanFilter *ThisAxis, double thisAngle, double thisRate, double deltaT){
	
	double rate;
       	rate = thisRate - ThisAxis->gy_bias;
        ThisAxis->angle += deltaT * rate;
	
	ThisAxis->P[0][0] += deltaT * (deltaT*ThisAxis->P[1][1] - ThisAxis->P[0][1] - ThisAxis->P[1][0] + ThisAxis->Q_a);
	ThisAxis->P[0][1] -= deltaT * ThisAxis->P[1][1];
	ThisAxis->P[1][0] -= deltaT * ThisAxis->P[1][1];
	ThisAxis->P[1][1] += ThisAxis->Q_b * deltaT;
	double s = ThisAxis->P[0][0] + ThisAxis->R_meas;
	double K[2] = { (ThisAxis->P[0][0] / s), (ThisAxis->P[1][0] / s)};


	double y = thisAngle - ThisAxis->angle;
	ThisAxis->angle += K[0] * y;
	ThisAxis->gy_bias += K[1] * y;

	double P00_temp = ThisAxis->P[0][0];
	double P01_temp = ThisAxis->P[0][1];
	ThisAxis->P[0][0] -= K[0] * P00_temp;
	ThisAxis->P[0][1] -= K[0] * P01_temp;
	ThisAxis->P[1][0] -= K[1] * P00_temp;
	ThisAxis->P[1][1] -= K[1] * P01_temp;

	return ThisAxis->angle;
}

void rotateBtoI(struct pCollection_args angle, struct CoordinateAxis *axis){

	struct CoordinateAxis temp_axis = *axis;
	angle.x = -angle.x;
	//angle.y = -angle.y;
	//angle.z = -angle.z;
	/*angle.x = round(angle.x);
	angle.y = round(angle.y);
	angle.z = round(angle.z);*/
	double cosx=cos(angle.x*M_PI/180.0);
	double sinx=sin(angle.x*M_PI/180.0);
	double cosy=cos(angle.y*M_PI/180.0);
	double siny=sin(angle.y*M_PI/180.0);
	double cosz=cos(angle.z*M_PI/180.0);
	double sinz=sin(angle.z*M_PI/180.0);
	
	
	/*double cosx=cos(angle.x);
	double sinx=sin(angle.x);
	double cosy=cos(angle.y);
	double siny=sin(angle.y);
	double cosz=cos(angle.z);
	double sinz=sin(angle.z);*/
	
	double coszcosx = cosz * cosx;
	double sinzcosx = sinz * cosx;
	double coszsinx = sinx * cosz;
	double sinzsinx = sinx * sinz;

	double mat[3][3];
	mat[0][0] = cosz * cosy;
	mat[1][0] = -cosy * sinz;
	mat[2][0] = siny;
	mat[0][1] = sinzcosx + (coszsinx * siny);
	mat[1][1] = coszcosx - (sinzsinx * siny);
	mat[2][1] = -sinx * cosy;
	mat[0][2] = (sinzsinx) - (coszcosx * siny);
	mat[1][2] = (coszsinx) + (sinzcosx * siny);
	mat[2][2] = cosy * cosx;

	axis->X = temp_axis.X * mat[0][0] + temp_axis.Y * mat[1][0] + temp_axis.Z * mat[2][0];
	axis->Y = temp_axis.X * mat[0][1] + temp_axis.Y * mat[1][1] + temp_axis.Z * mat[2][1];
	axis->Z = temp_axis.X * mat[0][2] + temp_axis.Y * mat[1][2] + temp_axis.Z * mat[2][2];
	
}
void updateXLCoordinateAxis(struct pCollection_args angle, struct CoordinateAxis *axis){
	
	
	//printf("in math\n");	
	struct CoordinateAxis temp_axis = *axis;
	//angle.x = -angle.x;
	//angle.y = -angle.y;
	//angle.z = -angle.z;
	/*angle.x = round(angle.x);
	angle.y = round(angle.y);
	angle.z = round(angle.z);*/
	double cosx=cos(angle.x*M_PI/180.0);
	double sinx=sin(angle.x*M_PI/180.0);
	double cosy=cos(angle.y*M_PI/180.0);
	double siny=sin(angle.y*M_PI/180.0);
	double cosz=cos(angle.z*M_PI/180.0);
	double sinz=sin(angle.z*M_PI/180.0);
	
	
	/*double cosx=cos(angle.x);
	double sinx=sin(angle.x);
	double cosy=cos(angle.y);
	double siny=sin(angle.y);
	double cosz=cos(angle.z);
	double sinz=sin(angle.z);*/
	
	double coszcosx = cosz * cosx;
	double sinzcosx = sinz * cosx;
	double coszsinx = sinx * cosz;
	double sinzsinx = sinx * sinz;

	double mat[3][3];
	mat[0][0] = cosz * cosy;
	mat[0][1] = -cosy * sinz;
	mat[0][2] = siny;
	mat[1][0] = sinzcosx + (coszsinx * siny);
	mat[1][1] = coszcosx - (sinzsinx * siny);
	mat[1][2] = -sinx * cosy;
	mat[2][0] = (sinzsinx) - (coszcosx * siny);
	mat[2][1] = (coszsinx) + (sinzcosx * siny);
	mat[2][2] = cosy * cosx;

	axis->X = temp_axis.X * mat[0][0] + temp_axis.Y * mat[1][0] + temp_axis.Z * mat[2][0];
	axis->Y = temp_axis.X * mat[0][1] + temp_axis.Y * mat[1][1] + temp_axis.Z * mat[2][1];
	axis->Z = temp_axis.X * mat[0][2] + temp_axis.Y * mat[1][2] + temp_axis.Z * mat[2][2];
	
	//printf("axis-X: %0.10lf\n",axis->X);

	/*axis->X = round(axis->X*1000)/1000.0;
	axis->Y = round(axis->Y*1000)/1000.0;
	axis->Z = round(axis->Z*1000)/1000.0;*/
	//	printf("difference: %4.3f", (axis->X-temp_axis.X));
}
