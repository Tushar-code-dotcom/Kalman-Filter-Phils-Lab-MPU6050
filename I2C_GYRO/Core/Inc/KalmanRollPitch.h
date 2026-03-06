/*
 * KalmanRollPitch.h
 *
 *  Created on: Mar 4, 2026
 *      Author: Tushar
 */

#ifndef INC_KALMANROLLPITCH_H_
#define INC_KALMANROLLPITCH_H_

#include <math.h>
#include <stdint.h>

#define g ((float) 9.81f)

typedef struct {
	float phi;
	float theta;
	float P[4];
	float Q[2];
	float R[3];
	float gyr[3];
} KalmanRollPitch;

void KalmanRollPitch_Init(KalmanRollPitch *kal, float Pinit, float *Q, float *R);
void KalmanRollPitch_Predict(KalmanRollPitch *kal, float *gyr, float T);
uint8_t KalmanRollPitch_Update(KalmanRollPitch *kal, float *acc	);


#endif /* INC_KALMANROLLPITCH_H_ */
