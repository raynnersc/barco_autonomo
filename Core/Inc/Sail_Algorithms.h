/*
 * Sail_Algorithms.h
 *
 *  Created on: 14 de dez de 2022
 *      Author: danil
 */

#ifndef INC_SAIL_ALGORITHMS_H_
#define INC_SAIL_ALGORITHMS_H_

#include <math.h>
#include <stdint.h>
#include "QMC5883.h"

#define PI 3.141592654
#define EARTH_FIELD_REFERENCE 0.233804
#define MAGNETIC_DECLINATION -22.92
#define PROPORTIONAL_GAIN_SERVO 0.155
#define SERVO_MAX_ANGLE 40
#define PROPORTIONAL_GAIN_DC 0.1
#define DC_MAX_DC 100
#define DC_MIN_DC 50
#define UTM_B1_X 608628.6896362832
#define UTM_B1_Y 7802939.849127788
#define UTM_B2_X 608592.6445984794
#define UTM_B2_Y 7802849.039444146
#define UTM_B3_X 608629.1525047406
#define UTM_B3_Y 7802938.496024627
#define BEACON_POWER -69

typedef struct{
	float x;
	float y;
	float d;
	float RSSI;
	float power;
} Beacon;

float BeaconDistance(float power, float rssi);
void BoatPosition(Beacon *b1, Beacon *b2, Beacon *b3, float *x, float *y);
float AngleFromGeoNorth(QMC_t *qmc);
float TargetAngleFromGeoNorth(Beacon *target_beacon, float x_boat, float y_boat);
float RudderAngle(float boat_angle, float target_angle);
uint8_t DCMotorDutyCycle(float boat_angle, float target_angle);

#endif /* INC_SAIL_ALGORITHMS_H_ */
