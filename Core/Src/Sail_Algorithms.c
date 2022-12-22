/*
 * Sail_Algorithms.c
 *
 *  Created on: 14 de dez de 2022
 *      Author: danil
 */

#include "Sail_Algorithms.h"


float BeaconDistance(float power, float rssi){
	float d;
	d = pow(10,((power-rssi)/20));
	return d;
}

void BoatPosition(Beacon *b1, Beacon *b2, Beacon *b3, float *x, float *y){

	float a;
	float b;
	float c;
	float d;
	float e;
	float f;

	a = (-2*(b1->x) + 2*(b2->x));
	b = (-2*(b1->y) + 2*(b2->y));
	c = pow((b1->d),2) - pow((b2->d),2) - pow((b1->x),2) + pow((b2->x),2) - pow((b1->y),2) + pow((b2->y),2);
	d = (-2*(b2->x) + 2*(b3->x));
	e = (-2*(b2->y) + 2*(b3->y));
	f = pow((b2->d),2) - pow((b3->d),2) - pow((b2->x),2) + pow((b3->x),2) - pow((b2->y),2) + pow((b3->y),2);

	*x = (c*e - f*b)/(e*a - b*d);
	*y = (c*d - a*f)/(b*d - a*e);
}

float AngleFromGeoNorth(QMC_t *qmc){

	if(qmc->compas + MAGNETIC_DECLINATION < -180){
		return (360 + (qmc->compas + MAGNETIC_DECLINATION));
	}
	else if(qmc->compas + MAGNETIC_DECLINATION > 180){
		return ((qmc->compas + MAGNETIC_DECLINATION)-360);
	}

	return (qmc->compas + MAGNETIC_DECLINATION);
}

float TargetAngleFromGeoNorth(Beacon *target_beacon, float x_boat, float y_boat){
	float del_y = target_beacon->y - y_boat;
	float del_x = target_beacon->x - x_boat;

	float target_rad = atan(del_y/del_x);
	int8_t n = target_rad/(2*PI);
	target_rad -= n*2*PI;

	return target_rad*180/PI;
}

float RudderAngle(float boat_angle, float target_angle){
	float error_angle = target_angle - boat_angle;
	float rudder_angle = PROPORTIONAL_GAIN_SERVO*error_angle;

	if(rudder_angle > SERVO_MAX_ANGLE) rudder_angle = SERVO_MAX_ANGLE;
	if(rudder_angle < -SERVO_MAX_ANGLE) rudder_angle = -SERVO_MAX_ANGLE;

	return rudder_angle;
}

uint8_t DCMotorDutyCycle(float boat_angle, float target_angle){
	float error_angle = target_angle - boat_angle;
	if(error_angle<0) error_angle = -error_angle;
	uint8_t dc;
	dc = 100 - (PROPORTIONAL_GAIN_DC*error_angle);

	if(dc > DC_MAX_DC) dc = DC_MAX_DC;
	if(dc < DC_MIN_DC) dc = DC_MIN_DC;

	return dc;
}






