/*
 * PID.cpp
 *
 *  Created on: Jul 13, 2024
 *      Author: AliakseiShtanhel
 */
#include "PID.hpp"

void PID::SetCoefficients(float Kp, float Ki, float Kd) {
	this->Kp = Kp;
	this->Ki = Ki;
	this->Kd = Kd;
	if(Ki == 0) this->I = 0;
}

void PID::SetCoefficients(float *Kp, float *Ki, float *Kd) {
	this->Kp = *Kp;
	this->Ki = *Ki;
	this->Kd = *Kd;
	if(*Ki == 0) this->I = 0;
}

float PID::Calculate(float setpoint, float feedback) {
	    float E = setpoint - feedback;
	    this->I += E;
	    float P = this->Kp * E;
	    float I = this->Ki * this->I;
	    float D = this->Kd * (E - this->dE);
	    this->dE = E;
	    return P + I + D;
	}
