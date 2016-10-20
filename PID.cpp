/*
 * PID.cpp
 *
 *  Created on: Mar 25, 2016
 *      Author: barawn
 */
#include "PID.h"

void PID::Compute() {
	// This is a pure-integer implementation of a PID loop, with scaled
	// Kp/Ki/Kd. The scaling is 256, so that means if we do
	// traditional (Setpoint-Input)*Kp, what we get is the P contribution
	// to Output, times 256.
	static long computedOutput;

	computedOutput = (setpoint - input)*_Kp;
	_ITerm = _ITerm + (setpoint - input);
	computedOutput += (_ITerm)*_Ki;
	computedOutput += (input - _lastInput)*_Kd;
	// Downshift by 8.
	computedOutput >>= 8;
	// Check if it falls out of bounds.
	if (computedOutput > _maxOut) computedOutput = _maxOut;
	if (computedOutput < _minOut) computedOutput = _minOut;
	output = computedOutput;
}

