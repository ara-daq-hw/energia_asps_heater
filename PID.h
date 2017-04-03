/*
 * PID.h
 *
 *  Created on: Mar 25, 2016
 *      Author: barawn
 */

#ifndef PID_H_
#define PID_H_

#include <limits.h>

class PID {
public:
	PID() : _Kp(0), _Ki(0), _Kd(0), _ITerm(0), _maxOut(SHRT_MAX), _minOut(SHRT_MIN), _lastInput(0) {}
	PID(double Kp, double Ki, double Kd) : _ITerm(0), _maxOut(SHRT_MAX), _minOut(SHRT_MIN), _lastInput(0) {
		SetTuningParameters(Kp, Ki, Kd);
		_ITerm = 0;
	}
	inline void SetTuningParametersRaw(long Kp, long Ki, long Kd) {
		_Kp = Kp;
		_Ki = Ki;
		_Kd = Kd;
	}
	inline void SetTuningParameters(double Kp, double Ki, double Kd) {
		_Kp = (long) (Kp*256.);
		_Ki = (long) (Ki*256.);
		_Kd = (long) (Kd*256.);
	}
	void Compute();
	inline void SetOutputLimits(int min, int max) {
		_minOut = min;
		_maxOut = max;
	}
	/** \brief Resets the PID.
	 *
	 * Reset() restores the PID to initial operation, eliminating the _ITerm
	 * history, and getting rid of the lastInput value. This can be called in the beginning
	 * to eliminate any I or D term contribution to the first computation.
	 *
	 * lastInput's value is left up to the user, but usually the smart option is to
	 * measure the input, and feed it to Reset(). That way the D-term will start off small.
	 */
	inline void Reset(int lastInput = 0) {
		_ITerm = 0;
		_lastInput = lastInput;
	}

	inline long ITerm() {
		return _ITerm;
	}

	int input;
	int output;
	int setpoint;
private:
	// Scaled Kp. This is 256 times Kp.
	long _Kp;
	// Scaled Ki. This is 256 times Ki.
	long _Ki;
	// Scaled Kd. This is 256 times Kd.
	long _Kd;
	// Scaled I-term.
	long _ITerm;
	// Max.
	int _maxOut;
	// Min.
	int _minOut;
	// Last input.
	int _lastInput;
};


#endif /* PID_H_ */
