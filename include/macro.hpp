#pragma once
#include "main.h"


inline pros::MotorGroup liftMotors({4, -2}, pros::MotorGearset::red); // lift motor group - ports 4, 2 (reversed)
inline pros::Rotation liftRot(19);
inline int liftStates[2] = {105, 120};
// [0] = bottom / starting pos (used by R2 homing)
// [1] = slightly above ground, after picking up an object
inline int currState = 0;

inline double kP = 2.5;
inline double kI = 0;
inline double kD = 0;

inline double maxLiftSpeedUp = 100; // TODO: tune this - top speed (out of 127) moving up
inline double maxLiftSpeedDown = 60; // TODO: tune this - top speed (out of 127) moving down, lower since gravity already helps
inline double liftSlewRate = 4; // TODO: tune this - max change in output per 10ms tick, so it ramps up instead of snapping
inline double minLiftOutput = 20; // TODO: tune this - minimum output (out of 127) applied whenever error is outside tolerance, so small errors still generate enough torque to overcome friction and actually move

inline void liftControl(double target) {
    double integral = 0;
    double derivative = 0;
    double prevOutput = 0;

	double error = target - liftRot.get_position() / 100.0;
	double prevError = error;

	double timer = 0;

	while (timer < 800) {
    	error = target - liftRot.get_position() / 100.0;
        integral += error;
        derivative = error - prevError;

    	double output = error * kP + integral * kI + derivative * kD;

    	if (fabs(error) >= 1) {
    		if (output > 0 && output < minLiftOutput) output = minLiftOutput;
    		if (output < 0 && output > -minLiftOutput) output = -minLiftOutput;
    	}

    	if (output > maxLiftSpeedUp) output = maxLiftSpeedUp;
    	if (output < -maxLiftSpeedDown) output = -maxLiftSpeedDown;

    	if (output > prevOutput + liftSlewRate) output = prevOutput + liftSlewRate;
    	if (output < prevOutput - liftSlewRate) output = prevOutput - liftSlewRate;

    	liftMotors.move(output);
    	prevOutput = output;

		if (fabs(error) < 1) {
			break;
		}

		timer += 10;
        prevError = error;
		pros::delay(10);
	}
	liftMotors.brake();
}