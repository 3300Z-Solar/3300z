#pragma once
#include "main.h"
#include "lemlib/api.hpp"
#include <algorithm>
#include <cmath>
#include <vector>


inline pros::MotorGroup liftMotors({9, -1}, pros::MotorGearset::green); // lift motor group - ports 4, 2 (reversed)

// lift preset positions in motor encoder degrees
// flip SHOW_LIFT_POS_DEBUG to true in main.cpp to re-measure these
inline int liftStates[6] = {0, -200, 160, 143, 125, 206};
// [0] = bottom / starting pos (used by R2 homing)
// [1] = slightly above ground, after picking up an object

// lemlib PID driving the lift - output is millivolts, clamped to +/-12000
inline lemlib::PID liftPid(500, // kP
                           0, // kI
                           1000, // kD
                           0, // integral anti windup range (deg)
                           false); // reset integral when sign of error flips

inline constexpr double LIFT_TOLERANCE = 2; // deg, how close counts as "at target"
inline constexpr int LIFT_DEFAULT_TIMEOUT = 1500; // ms, max time liftControl waits
inline constexpr double LIFT_MIN_OUTPUT = 1500; // mV floor so small errors still break stiction

inline double liftTarget = 0; // target the PID is driving toward
inline bool liftHolding = false; // true while the PID task owns the lift motors

// lift position in degrees, averaged across both lift motors
inline double liftGetPosition() {
    std::vector<double> positions = liftMotors.get_position_all();
    if (positions.empty()) return 0;
    double sum = 0;
    for (double position : positions) sum += position;
    return sum / positions.size();
}

// point the lift at a target and let the PID task drive there (returns immediately)
inline void liftSetTarget(double target) {
    liftTarget = target;
    liftPid.reset();
    liftHolding = true;
}

// hand the lift back to manual control (jog / raw move_voltage)
inline void liftManual() {
    liftHolding = false;
    liftPid.reset();
}

// true once the PID is on and the lift is within tolerance of its target
inline bool liftAtTarget() {
    return liftHolding && std::fabs(liftGetPosition() - liftTarget) <= LIFT_TOLERANCE;
}

// drive to target and block until it settles (or timeout), then keep holding - used by autonomous
inline bool liftControl(double target, int timeoutMs = LIFT_DEFAULT_TIMEOUT) {
    liftSetTarget(target);
    for (int elapsed = 0; elapsed < timeoutMs; elapsed += 10) {
        if (liftAtTarget()) return true;
        pros::delay(10);
    }
    return liftAtTarget();
}

// runs on its own pros::Task, updates the PID every 10ms
inline void liftTaskFn() {
    liftMotors.tare_position_all();
    while (true) {
        if (liftHolding) {
            double error = liftTarget - liftGetPosition();
            double output = liftPid.update(error);
            if (std::fabs(error) > LIFT_TOLERANCE && std::fabs(output) < LIFT_MIN_OUTPUT) {
                output = std::copysign(LIFT_MIN_OUTPUT, error);
            }
            liftMotors.move_voltage((int) std::clamp(output, -12000.0, 12000.0));
        }
        pros::delay(10);
    }
}
