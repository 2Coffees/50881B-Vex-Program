#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"
#include "pros/adi.hpp"
#include "pros/motors.h"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples

// inline pros::Motor intake(1);
// inline pros::adi::DigitalIn limit_switch('A');

// Negative port = reversed. One of the two is usually reversed since they're mirrored.
inline pros::MotorGroup dr4b({12, -19}, pros::MotorGears::red);
// 5.5W intake rollers (two sets of rubber wheels)
inline pros::Motor claw(13, pros::MotorGears::green);
// 5.5W wrist. TODO: set the real port (negative = reversed, flip if it moves the wrong way)
inline pros::Motor wrist(15, pros::MotorGears::green);

// Wrist positions in motor degrees from fully IN (stowed). TODO: measure OUT.
inline const int WRIST_IN  = 0;
inline const int WRIST_OUT = 721;
