#pragma once

#include "EZ-Template/piston.hpp"
#include "api.h"
#include "pros/adi.hpp"
#include "pros/link.h"

// Your motors, sensors, etc. should go here.  Below are examples

inline pros::Motor intake(1);
inline pros::Motor ladder_arm(2);
// inline pros::adi::DigitalIn limit_switch('A');
//inline pros::adi::Encoder test_rev('A','B', false);

/*
inline pros::Motor lift_left_1(-4);
inline pros::Motor lift_left_2(5);
inline pros::Motor lift_left_3(6);
inline pros::Motor lift_right_1(7);
inline pros::Motor lift_right_2(-8);
inline pros::Motor lift_right_3(-9);
*/
inline pros::MotorGroup lift({-4, 5, 6, 7, -8, -9}, pros::v5::MotorGears::red); 
inline pros::adi::DigitalOut dock(1);
inline pros::adi::DigitalOut platform(2);
inline ez::Piston mogo('G');
//inline ez::Piston lift_brake('H');
//inline ez::Piston lift_brake('B', 3);
inline pros::adi::Pneumatics lift_brake({3, 'B'}, false);
