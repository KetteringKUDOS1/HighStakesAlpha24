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
                            
inline pros::Motor lift_left_1(-3);     //Gear goes down for Lift DOWN
inline pros::Motor lift_left_2(5);      //Gear down up  for Lift Down 
inline pros::Motor lift_left_3(6);      //Gear down up for Lift Down 
inline pros::Motor lift_right_1(7);     //Gear down down for Lift Down 
inline pros::Motor lift_right_2(-8);    //Gear down up  for Lift Down 
inline pros::Motor lift_right_3(-9);    // Gear goes up for Lift DOWN
*/
inline pros::MotorGroup lift({5, 6,-8,-9}, pros::v5::MotorGears::red); 
inline pros::adi::DigitalOut dock(8);
inline pros::adi::DigitalOut platform(2);
inline ez::Piston mogo('G');

inline ez::Piston lift_brake('A');

// Motor Lift Group 
// 5 is center left 
// -3 top left ----- isnt plugged in
// 6 is bottom left
// 7 top right ---- is not plugged in 
// -8 center right
// -9 bottom right 




//  inline ez::Piston dock('H');


//inline ez::Piston lift_brake('B', 3);
//inline pros::adi::Pneumatics lift_brake({3, 'B'}, false);



// Port 1 used - Intake
// Port 2 used - Ladderarm
// Port 3 used - Lift
// Port 4 notused
// Port 5 used - Lift
// Port 6 used - Lift
// Port 7 used - Lift
// Port 8 used - Lift
// Port 9 used - Lift
// Port 10 used - Inertial
// Port 11 used - Drivetrain
// Port 12 used - Drivetrain
// Port 13 used - Drivetrain
// Port 14 used - Drivetrain
// Port 15 notused
// Port 16 used - Radio
// Port 17 used - Drivetrain
// Port 18 used - Drivetrain
// Port 19 used - Drivetrain
// Port 20 used - Drivetrain
// Port 21 notused

//3wire
    //F is mogo mech
    // 
