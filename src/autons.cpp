#include "autons.hpp"
#include "main.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "subsystems.hpp"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// These are out of 127
const int DRIVE_SPEED = 110;
const int TURN_SPEED = 90;
const int SWING_SPEED = 110;

///
// Constants
///
void default_constants() {
  // P, I, D, and Start I
  // https://ez-robotics.github.io/EZ-Template/tutorials/tuning_constants

  //chassis.pid_drive_constants_set(20.0, 0.0, 100.0);  //d was 12
  chassis.pid_drive_constants_set(5, 0,0.0);        // Fwd/rev constants, used for odom and non odom motions
  chassis.pid_heading_constants_set(7.0, 0.0, 20.0);        // Holds the robot straight while going forward without odom
  
  //chassis.pid_turn_constants_set(3.0, 0.05, 20.0, 15.0);     // Turn in place constants

  chassis.pid_turn_constants_set(25, 0.0000000001, 140, 0.05);
  chassis.pid_swing_constants_set(20, 0, 5);           // Swing constants

  chassis.pid_odom_angular_constants_set(12, 0, 45);    // Angular control for odom motions
//7
//17 shakes
  chassis.pid_odom_boomerang_constants_set(5.8, 0.0, 32.5);  // Angular control for boomerang motions

  // Exit conditions
  // https://ez-robotics.github.io/EZ-Template/tutorials/tuning_exit_conditions
  chassis.pid_turn_exit_condition_set(90_ms, 0_deg, 250_ms, 7_deg, 500_ms, 500000_ms);

  chassis.pid_swing_exit_condition_set(90_ms, 0_deg, 250_ms, 7_deg, 500_ms, 500000_ms);

  // chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);
  //chassis.pid_drive_exit_condition_set(200_ms, 1.5_in, 400_ms, 3.5_in, 1000_ms, 1000_ms);
  chassis.pid_drive_exit_condition_set(50_ms, 1_in, 150_ms, 8_in, 1000_ms, 1000000_ms);

  chassis.pid_odom_turn_exit_condition_set(150_ms, 3_deg, 250_ms, 7_deg, 500_ms, 750000_ms, false);
  // Can increase by increments of 10 only
   chassis.pid_odom_drive_exit_condition_set(100_ms, 2_in, 250_ms, 3_in, 500_ms, 500000_ms, false);

  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  // Slew constants
  // https://ez-robotics.github.io/EZ-Template/tutorials/slew_constants
  chassis.slew_turn_constants_set(3_deg, 70);
  chassis.slew_drive_constants_set(3_in, 70);
  chassis.slew_swing_constants_set(3_in, 80);

  // The amount that turns are prioritized over driving in odom motions
  // - if you have tracking wheels, you can run this higher.  1.0 is the max
  chassis.odom_turn_bias_set(0.9);
//7
  chassis.odom_look_ahead_set(20_in);           // This is how far ahead in the path the robot looks at
  chassis.odom_boomerang_distance_set(16_in);  // This sets the maximum distance away from target that the carrot point can be
  chassis.odom_boomerang_dlead_set(0.625);     // This handles how aggressive the end of boomerang motions are

  chassis.pid_angle_behavior_set(ez::shortest);  // Changes the default behavior for turning, this defaults it to the shortest path there
}



 
void AI_BaseLine(){
  platform.set_value(true);               // Prepare platform
  chassis.pid_targets_reset();            // Reset PID targets to 0
  chassis.drive_imu_reset();              // Reset gyro (IMU) position
  chassis.drive_sensor_reset();           // Reset drive sensors
  chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);   // Set motors to hold
  chassis.odom_pose_set({-56_in, 32_in, 180_deg}); // Starting position

  chassis.pid_odom_set({{{-56_in, 28_in}, fwd, 60}}, true); // Move forward to dock
  chassis.pid_wait();

  pros::delay(500);
  dock.set_value(false); 
  pros::delay(500);

 // intake.move_velocity(-200); 


//Lift Up
  lift.set_current_limit_all(2500);
  lift.move_absolute(-500, 75);
  chassis.drive_current_limit_set(2500);
  lift_brake.set(false);


//Drive Forward 
   chassis.pid_odom_set({{{-33_in, 7_in}, fwd, 70}}, true); // Move to ladder
  chassis.pid_wait();

  //pros::delay(3000); 




//Ratchets engaged
  pros::delay(3000); 
  lift_brake.set(false);


//   chassis.pid_odom_set({{{-44_in,6_in}, fwd, 110}, // First red ring intake
//                         {{-18_in, -28_in}, fwd, 110}, // Second red ring intake
//                         {{-9_in, -37_in}, fwd, 70}}, // Line up for 1st mogo
//                        true);
//   chassis.pid_wait();
 
// //Intake Stop
//   intake.brake();
//   intake.set_current_limit(0);
  


//Mogo CODE ENTER HERE




// //Lift Up
//   lift.set_current_limit_all(2500);
//   lift.move_absolute(-2000, 75);

//   // Driving/Turning to the ladder 
//   pros::delay(250); 
//   chassis.pid_odom_set({{{-4.75_in, -20.25_in, -45_deg}, fwd, 120}},
//     false); 
//   chassis.pid_wait();
//   pros::delay(500); //250

//   //Platform Set
//   platform.set_value(false);

//   //Raising Lift to above the High Stake
//   lift.set_current_limit_all(2500);
//   lift.move_absolute(-3180, 80); 
//   pros::delay(3000);

//   //Ratched Engaged/Locking
  pros::delay(500); 
  lift_brake.set(false);

  //END OF AUTON
}




void testTune(){
  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0

  chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);   // Set motors to hold.  This helps autonomous consistency
  chassis.odom_pose_set({-56_in, 32_in, 180_deg});
  lift.move_absolute(-380, DRIVE_SPEED); 
  chassis.pid_wait();
  //odom needs to be 2 inches more forward to be accurate


  chassis.pid_odom_set({{{-56_in, -32_in}, fwd, 127}}, 
                       false);
  chassis.pid_wait();


  // // Mtestve 40 inches forward
  // chassis.pid_drive_set(49_in, 120); // Drive 40 inches forward at 50% speed
  // chassis.pid_wait(); // Wait for the movement to complete

  // pros::delay(1000);

  // chassis.pid_drive_set(-49_in, 120); // Drive 40 inches forward at 50% speed
  // chassis.pid_wait(); // Wait for the movement to complete

  // pros::delay(1000); 

  // // // Turn 90 degrees
  // chassis.pid_turn_set(90, 120); 
  // chassis.pid_wait(); // Wait for the turn to complete
}
 
void tuning(){
  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD); 
  chassis.odom_pose_set({0_in, 0_in, 0_deg});

  chassis.pid_odom_set({{{36_in, 0_in}, fwd, 80}}, // Move forward to dock
                       true);
  chassis.pid_wait();
}




void ladder_arm_test(){

  mogo.set(true);
  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);   // Set motors to hold

  lift.set_current_limit_all(2500);
  lift.move_absolute(-1800, 950);
  lift_brake.set(false);



  //Ladder Arm Extend outwards in order to touch ladder
  pros::delay(400); //250
  ladder_arm.set_current_limit(2500);
  ladder_arm.move_absolute(-1000, 70); 

}


void No_Odom_Red_MSOE(){
long start_time = pros::millis();
  mogo.set(true);
  chassis.pid_targets_reset();                
  chassis.drive_imu_reset();                  
  chassis.drive_sensor_reset();               
  chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);

  // Intake On
  intake.move_velocity(-200);

  // Docking
  chassis.pid_drive_set(12_in, 90);
  chassis.pid_wait_until(3_in);
  pros::delay(200);
  dock.set_value(false);

  lift.set_current_limit_all(2500);
  lift.move_absolute(-1800, 150);
  //lift_brake.set(false);
  
  // Turn
  chassis.pid_turn_set(-92_deg, 120); //-90 was too far

  //lift_brake.set(false);

  chassis.drive_current_limit_set(2500);

  // Drive to ladder
  chassis.pid_drive_set(43_in, 120); 

  pros::delay(3000); 
  platform.set_value(false);
  chassis.drive_current_limit_set(0);


  while (pros::millis()- start_time < 3000){
    pros::delay(10);
  }
  ladder_arm.set_current_limit(2500);
  ladder_arm.move_absolute(-1000, 70); 



  lift.set_current_limit_all(2500);
  lift.move_absolute(-3180, 80);

  pros::delay(1500); 
  lift.move_absolute(-3150, 80);
  lift.move_absolute(-3180, 80);
  lift_brake.set(false);


  while (pros::millis()- start_time < 10000){
    pros::delay(10);
  }
  // Intake + Ladder Stop
  intake.brake();
  intake.set_current_limit(0);
  ladder_arm.set_current_limit(0);


}

void No_Odom_Blue_MSOE(){
  long start_time = pros::millis();
  mogo.set(true);
  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);   // Set motors to hold

  //Intake On
  intake.move_velocity(-200);

  // docking
  chassis.pid_drive_set(12_in, 90);
  chassis.pid_wait_until(3_in);
  pros::delay(200);
  dock.set_value(false);

  lift.set_current_limit_all(2500);
  lift.move_absolute(-1800, 150);
  lift_brake.set(false);
  

  chassis.pid_turn_set(92_deg, 120);

  lift_brake.set(false);


  chassis.drive_current_limit_set(2500);

  // go to ladder

  chassis.pid_drive_set(43_in, 120); //40_in 

  pros::delay(3000); //1500
  platform.set_value(false);

  ladder_arm.set_current_limit(2500);
  ladder_arm.move_absolute(-1000, 70); 

  //Raising Lift to above the High Stake
  lift.set_current_limit_all(2500);
  lift.move_absolute(-3180, 80);
  ladder_arm.set_current_limit(0);

  pros::delay(1500); 
  lift.move_absolute(-3150, 80);
  lift.move_absolute(-3180, 80);
  lift_brake.set(false);

    //Intake Stop
  intake.brake();
  intake.set_current_limit(0);


}




void MSOE(){
  long start_time = pros::millis();
  mogo.set(true);
  lift_brake.set(false);
  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);   // Set motors to hold

  //Setting the starting position/coordinates
  chassis.odom_pose_set({51_in, -14_in, 180_deg});

  intake.move_velocity(-200); // Begin intaking

  // docking
  chassis.pid_odom_set({{{51_in, -22_in}, fwd, 90}},
    false);
  chassis.pid_wait();
  pros::delay(200);
  dock.set_value(false);

  lift.set_current_limit_all(2500);
  lift.move_absolute(-1800, 150);
  lift_brake.set(false);
  

  chassis.pid_turn_set(35_deg, 120);

  //pros::delay(200);

  // drive forward to intake rings
  //chassis.pid_drive_set(6_in, 170); 
  //chassis.pid_wait();

  //lift.move_absolute(-1800, 150);
  lift_brake.set(false);

  chassis.drive_current_limit_set(2500);
  // go to ladder
  chassis.pid_odom_set({{{11_in, -20_in, 90_deg}, fwd, 160}},
    false); 
  chassis.pid_wait(); // 13,-20 at worlds
  

  pros::delay(1500);


  ladder_arm.set_current_limit(0);

  platform.set_value(false);


  //Raising Lift to above the High Stake
  lift.set_current_limit_all(2500);
  lift.move_absolute(-3180, 80);

  pros::delay(1500); 
  lift_brake.set(false);

}

void old_MSOE(){
  long start_time = pros::millis();
  mogo.set(true);
  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);   // Set motors to hold

  //Setting the starting position/coordinates
  chassis.odom_pose_set({-51_in, -14_in, 180_deg});

  intake.move_velocity(-200); // Begin intaking

  // docking
  chassis.pid_odom_set({{{-51_in, -20_in}, fwd, 90}},
    false);
  chassis.pid_wait();
  pros::delay(200);
  dock.set_value(false);

  lift.set_current_limit_all(2500);
  lift.move_absolute(-1800, 150);
  lift_brake.set(false);
  
  //pros::delay(200);

  // drive forward to intake rings
  //chassis.pid_drive_set(6_in, 170); 
  //chassis.pid_wait();

  //lift.move_absolute(-1800, 150);
  lift_brake.set(false);


  

  chassis.drive_current_limit_set(2500);

  // align with ladder
  chassis.pid_odom_set({{{-10_in, -35.5_in, 45_deg}, fwd, 150}}, // was at 120, 5/10 10 am
                        false);
  chassis.pid_wait(); 
  
  ladder_arm.set_current_limit(2500);
  ladder_arm.move_absolute(-1000, 70); 

  intake.brake();
  intake.set_current_limit(0);

  chassis.drive_current_limit_set(2500);
  // go to ladder
  chassis.pid_odom_set({{{4.5_in, -20.5_in, 45_deg}, fwd, 160}},
    false); 
  chassis.pid_wait();

  ladder_arm.set_current_limit(0);

  platform.set_value(false);


  //Raising Lift to above the High Stake
  lift.set_current_limit_all(2500);
  lift.move_absolute(-3180, 80);

  pros::delay(1500); 
  lift_brake.set(false);

}



void Blue_Worlds_2(){

  //Initlize and dock
  long start_time = pros::millis();
  mogo.set(true);
  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);   // Set motors to hold

  //Setting the starting position/coordinates
  chassis.odom_pose_set({51_in, -14_in, 180_deg});

  // Move forward to dock + Dock Engages
  chassis.pid_odom_set({{{51_in, -20_in}, fwd, 60}},
                        false);
  chassis.pid_wait();
  pros::delay(25); //200
  dock.set_value(false);
  pros::delay(50); //300

  //Raising 15" so it is not dragging on the floor
  lift.set_current_limit_all(2500);
  lift.move_absolute(-90, 75);


//Time Stamped 
  //Move forward so 15" can grab the High Stake Ring Stack
  chassis.pid_odom_set({{{46_in, -33_in}, fwd, 120}},
                        false);
  chassis.pid_wait();
  while (pros::millis()- start_time < 2800){
    pros::delay(10);
  }
  // Raise 15" so we can intake the platform rings
  lift.move_absolute(-800, 80);

  // Move backwards to mogo
  chassis.pid_odom_set({{{43_in, -5_in}, rev, 120}},
                        false);
  chassis.pid_wait();
  pros::delay(300);

  //Grabbing Mogo
  mogo.set(false);
  pros::delay(250);

  //Intake On
  intake.move_velocity(-200);

  //Intake Blue Alone Ring
  chassis.pid_odom_set({{{10_in, -26_in}, fwd, 70}}, 
                        false);
  chassis.pid_wait();
  pros::delay(300); //500

  //Intake Blue Bottom Stack Ring
  chassis.pid_odom_set({{{20_in, -50_in}, fwd, 70}},
              false); 
  chassis.pid_wait();
  pros::delay(400); //750

  //Driving backwards from the rings 
  chassis.pid_drive_set(-10_in, 120); 
  chassis.pid_wait();
  //lift.set_current_limit_all(2500);
  //lift.move_absolute(-2000, 75);

  //Driving to Ladder
  chassis.pid_turn_set(-45_deg, 90);
  chassis.pid_wait(); 
  chassis.pid_odom_set({{{10_in, -35.5_in, -45_deg}, fwd, 100}},
                        false);
  chassis.pid_wait(); 

//Intake Stop
  intake.brake();
  intake.set_current_limit(0);
  
//Lift Up
  lift.set_current_limit_all(2500);
  lift.move_absolute(-2000, 75);

  //Ladder Arm Extend outwards in order to touch ladder
  ladder_arm.set_current_limit(2500);
  pros::delay(500); //250
  ladder_arm.move_absolute(-1000, 70); 

  // Driving/Turning to the ladder 
  pros::delay(250); 
  chassis.pid_odom_set({{{-4.75_in, -20.25_in, -45_deg}, fwd, 120}},
    false); 
  chassis.pid_wait();
  pros::delay(500); //250

  //Ladder Arm Stop
  ladder_arm.set_current_limit(0);

  //Platform Set
  platform.set_value(false);

  //Raising Lift to above the High Stake
  lift.set_current_limit_all(2500);
  lift.move_absolute(-3180, 80); 
  pros::delay(3000);

  //Lift Moving down for High Stake
  while (pros::millis()- start_time < 23500){
    pros::delay(10);
  }
  lift.move_absolute(-2750, 50);

  //Raising Lift to above the High Stake
  pros::delay(3000);
  lift.move_absolute(-3500, 50);

  //Ratched Engaged/Locking
  pros::delay(500); 
  lift_brake.set(false);

  //END OF AUTON
}



void Red_Worlds(){
//Level One Plan For Red Side
  // Grab Stack for Claw 
  // Grab Mogo
  // Intake Blue alone ring
  // Intake blue ring from the stack
  // Turn and Line up for climb
  // Tier 3 climb buddy + platform
  // High Stake
  //Initlize and dock

  long start_time = pros::millis();
  mogo.set(true);
  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);   // Set motors to hold

  //Setting the starting position/coordinates
  chassis.odom_pose_set({-51_in, -14_in, 180_deg});

  // Move forward to dock + Dock Engages
  chassis.pid_odom_set({{{-51_in, -20_in}, fwd, 60}},
                        false);
  chassis.pid_wait();
  pros::delay(25); //200
  dock.set_value(false);
  pros::delay(50); //300

  //Raising 15" so it is not dragging on the floor
  lift.set_current_limit_all(2500);
  lift.move_absolute(-90, 75);


//Time Stamped 
  //Move forward so 15" can grab the High Stake Ring Stack
  chassis.pid_odom_set({{{-46_in, -33_in}, fwd, 120}},
                        false);
  chassis.pid_wait();
  while (pros::millis()- start_time < 2800){
    pros::delay(10);
  }
  // Raise 15" so we can intake the platform rings
  lift.move_absolute(-800, 80);

  // Move backwards to mogo
  chassis.pid_odom_set({{{-43_in, -5_in}, rev, 120}},
                        false);
  chassis.pid_wait();
  pros::delay(300);

  //Grabbing Mogo
  mogo.set(false);
  pros::delay(250);

  //Intake On
  intake.move_velocity(-200);

  //Intake Blue Alone Ring
  chassis.pid_odom_set({{{-10_in, -26_in}, fwd, 70}}, 
                        false);
  chassis.pid_wait();
  pros::delay(300); //500

  //Intake Blue Bottom Stack Ring
  chassis.pid_odom_set({{{-20_in, -50_in}, fwd, 70}},
              false); 
  chassis.pid_wait();
  pros::delay(400); //750

  //Driving backwards from the rings 
  chassis.pid_drive_set(-10_in, 120); 
  chassis.pid_wait();
  //lift.set_current_limit_all(2500);
  //lift.move_absolute(-2000, 75);

  //Driving to Ladder
  chassis.pid_turn_set(45_deg, 90);
  chassis.pid_wait(); 
  chassis.pid_odom_set({{{-10_in, -35.5_in, 45_deg}, fwd, 100}},
                        false);
  chassis.pid_wait(); 

//Intake Stop
  intake.brake();
  intake.set_current_limit(0);
  
//Lift Up
  lift.set_current_limit_all(2500);
  lift.move_absolute(-2000, 75);

  //Ladder Arm Extend outwards in order to touch ladder
  ladder_arm.set_current_limit(2500);
  pros::delay(500); //250
  ladder_arm.move_absolute(-1000, 70); 

  // Driving/Turning to the ladder 
  pros::delay(250); 
  chassis.pid_odom_set({{{4.75_in, -20.25_in, 45_deg}, fwd, 120}},
    false); 
  chassis.pid_wait();
  pros::delay(500); //250
//Y Coords: -20.5(June 2nd)
//X Coords: 4.5(June 2nd)

  //Ladder Arm Stop
  ladder_arm.set_current_limit(0);

  //Platform Set
  platform.set_value(false);

  //Raising Lift to above the High Stake
  lift.set_current_limit_all(2500);
  lift.move_absolute(-3180, 80); // 3130 was too low at one point | 5/9 6:00pm
  pros::delay(3000);

  //Lift Moving down for High Stake
  while (pros::millis()- start_time < 23500){
    pros::delay(10);
  }
  lift.move_absolute(-2750, 50);

  //Raising Lift to above the High Stake
  pros::delay(3000);
  lift.move_absolute(-3500, 50);

  //Ratched Engaged/Locking
  pros::delay(500); 
  lift_brake.set(false);

  //END OF AUTON
}

///
// Worlds Blue Auton
///
void Blue_Worlds(){
//Level One Plan For Blue Side
  // Grab Stack for Claw 
  // Grab Mogo
  // Intake red alone ring
  // Intake red ring from the stack
  // Turn and Line up for climb
  // Tier 3 climb buddy + platform
  // High Stake
 
  //Initlize and dock
  long start_time = pros::millis();

  mogo.set(true);
  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);   // Set motors to hold

  //Setting the starting position/coordinates
  chassis.odom_pose_set({51_in, -14_in, 180_deg});

  // Move forward to dock + Dock Engages
  chassis.pid_odom_set({{{51_in, -20_in}, fwd, 60}},
                        false);
  chassis.pid_wait();
  pros::delay(200);
  dock.set_value(false);
  pros::delay(300); 

  //Raising 15" so it is not dragging on the floor
  lift.set_current_limit_all(2500);
  lift.move_absolute(-90, 75);

  //Move forward for Claw 
  chassis.pid_odom_set({{{46_in, -33_in}, fwd, 120}},
                        false);
  chassis.pid_wait();

  // 15" Grabs Rings
  pros::delay(1000);

  // Raise 15" so we can intake the platform rings
  lift.move_absolute(-800, 80);

  // Move backwards to mogo
  chassis.pid_odom_set({{{43_in, -5_in}, rev, 120}},
                        false);
  chassis.pid_wait();

  //Delay for 15" picking up rings

  pros::delay(300); // was 1000, reduced for time savings | 5/9 12:30pm

  //Mogo Mech is Used
  mogo.set(false);

  //Delay 
  pros::delay(250);

  //Intake Rings On Code
  intake.move_velocity(-200);

  //Intake Blue Alone Ring
  chassis.pid_odom_set({{{10_in, -26_in}, fwd, 70}}, 
                        false);
  chassis.pid_wait();

  //Wait 
  pros::delay(500);

  //Intake Blue Bottom Stack Ring
  chassis.pid_odom_set({{{20_in, -50_in}, fwd, 70}},
              false); 
  chassis.pid_wait();

  //Wait
  pros::delay(750); //1100

  //Driving backwards from the rings 
  chassis.pid_drive_set(-10_in, 120); 
  chassis.pid_wait();
  lift.set_current_limit_all(2500);
  lift.move_absolute(-2000, 75);

  //Driving to Ladder
  chassis.pid_turn_set(-45_deg, 90);
  chassis.pid_wait(); 

  chassis.pid_odom_set({{{10_in, -35.5_in, -45_deg}, fwd, 100}},
                        false);
  chassis.pid_wait(); 

  //Ladder Arm Extend
  pros::delay(250);
  ladder_arm.set_current_limit(2500);
  ladder_arm.move_absolute(-1000, 70); 

  // delay bettween driving to the ladder and getting super close to the corner of the ladder
  //pros::delay(250); // Commented out for time savings | 5/9 12:30 pm
  chassis.pid_odom_set({{{-4.5_in, -20.5_in, -45_deg}, fwd, 120}},                 // -4.5, -20.5 |  5/9 11:20 | 3-4 inches too far right
    false); 
  chassis.pid_wait();


  //Delay between driving and lifting 
  pros::delay(250); 

  //Intake and Ladder Arm Stopping
  intake.brake();
  intake.set_current_limit(0);
  ladder_arm.set_current_limit(0);

  //Platform sets so ontop of rings 
  platform.set_value(false);


  //Raising Lift to max height of the lift 
  lift.set_current_limit_all(2500);

  lift.move_absolute(-3180, 80); // 3130 was too low at one point | 5/9 6:00pm
  pros::delay(3000);

  while (pros::millis()- start_time < 23500){ // 23000 because it may not always touch ladder | 5/9 11:50 am
    pros::delay(10);
  }

  //Delay between lift movements
  

  //Move lift to score on High Stake

  lift.move_absolute(-2750, 50);  //-2750 worked, but 15" didnt hard stop on high stake| 5/9 12:10 pm
  // -2900 was too high and 15" was overshooting | 5/9 11:50 am
   

  //Delay between lift moving down and the ratchet locking
  pros::delay(3000);

  //Raising Lift at end
  lift.move_absolute(-3500, 50);

  //Delay between lift moving down and the ratchet locking
  pros::delay(500); 

  //Ratchet Engaged
  lift_brake.set(false);

}

void red_AWP_match(){


  // FOR WORLDS GET RID OF PRELOAD USAGE


  // Initialization 
  platform.set_value(true); 
  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);   // Set motors to hold.  This helps autonomous consistency
  chassis.odom_pose_set({-56_in, 32_in, 180_deg});
  chassis.pid_odom_set({{{-56_in, 28_in}, fwd, 60}}, // Move forward to dock
                       true);
  chassis.pid_wait();
  pros::delay(200);
  dock.set_value(false);
  pros::delay(300);
  // Score on mogo

  lift.set_current_limit_all(2500);
  lift.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);

  lift.move_absolute(-1800, 80); // Move lift up to score on mogo
  pros::delay(1000);

  chassis.pid_odom_set({{{-52.75_in, 17_in}, fwd, 60}}, // Move forward to mogo
                       false);
  chassis.pid_wait();


  pros::delay(500);

  lift.move_absolute(-650, 80); // Move lift down to score on mogo -400
  
  pros::delay(500);

  //Score on alliance stake


  chassis.pid_odom_set({{{-58_in, 26_in}, rev, 120}}, // Move back to line up with ring stack
                       true);
  chassis.pid_wait();

  lift.move_absolute(0, 80);


  chassis.pid_odom_set({{{-57.5_in, 14.5_in}, fwd, 120}}, //45 // move forward to grab ring stack
                       true);
  chassis.pid_wait();

  pros::delay(500); // Wait to grab

  lift.move_absolute(-1900, 100); // Move lift up to score on alliance stake
  
  pros::delay(700);

  chassis.pid_turn_set(223, 120); // Turn to alliance stake
  chassis.pid_wait();

  chassis.pid_drive_set(0.5, 120); // Move a bit forward to alliance stake
  chassis.pid_wait();

  pros::delay(500);

  lift.move_absolute(-1350, 80);

  pros::delay(500);


  //Go around mogo 
 
  //chassis.pid_odom_set({{{-60_in, -11_in}, fwd, 80}},true);// Move past mogo       //57.5 // caused problems by turning whle 15" is on stake
  
  //chassis.pid_wait();


  // Score MoGo in positive corner
 // chassis.pid_drive_set(-3, 80); // Move back from alliance stake
  //chassis.pid_wait();
  
  chassis.pid_drive_set(-7_in, 120); // was -7
  chassis.pid_wait();

    //chassis.pid_odom_set({{{-47.5_in, 17_in}, rev, 80}}, // Move back to line up with mogo // uncomment for mostly working
    //chassis.pid_wait();

    // chassis.pid_turn_set(0, 90); // Turn so clamp is facing mogo
    //chassis.pid_wait();

  lift.move_absolute(-100, 80);

    //  chassis.pid_odom_set({{{-47.5_in, 6_in}, rev, 80}}, // Move into mogo
    //                     true);
    //  chassis.pid_wait();

    // mogo.set(true);

    // pros::delay(500);

    //  chassis.pid_turn_set(180, 100); // Turn back 
    //chassis.pid_wait();

  pros::delay(200);

  lift.move_absolute(-100, 80);

  chassis.pid_odom_set({{{-49_in, -28_in}, fwd, 120},
                        {{-49_in, -37_in}, fwd, 120}}, // Move to high stake ring stack
                       true);
  chassis.pid_wait();

  pros::delay(400);

  //chassis.pid_turn_set(15, 80); // Turn so mogo is facing corner
  //chassis.pid_wait();

  // removed for time
  //chassis.pid_odom_set({{{-53_in, -53_in}, rev, 60}}, // score mogo into corner
  //                     false);
  //chassis.pid_wait();

  // pros::delay(500);

  mogo.set(false);


  lift.move_absolute(-600, 80); //-500
  
  // Intake platform rings

  intake.move_velocity(-150);
  
  // skipping first ring with preload
  //chassis.pid_odom_set({{{-26_in, -25_in}, fwd, 50}}, // Intake 1st blue platform ring
  //                     true);
  //chassis.pid_wait();

  //pros::delay(400);

  chassis.pid_odom_set({{{-23.5_in, -47_in}, fwd, 120}}, // Get 2nd platform ring
                       true);
  chassis.pid_wait(); 

  lift.set_current_limit_all(2500);
  lift.move_absolute(-2000, 75); 

  chassis.pid_turn_set(47, 120); // Turn to ladder
  chassis.pid_wait();
  
  pros::delay(400);
  

  chassis.pid_odom_set({{{-13.5_in, -36.5_in}, fwd, 120}}, // Move to ladder //-13 and -36
                       true);
  chassis.pid_wait();

  chassis.pid_drive_set(6.75_in, 120);
  chassis.pid_wait();

  intake.brake();

  chassis.pid_turn_set(47, 120); // Turn to ladder
  chassis.pid_wait();

  chassis.pid_drive_set(-1_in, 120);
  chassis.pid_wait();

  pros::delay(500);
  platform.set_value(false);
  pros::delay(1000);
  ladder_arm.move_relative(-900, 100); //-900

  lift.set_current_limit_all(2500);
  lift.move_absolute(-3200, 75); 
  pros::delay(2000);

  lift.move_absolute(-2770, 75); 
  pros::delay(500);
  lift.move_absolute(-3700, 75);  //-3400
  pros::delay(800);
  lift_brake.set(false);
}

void blue_AWP_match(){
  // Initialization 
  platform.set_value(true);
  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);   // Set motors to hold.  This helps autonomous consistency
  chassis.odom_pose_set({56_in, 32_in, 180_deg});
  chassis.pid_odom_set({{{56_in, 28_in}, fwd, 60}}, // Move forward to dock
                       true);
  chassis.pid_wait();
  pros::delay(200);
  dock.set_value(false);
  pros::delay(300);
  // Score on mogo

  lift.set_current_limit_all(2500);
  lift.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);//Brake

  lift.move_absolute(-1800, 80); // Move lift up to score on mogo
  pros::delay(1000);

  chassis.pid_odom_set({{{52.75_in, 17_in}, fwd, 60}}, // Move forward to mogo
                       true);
  chassis.pid_wait();


  pros::delay(500);

  lift.move_absolute(-650, 80); // Move lift down to score on mogo
  
  pros::delay(500);

  // Score on alliance stake


  chassis.pid_odom_set({{{58_in, 26_in}, rev, 80}}, // Move back to line up with ring stack
                       true);
  chassis.pid_wait();

  lift.move_absolute(0, 80);

  chassis.pid_odom_set({{{57.5_in, 14.5_in}, fwd, 55}}, //45 // move forward to grab ring stack
                       true);
  chassis.pid_wait();

  pros::delay(500); // Wait to grab

  lift.move_absolute(-1900, 100); // Move lift up to score on alliance stake
  
  pros::delay(700);

  chassis.pid_turn_set(-223, 70); // Turn to alliance stake
  chassis.pid_wait();

  chassis.pid_drive_set(0.5, 60); // Move a bit forward to alliance stake
  chassis.pid_wait();

  pros::delay(500);

  lift.move_absolute(-1350, 80);

  pros::delay(500);

  //Go around mogo 
 
  //chassis.pid_odom_set({{{-60_in, -11_in}, fwd, 80}},true);// Move past mogo       //57.5 // caused problems by turning whle 15" is on stake
  
  //chassis.pid_wait();


  // Score MoGo in positive corner
 // chassis.pid_drive_set(-3, 80); // Move back from alliance stake
  //chassis.pid_wait();
  
  chassis.pid_drive_set(-7_in, 60); // was -3
  chassis.pid_wait();

    //chassis.pid_odom_set({{{-47.5_in, 17_in}, rev, 80}}, // Move back to line up with mogo // uncomment for mostly working
    //chassis.pid_wait();

    // chassis.pid_turn_set(0, 90); // Turn so clamp is facing mogo
    //chassis.pid_wait();

  lift.move_absolute(-100, 80);

    //  chassis.pid_odom_set({{{-47.5_in, 6_in}, rev, 80}}, // Move into mogo
    //                     true);
    //  chassis.pid_wait();

    // mogo.set(true);

    // pros::delay(500);

    //  chassis.pid_turn_set(180, 100); // Turn back 
    //chassis.pid_wait();

  pros::delay(200);

  lift.move_absolute(-100, 80);

  chassis.pid_odom_set({{{49_in, -37_in}, fwd, 80}}, // Move to high stake ring stack
                       true);
  chassis.pid_wait();

  pros::delay(400);

    //chassis.pid_turn_set(15, 80); // Turn so mogo is facing corner
    //chassis.pid_wait();

    // removed for time
    //chassis.pid_odom_set({{{-53_in, -53_in}, rev, 60}}, // score mogo into corner
    //                     false);
    //chassis.pid_wait();

    // pros::delay(500);

  mogo.set(false);


  lift.move_absolute(-500, 80);
  
  // Intake platform rings

  intake.move_velocity(-150);
  
    // skipping first ring with preload
    //chassis.pid_odom_set({{{-26_in, -25_in}, fwd, 50}}, // Intake 1st blue platform ring
    //                     true);
    //chassis.pid_wait();

    //pros::delay(400);


//Intake 2nd ring to go climb
  chassis.pid_odom_set({{{23.5_in, -47_in}, fwd, 80}}, // Get 2nd platform ring
                       true);
  chassis.pid_wait(); 


//Blue Auton CLimb

  lift.set_current_limit_all(2500);
  lift.move_absolute(-2000, 75); 

  chassis.pid_turn_set(-47, 70); // Turn to ladder
  chassis.pid_wait();
  
  pros::delay(400);

  chassis.pid_odom_set({{{14_in, -36_in}, fwd, 60}}, // Move to ladder
                       true);
  chassis.pid_wait();

  chassis.pid_drive_set(6.75_in, 60);
  chassis.pid_wait();

  intake.brake();


  chassis.pid_turn_set(-47, 70); // Turn to ladder
  chassis.pid_wait();

  chassis.pid_drive_set(-1_in, 70); 
  chassis.pid_wait();
 
  pros::delay(500);
  platform.set_value(false);
  pros::delay(1000);
  ladder_arm.move_relative(-800, 100);

  lift.set_current_limit_all(2500);
  lift.move_absolute(-3200, 75); 
  pros::delay(2000);

  lift.move_absolute(-2770, 75); 
  pros::delay(500);
  lift.move_absolute(-3400, 75);  //-3200
  pros::delay(800);
  lift_brake.set(false);


}

void purdueSkills(){

  bool isGearLocked = false;  // Gears are initially unlocked
   platform.set_value(true);  //false             // Prepare platform
   chassis.pid_targets_reset();            // Reset PID targets to 0
   chassis.drive_imu_reset();              // Reset gyro (IMU) position
   chassis.drive_sensor_reset();           // Reset drive sensors
   chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);  // Set motors to hold
   lift.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
   lift.set_current_limit_all(2500);
   
   // Startup Position and Docking mech
   chassis.odom_pose_set({-56_in, -32_in, 0_deg});  // 24” robot's start position  //180
   chassis.pid_odom_set({{{-56_in, -27.5_in}, fwd, 60}}, true); // Dock with second robot
   chassis.pid_wait();
   pros::delay(200);
   dock.set_value(false);
   pros::delay(300);
 
 
   lift.move_absolute(-1000, 75); //Gets 15" out of the way
 
   // Intake ON
   intake.move_velocity(-200); // Begin intaking
   chassis.pid_odom_set({{{-47_in, 0_in}, fwd, 70}}, true);
   
   chassis.pid_wait();
   
   chassis.pid_odom_set({{{-24_in, 24_in}, fwd, 70}}, true);
   chassis.pid_wait();
   
 
   // Turning and going to ladder to climb
   chassis.pid_turn_set(25_deg, 70);//75 and 25
   chassis.pid_odom_set({{{-6_in, 40_in}, fwd, 70}}, true);
   chassis.pid_wait();

   //Intake OFF
   intake.brake();

   lift.move_absolute(-600, 75);


   pros::delay(200);


  // Score 1st mogo 
   chassis.pid_odom_set({{{-53_in, 57_in}, rev, 70}}, true);
   chassis.pid_wait();


   chassis.pid_odom_set({{{-6_in, 40_in}, fwd, 70}}, true);
   chassis.pid_wait();

   // Line up to mogo
   chassis.pid_odom_set({{{7_in, 40_in}, fwd, 70}}, true);
   chassis.pid_wait();

   // Grab the mogo
   chassis.pid_odom_set({{{17_in, 30_in}, fwd, 70}}, true);
   chassis.pid_wait();

  // Score 2nd goal into corner
   chassis.pid_odom_set({{{54_in, 56_in}, fwd, 70}}, true);
   chassis.pid_wait();

   // Back out of the corner
   chassis.pid_drive_set(-8_in, 70);
   chassis.pid_wait();
   
   chassis.pid_odom_set({{{40_in, 30_in}, rev, 70}}, true);
   chassis.pid_wait();

   chassis.pid_odom_set({{{52_in, -10_in}, rev, 70}}, true);
   chassis.pid_wait();

   chassis.pid_odom_set({{{50_in, -40_in}, rev, 70}}, true);
   chassis.pid_wait();

  // Score 3rd goal in corner
   chassis.pid_odom_set({{{58_in, -53_in}, rev, 70}}, true);
   chassis.pid_wait();

   // Drive out of the corner
   chassis.pid_drive_set(8_in, 70);
   chassis.pid_wait();

   chassis.pid_odom_set({{{35_in, -42_in}, fwd, 70}}, true);
   chassis.pid_wait();


    // Align with 4th goal
   chassis.pid_odom_set({{{-2_in, -41_in}, fwd, 70}}, true);
   chassis.pid_wait();


  // Score 4th goal in corner
   chassis.pid_odom_set({{{-53_in, -58_in}, fwd, 70}}, true);
   chassis.pid_wait();

   chassis.pid_odom_set({{{2_in, -40_in}, rev, 70}}, true);  //-37
   chassis.pid_wait();


   pros::delay(500);
  
   // Climbing
       platform.set_value(false); //true
       lift.set_current_limit_all(2500);
       lift.move_absolute(-1000, 75); 
       printf("Lift Pos 1: %f\n", lift.get_position());
       if (lift.get_position() < -1990){
         printf("Done with lifting 1\n");
         pros::delay(500);
       }
       lift.move_absolute(-3400, 75); //-3200
       pros::delay(500);
       pros::delay(1200);
       lift_brake.set(false);  // Lock the brake
       lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);  // Ensure lift motors hold position
       lift.move_velocity(0);  // Prevent any movement
 
 
 }

void testingTues(){

 bool isGearLocked = false;  // Gears are initially unlocked
  platform.set_value(true);  //false             // Prepare platform
  chassis.pid_targets_reset();            // Reset PID targets to 0
  chassis.drive_imu_reset();              // Reset gyro (IMU) position
  chassis.drive_sensor_reset();           // Reset drive sensors
  chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);  // Set motors to hold
  lift.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
  lift.set_current_limit_all(2500);
  
  // Startup Position and Docking mech
  chassis.odom_pose_set({-56_in, -32_in, 0_deg});  // 24” robot's start position  //180
  chassis.pid_odom_set({{{-56_in, -28_in}, fwd, 60}}, true); // Dock with second robot
  chassis.pid_wait();
  pros::delay(200);
   dock.set_value(false);
  pros::delay(300);


  lift.move_absolute(-1000, 75); //Gets 15" out of the way

  // Intake ON
  intake.move_velocity(-200); // Begin intaking
  chassis.pid_odom_set({{{-47_in, 0_in}, fwd, 70}}, true);
  chassis.pid_wait();
  chassis.pid_odom_set({{{-24_in, 24_in}, fwd, 70}}, true);
  chassis.pid_wait();
  //Intake OFF
  intake.brake();

  // Turning and going to ladder to climb
  chassis.pid_turn_set(25_deg, 70);//75 and 25
  chassis.pid_odom_set({{{-4_in, 36_in}, fwd, 70}}, true); //-5,33
  chassis.pid_wait();
  pros::delay(1300);//1000 and 10000



  // Climbing
      platform.set_value(false); //true
      lift.set_current_limit_all(2500);
      lift.move_absolute(-1000, 75); 
      printf("Lift Pos 1: %f\n", lift.get_position());
      if (lift.get_position() < -1990){
        printf("Done with lifting 1\n");
        pros::delay(500);
      }
      lift.move_absolute(-3300, 75); //-3200
      pros::delay(500);
      pros::delay(1200);
      lift_brake.set(false);  // Lock the brake
      lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);  // Ensure lift motors hold position
      lift.move_velocity(0);  // Prevent any movement


}

void skills(){
  platform.set_value(true);
  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);   // Set motors to hold.  This helps autonomous consistency
  chassis.odom_pose_set({-58.75_in, 24_in, 180_deg});

  chassis.pid_odom_set({{{-58.75_in, 20_in}, fwd, 60}}, // Move forward to dock
                       true);
  chassis.pid_wait();

  pros::delay(500);
  // dock.set_value(false); // Activate docking
  pros::delay(500);

  lift.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
  lift.set_current_limit_all(2500);
  lift.move_velocity(-80);
  pros::delay(500);
  lift.brake();

  intake.move_velocity(-200); // Begin intaking


  chassis.pid_odom_set({{{-44_in, 4_in}, fwd, 110}, // First red ring intake
                        {{-24_in, -24_in}, fwd, 110}, // Second red ring intake
                        {{0_in, -40_in}, fwd, 70}}, // Line up for 1st mogo
                       true);
  chassis.pid_wait();

  intake.brake();

  lift.move_velocity(60);
  pros::delay(400);
  lift.brake();

  chassis.pid_turn_set(250_deg, 80); // Turn for first mogo facing frontwards
  chassis.pid_wait();


  chassis.pid_odom_set({{{-58.5_in, -58.5_in}, fwd, 70}, // Push mogo into corner
                        {{0_in, -45_in}, rev, 70},}, // Go for climb
                        true);
  chassis.pid_wait();

  pros::delay(2000);

  lift.move_velocity(-60);
  while (lift.get_position(2) > -1800){
    pros::delay(10);
  }
  lift.brake();
  pros::delay(1000);

  chassis.pid_turn_set(0_deg, 70); // Turn for first mogo facing frontwards
  chassis.pid_wait();

  pros::delay(500);

  chassis.pid_drive_set(13.5_in, 70);
  chassis.pid_wait();

  pros::delay(600);

  ladder_arm.move(100);
  pros::delay(500);
  ladder_arm.brake();

  platform.set_value(false);

  lift.move_velocity(-60);
  while (lift.get_position(2) > -3250){
    pros::delay(10);
  }
  lift.brake();
  lift_brake.set(false);

}

void thirtyPointSkills(){
  platform.set_value(true);               // Prepare platform
  chassis.pid_targets_reset();            // Reset PID targets to 0
  chassis.drive_imu_reset();              // Reset gyro (IMU) position
  chassis.drive_sensor_reset();           // Reset drive sensors
  chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);   // Set motors to hold
  chassis.odom_pose_set({-58.75_in, 24_in, 180_deg}); // Starting position

  chassis.pid_odom_set({{{-58.75_in, 20_in}, fwd, 60}}, true); // Move forward to dock
  chassis.pid_wait();

  pros::delay(500);
  // dock.set_value(false); // Activate docking
  pros::delay(500);

  intake.move_velocity(-200); // Begin intaking

  chassis.pid_odom_set({{{-44_in, 4_in}, fwd, 110}, // First red ring intake
                        {{-24_in, -24_in}, fwd, 110}, // Second red ring intake
                        {{0_in, -47_in}, fwd, 70}}, // Line up for 1st mogo
                       true);
  chassis.pid_wait();

  intake.brake();

  // Lift control for stack manipulation (adjust based on your needs)
  lift.set_current_limit_all(2500);
  lift.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
  lift.move_absolute(-500, 75); // Move lift to grab stack
  pros::delay(1000);

  // Move to the ladder and set up for climb
  chassis.pid_odom_set({{{-9_in, -32.5_in}, fwd, 80}}, true); // Move to ladder
  chassis.pid_wait();

  intake.brake(); // Stop intake motors

  // Turn to face the ladder
  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  platform.set_value(false);  // Lower platform for climb
  pros::delay(2000);          // Wait for platform to settle
  ladder_arm.move_relative(500, 100); // Move ladder arm for climbing

  lift.set_current_limit_all(2500); // Set current limit for the climb
  lift.move_absolute(-3200, 75);    // Move lift to the climbing position
  pros::delay(5000);                // Wait for lift to reach position

  lift.move_absolute(-2700, 75);    // Slightly adjust lift after climbing
  pros::delay(4000);                // Wait for the slight movement

  lift.move_absolute(-3200, 75);    // Final climb adjustment to the full position

  // Optional: Add any other post-climb actions here
  // lift_brake.set(false); // Example: releasing lift brake
}

void stateSkills() {   
    

  bool isGearLocked = false;  // Gears are initially unlocked

  platform.set_value(true);               // Prepare platform
  chassis.pid_targets_reset();            // Reset PID targets to 0
  chassis.drive_imu_reset();              // Reset gyro (IMU) position
  chassis.drive_sensor_reset();           // Reset drive sensors
  chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);  // Set motors to hold
  lift.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
  lift.set_current_limit_all(2500);
  
  // Startup Position and Docking mech
  chassis.odom_pose_set({-56_in, -32_in, 0_deg});  // 24” robot's start position  //180
  chassis.pid_odom_set({{{-56_in, -28_in}, fwd, 60}}, true); // Dock with second robot
  chassis.pid_wait();
  pros::delay(200);
  // dock.set_value(false);
  pros::delay(300);

  // Lining up for 1st ring and picking it up
  chassis.pid_odom_set({{{-53_in, -18_in}, fwd, 80}}, true); //70
  chassis.pid_wait();
  pros::delay(1000); // Waiting for 15" to pick up ring

  lift.move_absolute(-2000, 75); 


  // Turning and lining up to put the ring on the alliance stake
  chassis.pid_turn_set(70_deg, 70);
  chassis.pid_odom_set({{{-53_in, -4_in}, fwd, 70}}, true);
  chassis.pid_wait();  
  //Turns to face Red Alliance Stake
  chassis.pid_turn_set(-75_deg, 70);
  pros::delay(1000); //1000

  //Needs to go down by 4 inches max 
  lift.move_absolute(-1700, 60);
 
  pros::delay(3000); //wait till 15" puts the ring on Alliance stake




  //Turning and going to pick up Second red ring
  chassis.pid_odom_set({{{-45_in, 0_in}, rev, 70}}, true);
  chassis.pid_wait();
  lift.move_absolute(0, 60);  

  chassis.pid_turn_set(-75_deg, 70, ez::shortest); //75
  chassis.pid_odom_set({{{-30_in, 15_in}, fwd, 70}}, true);
  chassis.pid_wait();
  pros::delay(1000); //wait till 15" picks up ring

  //Driving and putting on Wall Stake
  chassis.pid_odom_set({{{0_in, 45_in}, fwd, 70}}, true);
  chassis.pid_wait();
  chassis.pid_turn_set(70_deg, 70); //-70
  chassis.pid_odom_set({{{0_in, 55_in}, fwd, 70}}, true);
  chassis.pid_wait();
  chassis.pid_odom_set({{{0_in, 40_in}, rev, 70}}, true);
  chassis.pid_wait();
  pros::delay(3000); //wait till 15" puts the ring on Wall stake
  chassis.pid_turn_set(100_deg, 70);

  //Driving backwards and putting mogo in Left Red corner
  chassis.pid_odom_set({{{-7_in, 36_in}, rev, 70}, 
                        {{-58_in, -58_in}, rev, 70},}, // Push mogo into corner
                        true);
  chassis.pid_wait();
 
 //ATM we have 1 mobile goal in corner, wall stake, alliance stake


  //Putting second Mogo In corner 
  chassis.pid_odom_set({{{6_in, 48_in}, fwd, 70}, 
                        {{16_in, 34_in}, fwd, 70},
                        {{55_in,55_in},fwd,70},},// Push mogo into corner
                        true);
  chassis.pid_wait();

  //Pushing third Mogo in corner 
  chassis.pid_odom_set({{{42_in, 17_in}, rev, 70}, 
                        {{58_in, -51_in}, rev, 70},}, // Push mogo into corner
                         true);
  chassis.pid_wait();

  //ATM we have 3 mobile goals in corner, wall stake, alliance stake
  //if we add climb without highstake then we are at 48 points

  //Intake first and second ring 
  chassis.pid_turn_set(-70_deg, 70);
  //Start Intake 
  chassis.pid_odom_set({{{23_in, -47_in}, fwd, 70},
                    {{-2_in,-47_in},fwd,70},}, 
                true);
  chassis.pid_wait();
  //Stop intake 
  chassis.pid_odom_set({{{-9_in, -46_in}, fwd, 70}}, true);
  chassis.pid_wait();

  //Putting 4th mobile goal in corner 
  // -53,-57
  chassis.pid_odom_set({{{-53_in, -57_in}, fwd, 70}}, true);
  chassis.pid_wait();
  chassis.pid_odom_set({{{-43_in, -57_in}, fwd, 70}}, true);
  chassis.pid_wait();
  chassis.pid_turn_set(70_deg, 70);

  //ATM we have 4 mobile goals in corner, wall stake, alliance stake
  //if we add climb without highstake then we are at 53 points


  //Intake ring  
  chassis.pid_odom_set({{{-24_in, -24_in}, fwd, 70}}, true);
  chassis.pid_wait();
  pros::delay(1000); //wait till 15" picks up ring
  chassis.pid_turn_set(70_deg, 70);

  chassis.pid_odom_set({{{-13.5_in, -36.5_in}, fwd, 60}}, // Move to ladder //-13 and -36
                       true);
  chassis.pid_wait();

  chassis.pid_drive_set(6.75_in, 60);
  chassis.pid_wait();

  chassis.pid_turn_set(47, 70); // Turn to ladder
  chassis.pid_wait();
  //Ready for high stake CLimb 

  // Climb for Each Corner if cant do high stake
  //   Red Alliance Stake Corner 
  //     (-35,0)
  //       chassis.pid_odom_set({{{-35_in, 0_in}, fwd, 60}},true);
  //       chassis.pid_wait();
  //   Blue Alliance Stake Corner
  //     (35,0)
  //       chassis.pid_odom_set({{{35_in, 0_in}, fwd, 60}},true);
  //       chassis.pid_wait();
  //   Left Wall Stake Corner 
  //     (0,35)
  //       chassis.pid_odom_set({{{0_in, 35_in}, fwd, 60}},true);
  //       chassis.pid_wait();
  //   Right Wall Stake Corner
  //     (0,-35)
  //       chassis.pid_odom_set({{{0_in, -35_in}, fwd, 60}},true);
  //       chassis.pid_wait();

  // //Climb At the End --- 5 Seconds 
        // lift.set_current_limit_all(2500);
        // lift.move_absolute(-2000, 75); 
        // printf("Lift Pos 1: %f\n", lift.get_position());
        // if (lift.get_position() < -1990){
        //   printf("Done with lifting 1\n");
        //   pros::delay(500);
        // }
        // lift.move_absolute(-3200, 75);
        // pros::delay(500);
        // ladder_arm.move_relative(-500, 100);
        // pros::delay(1200);
        // lift_brake.set(false); 
  //End Of CLIMBING 

}

void red_match_auton(){
  platform.set_value(true);
  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);   // Set motors to hold.  This helps autonomous consistency
  chassis.odom_pose_set({-56.8_in, -24_in, 180_deg});

  // dock.set_value(true);
  chassis.pid_odom_set({{{-56.8_in, -27_in}, fwd, 80}}, // Move forward to dock
                       true);
  chassis.pid_wait();

  // dock.set_value(false);

  pros::delay(1000);

  chassis.pid_odom_set({{{-52.4_in, -37_in}, fwd, 120}}, // Move to grab stack
                       true);
  chassis.pid_wait();

  pros::delay(500);
  intake.set_current_limit(2500);
  intake.move_velocity(-200);
  pros::delay(100);

  lift.set_current_limit_all(2500);
  lift.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
  lift.move_absolute(-500, 75);


  chassis.pid_odom_set({{{-24_in, -24_in}, fwd, 120}}, // Intake 1st blue platform ring
                       true);
  chassis.pid_wait();

  chassis.pid_odom_set({{{-23.5_in, -47_in}, fwd, 120}}, // Get 2nd platform ring
                       true);
  chassis.pid_wait(); 

  

  lift.set_current_limit_all(2500);
  lift.move_absolute(-2000, 75); 
  pros::delay(1000);

  chassis.pid_odom_set({{{-7_in, -30.5_in}, fwd, 120}}, // Move to ladder
                       true);
  chassis.pid_wait();

  intake.brake();


  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();



  platform.set_value(false);
  pros::delay(2000);
  ladder_arm.move_relative(500, 100);

  lift.set_current_limit_all(2500);
  lift.move_absolute(-3200, 75); 

  pros::delay(5000);

  lift.move_absolute(-2700, 75); 

  pros::delay(4000);

  lift.move_absolute(-3300, 75); 
  pros::delay(2000);

  lift_brake.set(false);
}

void blue_match_auton(){
  platform.set_value(true);
  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);   // Set motors to hold.  This helps autonomous consistency
  chassis.odom_pose_set({56.8_in, -24_in, 180_deg});

  // dock.set_value(true);
  chassis.pid_odom_set({{{56.8_in, -27_in}, fwd, 80}}, // Move forward to dock
                       true);
  chassis.pid_wait();

  // dock.set_value(false);

  pros::delay(1000);

  chassis.pid_odom_set({{{52.4_in, -37_in}, fwd, 60}}, // Move to grab stack
                       true);
  chassis.pid_wait();

  pros::delay(500);
  intake.set_current_limit(2500);
  intake.move_velocity(-200);
  pros::delay(100);

  lift.set_current_limit_all(2500);
  lift.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
  lift.move_absolute(-500, 75);


  chassis.pid_odom_set({{{24_in, -24_in}, fwd, 80}}, // Intake 1st blue platform ring
                       true);
  chassis.pid_wait();

  chassis.pid_odom_set({{{23.5_in, -47_in}, fwd, 80}}, // Get 2nd platform ring
                       true);
  chassis.pid_wait(); 

  

  lift.set_current_limit_all(2500);
  lift.move_absolute(-2000, 75); 
  pros::delay(1000);

   // Move to ladder
  //chassis.pid_odom_set({{{11_in, -37_in}, fwd, 80}}, // Move to ladder
  //chassis.pid_odom_set({{{9_in, -32.5_in}, fwd, 80}},
  //chassis.pid_odom_set({{{10_in, -34.5_in}, fwd, 80}},
  //chassis.pid_odom_set({{{11_in, -36.5_in}, fwd, 80}},
  chassis.pid_odom_set({{{5_in, -31_in}, fwd, 80}},
                       true);
  chassis.pid_wait();

  intake.brake();


  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();



  platform.set_value(false);
  pros::delay(2000);
  ladder_arm.move_relative(500, 100);

  lift.set_current_limit_all(2500);
  lift.move_absolute(-3200, 75); 

  pros::delay(5000);

  lift.move_absolute(-2700, 75); 

  pros::delay(4000);

  lift.move_absolute(-3300, 75); 
  pros::delay(2000);

  lift_brake.set(false);
}

///
// Drive Example
///
void drive_example() {
  // The first parameter is target inches
  // The second parameter is max speed the robot will drive at
  // The third parameter is a boolean (true or false) for enabling/disabling a slew at the start of drive motions
  // for slew, only enable it when the drive distance is greater than the slew distance + a few inches
  

  /*
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();
  */
}

///
// Turn Example
///
void turn_example() {
  // The first parameter is the target in degrees
  // The second parameter is max speed the robot will drive at

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();
}

///
// Combining Turn + Drive
///
void drive_and_turn() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Wait Until and Changing Max Speed
///
void wait_until_change_speed() {
  // pid_wait_until will wait until the robot gets to a desired position

  // When the robot gets to 6 inches slowly, the robot will travel the remaining distance at full speed
  chassis.pid_drive_set(24_in, 30, true);
  chassis.pid_wait_until(6_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // When the robot gets to -6 inches slowly, the robot will travel the remaining distance at full speed
  chassis.pid_drive_set(-24_in, 30, true);
  chassis.pid_wait_until(-6_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait();
}

///
// Swing Example
///
void swing_example() {
  // The first parameter is ez::LEFT_SWING or ez::RIGHT_SWING
  // The second parameter is the target in degrees
  // The third parameter is the speed of the moving side of the drive
  // The fourth parameter is the speed of the still side of the drive, this allows for wider arcs

  chassis.pid_swing_set(ez::LEFT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::LEFT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();
}

///
// Motion Chaining
///
void motion_chaining() {
  // Motion chaining is where motions all try to blend together instead of individual movements.
  // This works by exiting while the robot is still moving a little bit.
  // To use this, replace pid_wait with pid_wait_quick_chain.
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // Your final motion should still be a normal pid_wait
  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Auto that tests everything
///
void combining_movements() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, -45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Interference example
///
void tug(int attempts) {
  for (int i = 0; i < attempts - 1; i++) {
    // Attempt to drive backward
    printf("i - %i", i);
    chassis.pid_drive_set(-12_in, 127);
    chassis.pid_wait();

    // If failsafed...
    if (chassis.interfered) {
      chassis.drive_sensor_reset();
      chassis.pid_drive_set(-2_in, 20);
      pros::delay(1000);
    }
    // If the robot successfully drove back, return
    else {
      return;
    }
  }
}

// If there is no interference, the robot will drive forward and turn 90 degrees.
// If interfered, the robot will drive forward and then attempt to drive backward.
void interfered_example() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  if (chassis.interfered) {
    tug(3);
    return;
  }

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
}

// . . .
// Make your own autonomous functions here!
// . . .