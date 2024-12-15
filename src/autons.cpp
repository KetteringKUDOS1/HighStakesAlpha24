#include "autons.hpp"
#include "main.h"
#include "pros/motors.h"
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
  chassis.pid_drive_constants_set(20.0, 0.0, 100.0);         // Fwd/rev constants, used for odom and non odom motions
  chassis.pid_heading_constants_set(11.0, 0.0, 20.0);        // Holds the robot straight while going forward without odom
  chassis.pid_turn_constants_set(3.0, 0.05, 20.0, 15.0);     // Turn in place constants
  chassis.pid_swing_constants_set(6.0, 0.0, 65.0);           // Swing constants
  chassis.pid_odom_angular_constants_set(6.5, 0.0, 65.0);    // Angular control for odom motions
  chassis.pid_odom_boomerang_constants_set(5.8, 0.0, 32.5);  // Angular control for boomerang motions

  // Exit conditions
  // https://ez-robotics.github.io/EZ-Template/tutorials/tuning_exit_conditions
  chassis.pid_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);
  chassis.pid_odom_turn_exit_condition_set(150_ms, 3_deg, 250_ms, 7_deg, 500_ms, 750_ms);
  // Can increase by increments of 10 only
  chassis.pid_odom_drive_exit_condition_set(150_ms, 1_in, 250_ms, 3_in, 500_ms, 750_ms);
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

  chassis.odom_look_ahead_set(7_in);           // This is how far ahead in the path the robot looks at
  chassis.odom_boomerang_distance_set(16_in);  // This sets the maximum distance away from target that the carrot point can be
  chassis.odom_boomerang_dlead_set(0.625);     // This handles how aggressive the end of boomerang motions are

  chassis.pid_angle_behavior_set(ez::shortest);  // Changes the default behavior for turning, this defaults it to the shortest path there
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

  pros::delay(1000);
  
  chassis.pid_odom_set({{{0_in, 0_in}, rev, 80}}, // Move forward to dock
                       true);
  chassis.pid_wait();
}

void red_AWP_match(){

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
                       true);
  chassis.pid_wait();


  pros::delay(500);



  lift.move_absolute(-400, 80); // Move lift down to score on mogo
  
  pros::delay(500);

  // Score on alliance stake


  chassis.pid_odom_set({{{-58_in, 26_in}, rev, 80}}, // Move back to line up with ring stack
                       true);
  chassis.pid_wait();

  lift.move_absolute(0, 80);

  chassis.pid_odom_set({{{-57.5_in, 14.5_in}, fwd, 55}}, //45 // move forward to grab ring stack
                       true);
  chassis.pid_wait();

  pros::delay(800); // Wait to grab

  lift.move_absolute(-1900, 100); // Move lift up to score on alliance stake
  
  pros::delay(700);

  chassis.pid_turn_set(223, 60); // Turn to alliance stake
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
  
  chassis.pid_drive_set(-3_in, 60);
  chassis.pid_wait();
  
  chassis.pid_odom_set({{{-47.5_in, 17_in}, rev, 80}}, // Move back to line up with mogo
                       true);
  chassis.pid_wait();

 // chassis.pid_turn_set(0, 90); // Turn so clamp is facing mogo
  //chassis.pid_wait();

  lift.move_absolute(-100, 60);

//  chassis.pid_odom_set({{{-47.5_in, 6_in}, rev, 80}}, // Move into mogo
  //                     true);
//  chassis.pid_wait();

 // mogo.set(true);

 // pros::delay(500);

//  chassis.pid_turn_set(180, 100); // Turn back 
  //chassis.pid_wait();

  pros::delay(200);

  lift.move_absolute(-100, 80);

  chassis.pid_odom_set({{{-50_in, -28_in}, fwd, 80},
                        {{-50_in, -37_in}, fwd, 40}}, // Move to high stake ring stack
                       true);
  chassis.pid_wait();

  pros::delay(800);

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

  chassis.pid_odom_set({{{-23.5_in, -47_in}, fwd, 80}}, // Get 2nd platform ring
                       true);
  chassis.pid_wait(); 

  

  

  lift.set_current_limit_all(2500);
  lift.move_absolute(-2000, 75); 

  chassis.pid_turn_set(49, 60); // Turn to ladder
  chassis.pid_wait();
  
  pros::delay(500);
  

  chassis.pid_odom_set({{{-11.5_in, -36_in}, fwd, 60}}, // Move to ladder //11.5,-33
                       true);
  chassis.pid_wait();

  chassis.pid_drive_set(5.5_in, 60);
  chassis.pid_wait();

  intake.brake();

  chassis.pid_turn_set(49, 70); // Turn to ladder
  chassis.pid_wait();

  pros::delay(500);
  platform.set_value(false);
  pros::delay(1000);
  ladder_arm.move_relative(500, 100);

  lift.set_current_limit_all(2500);
  lift.move_absolute(-3200, 75); 
  pros::delay(2000);

  lift.move_absolute(-2700, 75); 
  pros::delay(500);
  lift.move_absolute(-3200, 75); 



  


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
  dock.set_value(false); // Activate docking
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
//
// High Stake and Climb 30 point Skills
//
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
  dock.set_value(false); // Activate docking
  pros::delay(500);

  intake.move_velocity(-200); // Begin intaking

  chassis.pid_odom_set({{{-44_in, 4_in}, fwd, 110}, // First red ring intake
                        {{-24_in, -24_in}, fwd, 110}, // Second red ring intake
                        {{0_in, -47_in}, fwd, 70}}, // Line up for 1st mogo
                       true);
  chassis.pid_wait();

  intake.brake();

  // Pickup 3rd ring (optional)
  // intake.set_current_limit(2500);
  // intake.move_velocity(-200);
  // pros::delay(100);

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

//
//4 mobile goals with climb
//
void twoMobilewithclimb(){
  platform.set_value(true);               // Prepare platform
  chassis.pid_targets_reset();            // Reset PID targets to 0
  chassis.drive_imu_reset();              // Reset gyro (IMU) position
  chassis.drive_sensor_reset();           // Reset drive sensors
  chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);  // Set motors to hold

  chassis.odom_pose_set({-56.8_in, -24_in, 180_deg});
  dock.set_value(true);
  chassis.pid_odom_set({{{-56.8_in, -27_in}, fwd, 80}}, // Move forward to dock
                       true);
  chassis.pid_wait();

  dock.set_value(false);
  mogo.set(true);
    lift.move_absolute(-500, 75); 

  pros::delay(1000);
  chassis.pid_odom_set({{{-29_in, -23_in}, fwd, 110}, // First red ring intake
                         {{0_in, -40_in}, fwd, 110}, 
                        {{-15_in, -45_in}, rev, 110}, // Second red ring intake
                        {{-55_in, -57_in}, rev, 70}}, 
                       true);
  chassis.pid_wait();
  chassis.pid_odom_set({{{-33_in, -44_in}, fwd, 80}}, true); 
  chassis.pid_wait();

  intake.move_velocity(-200); 

  chassis.pid_odom_set({{{-47_in, 0_in}, fwd, 110}, 
                        {{-24_in, 24_in}, fwd, 110}},true); 
  chassis.pid_wait();
  intake.brake();

  chassis.pid_odom_set({{{9_in, 48_in}, fwd, 110}, 
                        {{-8_in, 49_in}, rev, 110}, 
                        {{-25_in, 49_in}, rev, 70}, 
                        {{-53_in, 60_in}, rev, 70}, 
                        {{-30_in, 15_in}, fwd, 40}},
                       true);
  chassis.pid_wait();

 platform.set_value(false);
  pros::delay(2000);
  ladder_arm.move_relative(500, 100);

  lift.set_current_limit_all(2500);
  lift.move_absolute(-3200, 75); 

  pros::delay(5000);

  lift.move_absolute(-2700, 75); 

  pros::delay(4000);

  lift.move_absolute(-3200, 75); 
  lift_brake.set(false);

}


void fortySevenPointSkills(){
  // -56, -15 - 24in
  // -56, -19 - dock with 15 in
  // 0, -38 - reverse at this point

  platform.set_value(true);               // Prepare platform
  chassis.pid_targets_reset();            // Reset PID targets to 0
  chassis.drive_imu_reset();              // Reset gyro (IMU) position
  chassis.drive_sensor_reset();           // Reset drive sensors
  chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);  // Set motors to hold
  //Startup Position and Docking mech
    chassis.odom_pose_set({-56_in, -15_in, 180_deg});
    dock.set_value(true);
    chassis.pid_odom_set({{{-56_in, -19_in}, fwd, 80}}, // Move forward to dock
                        true);
    chassis.pid_wait();

    dock.set_value(false);
    mogo.set(true);
    lift.move_absolute(-500, 75); 

    pros::delay(1000);

    //Driving to put the first mogo in corner 
    chassis.pid_odom_set({{{-27_in, -27_in}, fwd, 110}, //passing mogo
                          {{0_in, -40_in}, fwd, 110}, //about to reverse
                          {{-15_in, -45_in}, rev, 110}, //running into the mogo and pushing it into the corner
                          {{-51_in, -57_in}, rev, 70}}, //Mogo should be in corner
                        true);
    chassis.pid_wait();

    chassis.pid_odom_set({{{0_in, -50_in}, fwd, 110}, // Going for 2nd mogo
                        {{18_in, -25_in}, fwd, 110}, //lining up to the second mogo
                          {{50_in, -51_in}, fwd, 70}}, //Mogo should be in corner
                        true);
    chassis.pid_wait();

    chassis.pid_odom_set({{{44_in, -35_in}, rev, 110}, //backup
                          {{46_in, -5_in},fwd, 110}, //running into the third mogo and pushing it into the corner
                          {{57_in, 56_in}, rev, 110}}, //Mogo should be in corner
                        true);
    chassis.pid_wait();

    chassis.pid_odom_set({{{40_in, 47_in}, fwd, 110}}, //backup
                        true);
    chassis.pid_wait();
    chassis.pid_odom_set({{{9_in, 48_in}, fwd, 110}, 
                          {{-8_in, 49_in}, rev, 110}, 
                          {{-25_in, 49_in}, rev, 70}, 
                          {{-53_in, 60_in}, rev, 70}, 
                          {{-8_in, 35_in}, fwd, 40}},
                        true);
    chassis.pid_wait();
  //   //Intaking first ring 
  //   intake.move_velocity(-200); // Begin intaking

  //   chassis.pid_odom_set({{{4_in, 47_in}, fwd, 110}}, //First ring to intake
  //                        true);
  //   chassis.pid_wait();
  //   intake.brake();

  //   chassis.pid_odom_set({{{-15_in, 47_in}, fwd, 110}, //Lining up for final Mogo
  //                          {{-54_in, -55_in}, fwd, 110}}, //mogo should be in corner
  //                        true);
  //   chassis.pid_wait();
  //   //Intaking second ring 
  //   intake.move_velocity(-200); // Begin intaking

  //   chassis.pid_odom_set({{{-24_in, 24_in}, fwd, 110}}, //second ring to intake
  //                        true);
  //   chassis.pid_wait();
  //   intake.brake();

  //   //Climbing
  //   chassis.pid_odom_set({{{-8_in, 35_in}, fwd, 110}}, //Lining up to climb
  //                        true);
  //   chassis.pid_wait();

  // //Need to put Climbing code 
  //   pros::delay(600);

  //   ladder_arm.move(100);
  //   pros::delay(500);
  //   ladder_arm.brake();

  //   platform.set_value(false);

  //   lift.move_velocity(-60);
  //   while (lift.get_position(2) > -3250){
  //     pros::delay(10);
  //   }
  //   lift.brake();
  //   lift_brake.set(false);


}
void fiftyNineSkills() {
    // Prepare platform
    platform.set_value(true);   
    
    // Reset chassis settings
    chassis.pid_targets_reset();                
    chassis.drive_imu_reset();                  
    chassis.drive_sensor_reset();               
    chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD); 


  // Start at the 24” robot's starting position
  chassis.odom_pose_set({-54_in, -14_in, 180_deg});  // 24” robot's start position
  chassis.pid_odom_set({{{-54_in, -18_in}, fwd, 60}}, true); // Dock with second robot
  chassis.pid_wait();
  //Lining up for 1st ring and picking it up
  chassis.pid_odom_set({{{-38_in, -36_in}, fwd, 70}}, true);
  chassis.pid_wait();
  pros::delay(500);
    intake.set_current_limit(2500);
    intake.move_velocity(-200);
    pros::delay(100);
    lift.set_current_limit_all(2500);
    lift.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    lift.move_absolute(-500, 75);
  // First Mobile goal with ring
  chassis.pid_odom_set({{{47_in, 47_in}, fwd, 70}, 
                        {{35_in, 47_in}, fwd, 70}}, 
                        true);
  //Clamp mech for mobile goal - pick it up
  //Place ring on mobile goal code
  chassis.pid_odom_set({{{-53_in, -55_in}, fwd, 70}}, true);
  //Lining up for 2nd ring and picking it up
  chassis.pid_odom_set({{{-13_in, -49_in}, fwd, 70}}, true);
  chassis.pid_wait();
  pros::delay(500);
  intake.set_current_limit(2500);
  intake.move_velocity(-200);
  pros::delay(100);

  lift.set_current_limit_all(2500);
  lift.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
  lift.move_absolute(-500, 75);
  // Second Mobile goal with ring
  chassis.pid_odom_set({{{13_in, -31_in}, fwd, 70}}, true);
  //Clamp mech for mobile goal - pick it up
  //Place ring on mobile goal code
  chassis.pid_odom_set({{{34_in, -43_in}, fwd, 70}, // Updated 1st coordinate
                        {{53_in, -57_in}, fwd, 70}}, // Updated 2nd coordinate
                        true);
  //Clamp mech for mobile goal - Drop it in corner
  chassis.pid_odom_set({{{57_in, -48_in}, fwd, 70}}, true);

  // lining up for 3rd Ring/Stack
  chassis.pid_odom_set({{{52_in, -37_in}, fwd, 70}}, true);
  chassis.pid_wait();
  pros::delay(500);
  intake.set_current_limit(2500);
  intake.move_velocity(-200);
  pros::delay(100);
  lift.set_current_limit_all(2500);
  lift.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
  lift.move_absolute(-500, 75);
  // Third Mobile goal with ring/stack
  chassis.pid_odom_set({{{47_in, -13_in}, fwd, 70}}, true);
  //Clamp mech for mobile goal - pick it up
  //Place ring on mobile goal code
  chassis.pid_odom_set({{{58_in, 53_in}, fwd, 70}}, true);
  //Clamp mech for mobile goal - Drop it in corner
  chassis.pid_odom_set({{{26_in, 58_in}, fwd, 70}}, true);
  // lining up for 4th Ring
  chassis.pid_odom_set({{{12_in, 52_in}, fwd, 70}}, true);
  chassis.pid_wait();
  pros::delay(500);
  intake.set_current_limit(2500);
  intake.move_velocity(-200);
  pros::delay(100);
  lift.set_current_limit_all(2500);
  lift.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
  lift.move_absolute(-500, 75);
  // Fourth Mobile goal with ring/stack
  chassis.pid_odom_set({{{-10_in, 48_in}, fwd, 70}}, true);
  //Clamp mech for mobile goal - pick it up
  //Place ring on mobile goal code
  chassis.pid_odom_set({{{-52_in, 59_in}, fwd, 70}}, true);
  //Clamp mech for mobile goal - Drop it in corner
  chassis.pid_odom_set({{{-55_in, 56_in}, fwd, 70}}, true);

  intake.move_velocity(-200); // Begin intaking

  chassis.pid_odom_set({{{-47_in, 47_in}, fwd, 110}, // First red ring intake
                        {{-24_in, 24_in}, fwd, 110}}, // Second red ring intake
                        true);
  chassis.pid_wait();
  intake.brake();

  //Climbing positions
  chassis.pid_odom_set({{{-13_in, 39_in}, fwd, 70}, // Line up for climb
                          {{-9_in, 34_in}, rev, 70},},true); // Go for climb

  // Climb ( this is code for high stake - take out that code) 
  platform.set_value(false);
  pros::delay(2000);
  ladder_arm.move_relative(500, 100); 
  lift.set_current_limit_all(2500); 
  lift.move_absolute(-3200, 75);
  pros::delay(5000); 
  lift.move_absolute(-2800, 75);
  pros::delay(4000); 
  lift.move_absolute(-3200, 75);
}

void sixtyTwoSkills() {
  // Prepare platform
  platform.set_value(true);

  // Reset chassis settings
  chassis.pid_targets_reset();
  chassis.drive_imu_reset();
  chassis.drive_sensor_reset();
  chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD); 

  // 1. Place 1st mobile goal on the wall stake (assumed coordinates for wall)
  chassis.odom_pose_set({-65_in, -65_in, 0_deg}); // Assuming -65, -65 for the wall stake location
  chassis.pid_odom_set({{{-65_in, -65_in}, fwd, 60}}, true); // Move to wall stake
  chassis.pid_wait();

  // Place the mobile goal on the wall stake (intake or drop method, if applicable)
  intake.move_velocity(-200); // Start intaking or place mobile goal on the stake
  pros::delay(500);           // Wait for the intake process
  intake.brake();             // Stop the intake or disengage the mobile goal

  // Move to the 1st mobile goal position
  chassis.pid_odom_set({{{-54_in, -19_in}, fwd, 60}}, true); // Move to first mobile goal
  chassis.pid_wait();

  // 2. Place 1st mobile goal with ring
  intake.move_velocity(-200); // Start intaking first ring
  pros::delay(500);           // Wait for the intake to complete
  intake.brake();             // Stop intake

  // Move 1st mobile goal to 1st corner
  chassis.pid_odom_set({{{-54_in, -28_in}, fwd, 60}}, true); // Move to first corner
  chassis.pid_wait();

  // 3. Place 2nd mobile goal with ring
  chassis.odom_pose_set({-54_in, -28_in, 0_deg});  // Move to second goal position
  chassis.pid_odom_set({{{-54_in, -28_in}, fwd, 60}}, true);
  chassis.pid_wait();
  intake.move_velocity(-200); // Start intaking second ring
  pros::delay(500);           // Wait for the intake
  intake.brake();             // Stop intake

  // Move mobile goal to second corner
  chassis.pid_odom_set({{{-47_in, -47_in}, fwd, 60}}, true); // Move to second corner
  chassis.pid_wait();

  // 4. Place 3rd mobile goal with ring
  chassis.odom_pose_set({-47_in, -47_in, 0_deg});  // Move to third goal position
  chassis.pid_odom_set({{{-47_in, -47_in}, fwd, 60}}, true);
  chassis.pid_wait();
  intake.move_velocity(-200); // Start intaking third ring
  pros::delay(500);           // Wait for the intake
  intake.brake();             // Stop intake

  // Move mobile goal to third corner
  chassis.pid_odom_set({{{-24_in, -48_in}, fwd, 60}}, true); // Move to third corner
  chassis.pid_wait();

  // 5. Place 4th mobile goal with ring
  chassis.odom_pose_set({-24_in, -48_in, 0_deg});  // Move to fourth goal position
  chassis.pid_odom_set({{{-24_in, -48_in}, fwd, 60}}, true);
  chassis.pid_wait();
  intake.move_velocity(-200); // Start intaking fourth ring
  pros::delay(500);           // Wait for the intake
  intake.brake();             // Stop intake

  // Move mobile goal to fourth corner
  chassis.pid_odom_set({{{-65_in, -65_in}, fwd, 60}}, true); // Move to fourth corner
  chassis.pid_wait();

  // 6. Final move to the climbing position (-3, 24)
  chassis.odom_pose_set({-3_in, 24_in, 0_deg});  // Move to climbing position
  chassis.pid_odom_set({{{-3_in, 24_in}, fwd, 70}}, true);  // Level 3 climb setup
  chassis.pid_wait();

  // Start Level 3 climb (e.g., lift robot)
  platform.set_value(false);   // Assuming platform lowers to help with climb
  pros::delay(1000);           // Allow climb time
}
void sixtyFiveSkills() {
  // Prepare platform
  platform.set_value(true);

  // Reset chassis settings
  chassis.pid_targets_reset();
  chassis.drive_imu_reset();
  chassis.drive_sensor_reset();
  chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD); 

  // 1. Place the first ring on the first wall stake (coordinates for wall stake 1)
  chassis.odom_pose_set({-65_in, -65_in, 0_deg}); // Assuming coordinates (-65, -65) for wall stake 1
  chassis.pid_odom_set({{{-65_in, -65_in}, fwd, 60}}, true); // Move to first wall stake
  chassis.pid_wait();

  // Place the first ring on the wall stake
  intake.move_velocity(-200); // Start intaking first ring
  pros::delay(500);           // Wait for intake
  intake.brake();             // Stop intake

  // Move to 1st mobile goal position
  chassis.pid_odom_set({{{-54_in, -19_in}, fwd, 60}}, true); // Move to first mobile goal
  chassis.pid_wait();

  // 2. Place the 1st mobile goal with the ring and move it to the 1st corner
  intake.move_velocity(-200); // Start intaking first ring (if needed for mobile goal)
  pros::delay(500);           // Wait for intake
  intake.brake();             // Stop intake

  // Move the mobile goal to the 1st corner
  chassis.pid_odom_set({{{-54_in, -28_in}, fwd, 60}}, true); // Move to first corner
  chassis.pid_wait();

  // 3. Place the second ring on the second wall stake
  chassis.odom_pose_set({-65_in, -65_in, 0_deg}); // Move to second wall stake position
  chassis.pid_odom_set({{{-47_in, -47_in}, fwd, 60}}, true); // Move to second wall stake
  chassis.pid_wait();

  // Place the second ring on the second wall stake
  intake.move_velocity(-200); // Start intaking second ring
  pros::delay(500);           // Wait for intake
  intake.brake();             // Stop intake

  // 4. Place the 2nd mobile goal with the ring and move it to the 2nd corner
  chassis.odom_pose_set({-47_in, -47_in, 0_deg});  // Move to second goal position
  chassis.pid_odom_set({{{-47_in, -47_in}, fwd, 60}}, true);
  chassis.pid_wait();
  intake.move_velocity(-200); // Start intaking second ring (if needed for mobile goal)
  pros::delay(500);           // Wait for intake
  intake.brake();             // Stop intake

  // Move mobile goal to second corner
  chassis.pid_odom_set({{{-24_in, -48_in}, fwd, 60}}, true); // Move to second corner
  chassis.pid_wait();

  // 5. Place the 3rd mobile goal with the ring and move it to the 3rd corner
  chassis.odom_pose_set({-24_in, -48_in, 0_deg});  // Move to third goal position
  chassis.pid_odom_set({{{-24_in, -48_in}, fwd, 60}}, true);
  chassis.pid_wait();
  intake.move_velocity(-200); // Start intaking third ring (if needed)
  pros::delay(500);           // Wait for intake
  intake.brake();             // Stop intake

  // Move mobile goal to third corner
  chassis.pid_odom_set({{{-65_in, -65_in}, fwd, 60}}, true); // Move to third corner
  chassis.pid_wait();

  // 6. Place the 4th mobile goal with the ring and move it to the 4th corner
  chassis.odom_pose_set({-65_in, -65_in, 0_deg});  // Move to fourth goal position
  chassis.pid_odom_set({{{-65_in, -65_in}, fwd, 60}}, true);
  chassis.pid_wait();
  intake.move_velocity(-200); // Start intaking fourth ring
  pros::delay(500);           // Wait for intake
  intake.brake();             // Stop intake

  // Move mobile goal to fourth corner
  chassis.pid_odom_set({{{0_in, -47_in}, fwd, 60}}, true); // Move to fourth corner (update as per field design)
  chassis.pid_wait();

  // 7. Final move to the climbing position (-3, 24)
  chassis.odom_pose_set({-3_in, 24_in, 0_deg});  // Move to climbing position
  chassis.pid_odom_set({{{-3_in, 24_in}, fwd, 70}}, true);  // Level 3 climb setup
  chassis.pid_wait();

  // Start Level 3 climb (e.g., lift robot)
  platform.set_value(false);   // Assuming platform lowers to help with climb
  pros::delay(1000);           // Allow climb time
}

void red_match_auton(){
  platform.set_value(true);
  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);   // Set motors to hold.  This helps autonomous consistency
  chassis.odom_pose_set({-56.8_in, -24_in, 180_deg});

  dock.set_value(true);
  chassis.pid_odom_set({{{-56.8_in, -27_in}, fwd, 80}}, // Move forward to dock
                       true);
  chassis.pid_wait();

  dock.set_value(false);

  pros::delay(1000);

  chassis.pid_odom_set({{{-52.4_in, -37_in}, fwd, 60}}, // Move to grab stack
                       true);
  chassis.pid_wait();

  pros::delay(500);
  intake.set_current_limit(2500);
  intake.move_velocity(-200);
  pros::delay(100);

  lift.set_current_limit_all(2500);
  lift.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
  lift.move_absolute(-500, 75);


  chassis.pid_odom_set({{{-24_in, -24_in}, fwd, 80}}, // Intake 1st blue platform ring
                       true);
  chassis.pid_wait();

  chassis.pid_odom_set({{{-23.5_in, -47_in}, fwd, 80}}, // Get 2nd platform ring
                       true);
  chassis.pid_wait(); 

  

  lift.set_current_limit_all(2500);
  lift.move_absolute(-2000, 75); 
  pros::delay(1000);

  chassis.pid_odom_set({{{-7_in, -30.5_in}, fwd, 80}}, // Move to ladder
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

  dock.set_value(true);
  chassis.pid_odom_set({{{56.8_in, -27_in}, fwd, 80}}, // Move forward to dock
                       true);
  chassis.pid_wait();

  dock.set_value(false);

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