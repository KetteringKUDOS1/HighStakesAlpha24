#include "main.h"
#include <cstdio>
#include "EZ-Template/util.hpp"
#include "autons.hpp"
#include "pros/device.hpp"
#include "pros/imu.h"
#include "pros/imu.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/optical.h"
#include "pros/rtos.hpp"
#include "subsystems.hpp"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// Chassis constructor
ez::Drive chassis(
    // These are your drive motors, the first motor is used for sensing!
    {11, -12, 13, -14, -15},    // Left Chassis Ports (negative port will reverse it!)
    {20, -19, 18, -17, 4},    // Right Chassis Ports (negative port will reverse it!)
    21,      // IMU Port
    3.25,  // Wheel Diameter (Remember, 4" wheels without screw holes are actually 4.125!)
    360);   // Wheel RPM

// Are you using tracking wheels?  Comment out which ones you're using here!
//  `2.75` is the wheel diameter
//  `4.0` is the distance from the center of the wheel to the center of the robot
 ez::tracking_wheel right_tracker({'C', 'D'}, 2.75, 4.7);  // ADI Encoders
 ez::tracking_wheel left_tracker({-'E', -'F'}, 2.75, 4.7);  // ADI Encoders plugged into a Smart port
//ez::tracking_wheel horiz_tracker(1, 2.75, 4.0);             // Rotation sensors


// Set arm motors to hold when braking, preventing back driving
// WARNING: 
// Do not leave motors on hold when under load for long periods,
// they will draw a lot of current and overheat


// Slew configuration\

bool arm_slew_active = false;
int arm_dir = 0;
bool arm_moving = false;
bool arm_decelerating = false;
bool arm_accelerating = false;
float arm_decel_velocity = 0;
float arm_accel_velocity = 0;
bool on_rings = false;
bool unclimbing = false;
uint32_t start_time;
uint32_t end_time;
uint32_t duration;

int target_rpm = 0;
int current_rpm = 0;
int rpm_increment = 10;  // Adjust this value for smoother or faster transitions


//Move Arm Function 
void move_arm(int target_rpm) {
    arm_moving = true;
    arm_slew_active = true;
    arm_dir = (target_rpm > 0) ? 1 : -1;

    lift.move_velocity(target_rpm);
}

//Home Arm to the Floor
void home_arm(){
  int current_limit = 1800;

  // Run homing for 0.5 seconds
  // This allows for a consistent homing time and ensures it will exit even if the current limit isnt met
  for(int i = 0; i < 50; i++){ //TODO: SHORTEN THE LOOPS FOR AUTON, NOT INITIALIZE
    if (lift.get_current_draw() < current_limit){
    //move_arm(40);
    lift.move(40);
    }
    else{
      lift.brake();
    }
    printf("Homing current %d\n", lift.get_current_draw());
    pros::delay(ez::util::DELAY_TIME);
  }
  lift.brake();
  lift.tare_position_all();

  return;

}

bool lift_task_enabled = false;
bool climb_task_enabled = false;
bool driveSafe_task_enabled = false;
bool deClimb_task_enabled = false;

void deClimb() {
  driveSafe_task_enabled = false;
  climb_task_enabled = false;
  if(deClimb_task_enabled == true){  
    //Sets the Curent Limits 
    lift.set_current_limit_all(2500);
    ladder_arm.set_current_limit(2500);
    //Lift move up and Ladder arm go in
    lift.move_relative(-50, 50); //Auton ends at -3500
    ladder_arm.move_absolute(0, 100); 
    ladder_arm.move_velocity(100);
    pros::delay(250);

    //Lift goes down 
    lift.move_absolute(-1000, 200);
    pros::delay(500);

    //Ladder arm stops and platform goes goes down so we are back on the ground
    ladder_arm.brake();
    pros::delay(500);
    platform.set_value(true);
    on_rings = false;
  }
  deClimb_task_enabled = false;
  ladder_arm.set_current_limit(0);
}

void lift_task(){
  pros::delay(2000);
  while (true){
    if (lift_task_enabled){
      lift.set_current_limit_all(2500);
      lift.move_absolute(-1950, 75); //2000
      printf("Lift Pos 1: %f\n", lift.get_position());
      if (lift.get_position() < -1850){
        printf("Done with lifting 1\n");
        //pros::delay(500);
        lift_task_enabled = false;
      }
    }
  if (climb_task_enabled){
      platform.set_value(false);
      on_rings = true;
      lift.set_current_limit_all(2500);
      lift.move_absolute(-3525, 90);
      ladder_arm.move_absolute(-1000, 100);
      lift_brake.set(false);
      printf("Lift Pos 2: %f\n", lift.get_position());
      if (lift.get_position() < -3400){
        printf("Done with lifting 2\n");
        climb_task_enabled = false;
      }
    }
  }
    pros::delay(ez::util::DELAY_TIME);
}

pros::Task LiftTask(lift_task);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  // Print our branding over your terminal :D
  ez::ez_template_print();

  pros::delay(500);  // Stop the user from doing anything while legacy ports configure

  // Are you using tracking wheels?  Comment out which ones you're using here!
  right_tracker.ticks_per_rev_set(8192);
  left_tracker.ticks_per_rev_set(8192);
  chassis.odom_tracker_right_set(&right_tracker);
  chassis.odom_tracker_left_set(&left_tracker);
  // chassis.odom_tracker_back_set(&horiz_tracker);
  
  // Configure your chassis controls
  chassis.opcontrol_curve_buttons_toggle(true);  // Enables modifying the controller curve with buttons on the joysticks
  chassis.opcontrol_drive_activebrake_set(0);    // Sets the active brake kP. We recommend ~2.  0 will disable.
  chassis.opcontrol_curve_default_set(0, 0);     // Defaults for curve. If using tank, only the first parameter is used. (Comment this line out if you have an SD card!)

  // Set the drive to your own constants from autons.cpp!
  default_constants();

  // These are already defaulted to these buttons, but you can change the left/right curve buttons here!
  // chassis.opcontrol_curve_buttons_left_set(pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT);  // If using tank, only the left side is used.
  // chassis.opcontrol_curve_buttons_right_set(pros::E_CONTROLLER_DIGITAL_Y, pros::E_CONTROLLER_DIGITAL_A);
  // Autonomous Selector using LLEMU



  //NEED TO GO THRU AUTON SELCTIOSN TO GET RID OF SOME

  ez::as::auton_selector.autons_add({
    //Worlds Autons
      Auton("Red Auton: Worlds", Red_Worlds),
      //Auton("Blue Auton: Worlds", Blue_Worlds),
      //Auton("MSOE: Worlds", MSOE),
     // Auton("MSOE No ODOM", No_Odom_MSOE),
    //Previous Competitions Codes
      //Auton("purdue skills",purdueSkills),
      //Auton("Red AWP Test", red_AWP_match),
      //Auton("Blue AWP Test", blue_AWP_match),


    //Auton("Test Ladder", ladder_arm_test),



    //Tuning Codes
      //Auton("tuning",tuning),
      //Auton("testTune",testTune),

    // PROS EXAMPLES
      //Auton("Example Turn\n\nTurn 3 times.", turn_example),
      //Auton("Drive and Turn\n\nDrive forward, turn, come back. ", drive_and_turn),
      //Auton("Drive and Turn\n\nSlow down during drive.", wait_until_change_speed),
      //Auton("Swing Example\n\nSwing in an 'S' curve", swing_example),
      //Auton("Motion Chaining\n\nDrive forward, turn, and come back, but blend everything together :D", motion_chaining),
      //Auton("Combine all 3 movements", combining_movements),
      //Auton("Interference\n\nAfter driving forward, robot performs differently if interfered or not.", interfered_example),
  });

  // Initialize chassis and auton selector
  chassis.initialize();
  ez::as::initialize();

  master.rumble(".");
  platform.set_value(true);
  lift.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
  
  lift_brake.set(true);
  lift.move_relative(-50, 50); //200

  pros::delay(500); // TODO: SHORTEN DELAY
  home_arm();
  pros::delay(200);

  //ladder_arm.move_relative(-200, 100);
  pros::delay(200);
  intake.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE); 


  ladder_arm.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  ladder_arm.brake();
  ladder_arm.tare_position();

  
  dock.set_value(true); //driver is false auton is true 
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
  // . . .
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
  // . . .
   
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */

  ez::slew lift_slew;


void autonomous() {
  //lift.move_absolute(-2000, 75);
  //pros::delay(9999999);
  home_arm();
  //master.print(0,0,"%lf",chassis.odom_theta_get());

 /* for (int i = 0; i < 100 - 1; i++) {
  master.print(0,0,"%lf",chassis.get_value(degrees));
  }*/

  ez::as::auton_selector.selected_auton_call();  // Calls selected auton from autonomous selector

  /*
    chassis.pid_targets_reset();                // Resets PID targets to 0
    chassis.drive_imu_reset();                  // Reset gyro position to 0
    chassis.drive_sensor_reset();               // Reset drive sensors to 0
    chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Set motors to hold.  This helps autonomous consistency
    chassis.odom_pose_set({0_in, 0_in, 0_deg});
    chassis.drive_width_set(15.3_in);  // Measure this with a tape measure
    lift.tare_position(2);  // Resets lift position

    
    chassis.pid_odom_set({{{0_in, 4_in}, fwd, 40}}, // Move forward to dock
                        true);
    chassis.pid_wait();

    pros::delay(500);
    dock.set_value(true); // Activate docking
    pros::delay(500);

    lift.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD); // Change arm to hold position when stopped

    move_arm(-100); // Move arm up to bring 15" off the ground
    pros::delay(300);
    lift.brake();

    pros::delay(300);

    intake.move_velocity(-200); // Begin intaking

    chassis.pid_odom_set({{{-12_in, 30_in}, fwd, 110}, // Move to intake first ring
                          {{-12_in, 36_in}, fwd, 110}, // Avoid ladder
                          {{-37_in, 53_in}, fwd, 80}}, // Move to intake second ring
                        true);
    chassis.pid_wait();

    pros::delay(500);
    intake.brake(); // Stop intaking

    move_arm(100); // Move arm down to bring 15" to the ground
    pros::delay(300);
    lift.brake();

    chassis.pid_odom_set({{{-23_in, 69_in}, fwd, 110}}, // Move to third ring
                        true);
    chassis.pid_wait();

    pros::delay(2000); // Wait for 15" to grab it

    chassis.pid_odom_set({{{-52_in, 50_in}, fwd, 110}}, // Move towards ladder
                        true);
    chassis.pid_wait();
    */
  
}

/**
 * Gives you some extras to run in your opcontrol:
 * - run your autonomous routine in opcontrol by pressing DOWN and B
 *   - to prevent this from accidentally happening at a competition, this
 *     is only enabled when you're not connected to competition control.
 * - gives you a GUI to change your PID values live by pressing X
 */
void ez_template_etxras() {
  if (!pros::competition::is_connected()) {
    // PID Tuner
    // - after you find values that you're happy with, you'll have to set them in auton.cpp

    // Enable / Disable PID Tuner
    //  When enabled:
    //  * use A and Y to increment / decrement the constants
    //  * use the arrow keys to navigate the constants
    if (master.get_digital_new_press(DIGITAL_X))
      chassis.pid_tuner_toggle();

    // Trigger the selected autonomous routine
    if (master.get_digital(DIGITAL_B) && master.get_digital(DIGITAL_DOWN)) {
      pros::motor_brake_mode_e_t preference = chassis.drive_brake_get();
      autonomous();
      chassis.drive_brake_set(preference);
    }

    // Blank pages for odom debugging
    if (chassis.odom_enabled() && !chassis.pid_tuner_enabled()) {
      // This is Blank Page 1, it will display X, Y, and Angle
      if (ez::as::page_blank_is_on(0)) {
        screen_print("x: " + std::to_string(chassis.odom_x_get()) +
                         "\ny: " + std::to_string(chassis.odom_y_get()) +
                         "\nangle: " + std::to_string(chassis.odom_theta_get()),
                     1);  // Don't override the top Page line
      }
      // This is Blank Page 2, it will display every tracking wheel.
      // Make sure the tracking wheels read POSITIVE going forwards or right.
      else if (ez::as::page_blank_is_on(1)) {
        if (chassis.odom_tracker_left != nullptr)
          screen_print("left tracker: " + std::to_string(chassis.odom_tracker_left->get()), 1);
        else
          screen_print("no left tracker", 1);

        if (chassis.odom_tracker_right != nullptr)
          screen_print("right tracker: " + std::to_string(chassis.odom_tracker_right->get()), 2);
        else
          screen_print("no right tracker", 2);

        if (chassis.odom_tracker_back != nullptr)
          screen_print("back tracker: " + std::to_string(chassis.odom_tracker_back->get()), 3);
        else
          screen_print("no back tracker", 3);

        if (chassis.odom_tracker_front != nullptr)
          screen_print("front tracker: " + std::to_string(chassis.odom_tracker_front->get()), 4);
        else
          screen_print("no front tracker", 4);
      }
    }

    chassis.pid_tuner_iterate();  // Allow PID Tuner to iterate
  } else {
    // Remove all blank pages when connected to a comp switch
    if (ez::as::page_blank_amount() > 0)
      ez::as::page_blank_remove_all();

    // Disable PID tuner
    if (chassis.pid_tuner_enabled())
      chassis.pid_tuner_disable();
  }
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */



// Driver Control Code
  int climb_status = 0;
  bool isGearLocked = true;
  bool isLadderArmOut = true;
void opcontrol() {
  // This is preference to what you like to drive on
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);
  bool mogo_toggle = false;
  bool mogo_pressed = false;
  //chassis.pid_drive_set(3_in, 70);
  //chassis.pid_wait();
  // bool dock_toggle = false;
  // bool dock_pressed = false;
  
  isGearLocked = false;
  lift.move_relative(-150, 120);

  pros::Controller master(pros::E_CONTROLLER_MASTER);
  
  while (true) {
    // Gives you some extras to make EZ-Template easier
    // ez_template_etxras();
    master.print(0,0,"%lf",chassis.odom_theta_get());
 
    //chassis.opcontrol_tank();  // Tank control
    chassis.opcontrol_arcade_standard(ez::SPLIT);   // Standard split arcade
    // chassis.opcontrol_arcade_standard(ez::SINGLE);  // Standard single arcade
    // chassis.opcontrol_arcade_flipped(ez::SPLIT);    // Flipped split arcade
    // chassis.opcontrol_arcade_flipped(ez::SINGLE);   // Flipped single arcade
    
    // printf("DT Motor Amps: %2.f", chassis.drive_mA_left());

    // First Layer Controls
    if (!master.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
      //DeCLimb (X) 
      if(master.get_digital(pros::E_CONTROLLER_DIGITAL_X)){
        lift_brake.set(true);
        isGearLocked = false;
          deClimb_task_enabled = true;
          deClimb();
      }
      //UP Arrow needs to be Ladder arm retract so inwards
      if(master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)){
        ladder_arm.set_current_limit(2500);
        ladder_arm.move_velocity(100);
      }
      else {
        if (!lift_task_enabled && !deClimb_task_enabled){
          ladder_arm.set_current_limit(0);
          ladder_arm.brake();
        }    
      }
      // Intake rings (A)
      if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) { 
        if (abs(intake.get_target_velocity()) < 1){
          intake.set_current_limit(2500);
          intake.move_velocity(-200);
        }else{
          intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST); 
          intake.brake();
          pros::delay(10);
          intake.set_current_limit(0);  
        }
      }
      // Ring platform down (RIGHT Arrow)
      if (master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)){
        platform.set_value(false);
        on_rings = true;
      }
      // Ring platform up (Y)
      if (master.get_digital(pros::E_CONTROLLER_DIGITAL_Y)){
        platform.set_value(true);
        climb_task_enabled = false;
        on_rings = false;
      }
      // Auto Climb (DOWN Arrow)
      if (master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)){
        if (lift.get_position() >= -1900){
          printf("Running lift task\n");
          lift_task_enabled = true;
        }
        else if (lift.get_position() < -1600){
          printf("Running climb task\n");
          climb_task_enabled = true;
        }
      }
      // RIGHT needs to be the ratchet Retracts so Lock 
      if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A)){
        isGearLocked = true;
        lift_brake.set(false);
      }
      // A  needs to be the ratchet extends so Unlock 
      if (master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)){
        lift_brake.set(true);
        isGearLocked = false;
        unclimbing = true;
        lift.move_relative(-150, 50);
      }else{
          unclimbing = false;
      }

      // Mogo Rectract is R2 which means Mogo UP
      
      //Lift Down (L2)
      if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
        lift.set_current_limit_all(2500);
        lift.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
        if (!isGearLocked) { 
          lift.move_velocity(70);
        }else{
          lift.move_velocity(0);
        }
      }
      // Lift up (L1)
      if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
        if(lift.get_position() < -2000 && on_rings == false){
          lift.brake();
        }
        else if (lift.get_position() < -3550){
          lift.brake();
        }
        else{
        lift.set_current_limit_all(2500);
        lift.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
        lift_task_enabled = false;
        move_arm(-70); 
        } 
      }
      if(!master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)&& !master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
        // When no buttons are pressed, hold the arm in place
        lift.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
        if (!lift_task_enabled && !climb_task_enabled) {
          lift.brake();
        }
      } 
      // Mogo Rectract is R2 which means Mogo UP
      if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
        mogo.set(true);
      }
      // Mogo Extend is R1 which means Mogo Down
      if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
        mogo.set(false);
      }
    }
    else {       // Second Layer (when holding B)
      // Outtake rings (Left2)
      if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)){
        if (abs(intake.get_target_velocity()) < 1){
          intake.set_current_limit(2500);
          intake.move_velocity(200);
        }else{
          intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
          intake.brake();
          pros::delay(10);
          // Set current limit to 0 to prevent overcurrent draw when stationary
          intake.set_current_limit(0); 
        }
      }


      // L2 needs to be floor height of the DR4B
      if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
          if(isGearLocked == true){
            lift.brake();
        }else{
          if(isGearLocked == false){
              lift.set_current_limit_all(2500);
              lift.move_absolute(90, 80); 
          }
        }
      }
      // X needs to be Dock 
      if(master.get_digital(pros::E_CONTROLLER_DIGITAL_X)){
        dock.set_value(false);
      }
      // Mogo Rectract is R2 which means Mogo UP
      if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
        mogo.set(true);
      }
      // Mogo Extend is R1 which means Mogo Down
      if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
        mogo.set(false);
      }
      // A needs to be UnDock
      if(master.get_digital(pros::E_CONTROLLER_DIGITAL_A)){
        dock.set_value(true);
      }
      // A  needs to be the ratchet extends so Unlock 
      if (master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)){
        lift_brake.set(true);
        isGearLocked = false;
        unclimbing = true;
        lift.move_relative(-150, 50);
      }else{
          unclimbing = false;
      }
      // UP needs to be ladder arm extend so outwards
      if (master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)){
        ladder_arm.set_current_limit(2500);
        ladder_arm.move_velocity(-100);
      }else{
        if (!lift_task_enabled)
        {
          ladder_arm.set_current_limit(0);
          ladder_arm.brake();
        }    
      }
    }
    pros::delay(ez::util::DELAY_TIME);  // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}