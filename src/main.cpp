#include "main.h"
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
    10,      // IMU Port
    3.25,  // Wheel Diameter (Remember, 4" wheels without screw holes are ac4tually 4.125!)
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

// Stop all arm motors using configured brake mode

//Version Two - Working
void stop_arm() {
  // Set the current limit to prevent excessive current draw when stopping.
  lift.set_current_limit(200);

  // Set brake mode to HOLD to keep the arm in place only if needed.
  lift.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD); // Hold the arm in position

  // Prevents arm from moving down when below the lower threshold
  if (lift.get_position(2) > -30 && arm_dir == 1) {
    // Arm is near the bottom and trying to move down, stop movement and don't brake
    return;
  }

  // Prevents arm from moving up when above the upper threshold
  if (lift.get_position(2) < -3400 && arm_dir == -1) {
    // Arm is near the upper limit and trying to move up, stop movement
    return;
  }

  // If the arm is moving and slew is enabled, handle the deceleration
  if (arm_moving && arm_slew_active) {
    // Configure variables for deceleration
    if (!arm_decelerating) {
      arm_decelerating = true;
      arm_decel_velocity = lift.get_actual_velocity(2);  // Get current velocity
      start_time = end_time - 1;  // Small start time offset to avoid high initial acceleration
    }

    // Record time and calculate duration since last update
    end_time = pros::millis();
    duration = end_time - start_time;

    // If velocity is under 30, stop decelerating and stop the arm
    if (abs(arm_decel_velocity) < 30) {
      arm_decel_velocity = 0;
      arm_decelerating = false;
      arm_moving = false;
      lift.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);  // Hold position when done moving
    }

    // Limit velocity to a maximum value if it exceeds max RPM
    if (abs(arm_decel_velocity) > 200) {
      arm_decel_velocity = 200;
    }

    // Calculate direction and decrease velocity for smooth deceleration
    arm_dir = arm_decel_velocity / abs(arm_decel_velocity);
    arm_decel_velocity -= 20.0 * arm_dir * (duration / 30.0);  // Apply deceleration

    // Move the arm at the calculated deceleration velocity
    lift.move_velocity(arm_decel_velocity);

    // Update start time for the next loop
    start_time = pros::millis();
  }
  // If slew is not active, simply stop the arm
  else {
    if (unclimbing) {
      return;  // Do nothing if unclimbing flag is set
    }
    // Keep brake mode as HOLD and stop the arm if no movement is happening
    lift.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);  // Make sure the arm doesn't move if no button pressed
    lift.move_velocity(0); // Actually stop the arm movement
  }
}


//CONNORS VERSION
  // void stop_arm(){
  //   // Prevents arm from moving up when above the upper threshold
  //   lift.set_current_limit(200);
  //   lift.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE); //Brake 
  //   lift.set_current_limit(200);

  //   if (lift.get_position(2) < -3400){
  //     lift.brake();

  //     return;
  //   }
  //   // Prevents arm from moving down when below lower threshold
  //   else if(lift.get_position(2)> -30 && arm_dir == 1){
  //     //lift.brake();

  //     return;
  //   }

  //   // Check if the arm is moving and if we have slew enabled
  //   // Slew is the process of slowly accelerating/decelerating
  //   if (arm_moving && arm_slew_active){
  //     // Configure variables if the arm is beginning to decelerate
  //     if (arm_decelerating == false){
  //       arm_decelerating = true;
  //       arm_decel_velocity = lift.get_actual_velocity(2);
  //       // Initialize start time to be slightly smaller than end time to reduce initial acceleration
  //       start_time = end_time - 1;
  //     }
  //     // Record time to get how long since last function call
  //     end_time = pros::millis();

  //     // Calculate time elapsed since last time function was called
  //     duration = end_time - start_time;

  //     // Set it to zero and stop decelerating if velocity is under 30
  //     if (abs(arm_decel_velocity) < 30){
  //       arm_decel_velocity = 0;
  //       arm_decelerating = false;
  //       arm_moving = false;
  //       lift.brake();

  //     }

  //     // Fix velocity to maximum RPM if rpm is above the max
  //     if (abs(arm_decel_velocity) > 200){
  //       arm_decel_velocity = 200;
  //     }
  //     // Extract arm direction, -1 for down, 1 for up
  //     arm_dir = arm_decel_velocity/abs(arm_decel_velocity);

  //     // Decrease velocity by ~~660 RPM per second
  //     arm_decel_velocity -= 20.0*arm_dir*(duration/30.0);

  //     // Debug print statements
  //     printf("Vel: %f\n", arm_decel_velocity);
  //     //printf("Duration: %d\n", (duration));
  //     //printf("Calc: %f\n", (duration/30.0));
  //     //printf("Change: %f\n", 20.0*arm_dir*(duration/30.0));
  //     printf("Pos: %f\n", lift.get_position(2));

  //     // Move the arm at the calculated deceleration velocity  
  //     lift.move_velocity(arm_decel_velocity);

  //     // Record time to get how long since last function call
  //     start_time = pros::millis();
  //   }
  //   // Stop the arm if slew is not active or the arm is not moving
  //   else{
  //     if (unclimbing){
  //       return;
  //     }
  //     lift.brake();
  //   }

  // }


//Version 2
int target_rpm = 0;
int current_rpm = 0;
int rpm_increment = 10;  // Adjust this value for smoother or faster transitions


// DOUBLE CHECK IF THIS IS NECESSARY

void move_arm(int target_rpm) {
    // Gradually ramp to the target RPM for smoother movement
    static int current_rpm = 0;
    int ramp_rate = 5;  // Ramp rate for smooth acceleration and deceleration

    // Gradually adjust the RPM towards the target
    if (current_rpm < target_rpm) {
        current_rpm += ramp_rate;
        if (current_rpm > target_rpm) current_rpm = target_rpm; // Don't exceed target
    } else if (current_rpm > target_rpm) {
        current_rpm -= ramp_rate;
        if (current_rpm < target_rpm) current_rpm = target_rpm; // Don't go below target
    }

    // Set motor velocity to the current smooth RPM
    lift.move_velocity(current_rpm);

}


//Connors Version
  // Move the DR4B arm at the desired RPM
  // void move_arm(int rpm = 100){
  //   // Debug statement
    
  //   //lift.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD); //Brake
  //   // Save motors when lift is fully lowered
  //   // Otherwise set motors to hold
  //   if (lift.get_position(2) > 0){
      
  //     //lift.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  //   }
  //   else{
  //     //lift.set_current_limit_all(2500);
  //     // lift.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
  //   }

  //   // Extract most recent direction moved. -1 for down, 1 for up
  //   arm_dir = rpm/abs(rpm);

  //   // Debug print statements
  //   //printf("Direction: %i\n", dir);
  //   //printf("Current: %i\n", lift_left_2.get_current_draw());

  //   // Prevents arm from moving up when above the upper threshold
  //   if (lift.get_position(2) < -2000 && arm_dir == -1 && !on_rings){
  //     stop_arm();
  //     return;
  //   }
  //   if (lift.get_position(2) < -3200 && arm_dir == -1 && on_rings){
  //     //stop_arm();
  //     //return;
  //   }
  //   // Prevents arm from moving down when below lower threshold
  //   else if(lift.get_position(2) > -30 && arm_dir == 1){
  //     //stop_arm();
  //     return;
  //   }

  //   // Debug print statement
  //   //printf("Position: %f\n",lift_left_2.get_position());
    

  //   /// Configure variables if the arm is beginning to accelerate and slew is active
  //   // Slew is the process of slowly accelerating/decelerating
  //   if (arm_moving == false && arm_slew_active){
  //     arm_accelerating = true;
  //     // Initialize start time to be slightly smaller than end time to reduce initial acceleration
  //     start_time = end_time - 1;
  //     arm_accel_velocity = 0;
  //   }

  //   arm_moving = true;

  //   // Record time to get how long since last function call
  //   end_time = pros::millis();

  //   // Calculate time elapsed since last time function was called
  //   duration = end_time - start_time;


  //   if (arm_accelerating){
  //     // Increase the acceleration velocity if it is under the desired RPM
  //     if (abs(arm_accel_velocity) < abs(rpm)){
  //       // Increase velocity by ~660 RPM per second
  //       arm_accel_velocity += 20 * arm_dir * (duration/30.0);
  //     }

  //     // Catch if arm velocity is exceeding the desired RPM
  //     else if (abs(arm_accel_velocity) > abs(rpm)){
  //       // Fix velocity to the desired RPM
  //       arm_accel_velocity = rpm;

  //       // Stop accelerating
  //       arm_accelerating = false;
  //     }
  //     // Change RPM to use calculated velocity instead
  //     rpm = arm_accel_velocity;
  //   }

  //   // Move the lift motors at the desired RPM
  //   printf("MOVING ARM at: %d\n", rpm);
  //   lift.move_velocity(rpm);

  //   // Record time
  //   start_time = pros::millis();
  // }


void home_arm(){
  int current_limit = 900;

  // Run homing for 0.5 seconds
  // This allows for a consistent homing time and ensures it will exit even if the current limit isnt met
  for(int i = 0; i < 50; i++){ //TODO: SHORTEN THE LOOPS FOR AUTON, NOT INITIALIZE
    if (lift.get_current_draw() < current_limit){
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

void lift_task(){
  pros::delay(2000);
  while (true){
    if (lift_task_enabled){
      lift.set_current_limit_all(2500);
      lift.move_absolute(-2000, 75); //2000
      printf("Lift Pos 1: %f\n", lift.get_position());
      if (lift.get_position() < -1990){
        printf("Done with lifting 1\n");
        pros::delay(500);
        lift_task_enabled = false;
      }
    }
    if (climb_task_enabled){
      platform.set_value(false);
      on_rings = true;
      lift.set_current_limit_all(2500);
      lift.move_absolute(-5000, 75); //3200
      ladder_arm.move_relative(-500, 100);
      printf("Lift Pos 2: %f\n", lift.get_position());
      if (lift.get_position() < -3190){
        printf("Done with lifting 2\n");
        climb_task_enabled = false;
      }
    }

    if(deClimb_task_enabled){
      driveSafe_task_enabled = false;
      climb_task_enabled = false;

      //Ladder Arm In 
      ladder_arm.move_absolute(0, 100); //TODO: ADD TO DRIVER INITIALIZE

      //Unlock Gears
      lift_brake.set(true);
      lift.set_current_limit_all(2500);

      // lift.move_relative(-150, 50); TODO: TEST THIS CODE BY UNCOMMENTING. THIS IS TO UNBIND 

      //Bring Robot straight to the ground
      lift.move_absolute(-1700, 75); //TODO CHANGE TO ZERO AND TEST WITH GAMMA 15
      pros::delay(500);
      //Platform lowers so we can touch ground
      platform.set_value(true);
      on_rings = false;

      deClimb_task_enabled = false;
      
    }


    pros::delay(ez::util::DELAY_TIME);
  }
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
    //Auton("tuning",tuning),
    Auton("testTune",testTune),
     //Auton("purdue skills",purdueSkills),
    // Auton("Red AWP Test", red_AWP_match),
    //Auton("Blue AWP Test", blue_AWP_match),
      //Auton("Adriana's Monday Skills" ,fortySevenPointSkills),
      //Auton("Match", blue_match_auton),
      //Auton("Connor's Skills", skills),
      //Auton("Adriana'Skills ",twoMobilewithclimb),
      //Auton("Sixty-Five Skills", sixtyFiveSkills),
      //Auton("Sixty-Two Skills",sixtyTwoSkills),
      //Auton(" Fifty-Nine Skills", stateSkills),
      //Auton("Thirty Skills", thirtyPointSkills),
      //Auton("Match", match_auton),
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
  platform.set_value(true); // driver was true    auton was false
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

  
  //dock.set_value(true); //driver is false auton is true 
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

 void arm_slew_auto(int target){ // CAN BE DELETED

    lift_slew.constants_set(200, 10);
    lift_slew.initialize(true, 150, target, lift.get_position(2));
    int rpm;
    while(lift.get_position(2) > target){
      rpm = lift_slew.iterate(lift.get_position(2));
      lift.move_velocity(rpm);
      pros::delay(10);
    }
 }



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



int climb_status = 0;
bool isGearLocked = true;
void opcontrol() {
  // This is preference to what you like to drive on
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);
  bool mogo_toggle = false;
  bool mogo_pressed = false;
  //dock.set_value(true);
  //chassis.pid_drive_set(3_in, 70);
   //chassis.pid_wait();
   //dock.set_value(false);
  // bool dock_toggle = false;
  // bool dock_pressed = false;


  isGearLocked = false;

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

    // Both Layer Controls

    // MoGo Up (L1)
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
      mogo.set(true);
    }
    //MoGo Down (L2)
    else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
      mogo.set(false);
    }

    // Lift Down (R2)
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
      lift.set_current_limit_all(2500);
        if (!isGearLocked) {  // Only move down if gears are unlocked
            lift_task_enabled = false;
            // Set brake mode to COAST when moving down for smoother motion
            lift.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);

            // Stop arm at drive safe height on layer 1
            if (!master.get_digital(pros::E_CONTROLLER_DIGITAL_B) && lift.get_position() < -500){
              move_arm(50);
            }
            // Allow arm to go to floor on layer 2
            if(master.get_digital(pros::E_CONTROLLER_DIGITAL_B)){
              move_arm(50);
            }
              // Smooth movement down
        }
        else {  // Gears are locked, do not move the lift
            lift.move_velocity(0);  // Stop the lift
        }
    }

    // Lift up (R1)
    else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
      lift.set_current_limit_all(2500);
        if (!isGearLocked) {  // Only move up if gears are unlocked
            lift_task_enabled = false;
            // Set brake mode to COAST when moving up for smoother motion
            lift.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
            move_arm(-50);  // Smooth movement up
        }
        else {  // Gears are locked, do not move the lift
            lift.move_velocity(0);  // Stop the lift
        }
    }
    else {
        // When no buttons are pressed, hold the arm in place
        if (!lift_task_enabled && !climb_task_enabled) {
            lift.set_current_limit_all(-2500); //TODO: CHANGE TO POSITIVE?
            stop_arm();
            chassis.drive_current_limit_set(2500);
        }
        
        // Set brake mode to HOLD to keep the arm in place
        lift.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    }

    // First Layer Controls
    if (!master.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
    
      // Intake rings (A)
      if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) { 
        if (abs(intake.get_target_velocity()) < 1){
          intake.set_current_limit(2500);
          intake.move_velocity(-200);
        }
        else{
                    // Set brake mode to HOLD to keep the intake in position and prevent any free movement
        intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);  // Hold position
        intake.brake();
        pros::delay(10);
        // Set current limit to 0 to prevent overcurrent draw when stationary
        intake.set_current_limit(0);  // Disable current limit
        }
      }
      // Outtake rings (Left2)
      if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)){
          if (abs(intake.get_target_velocity()) < 1){
            intake.set_current_limit(2500);
            intake.move_velocity(200);
          }
          else{
                      // Set brake mode to HOLD to keep the intake in position and prevent any free movement
          intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);  // Hold position
          intake.brake();
          pros::delay(10);
          // Set current limit to 0 to prevent overcurrent draw when stationary
          intake.set_current_limit(0);  // Disable current limit
          }
            
      }


        // Ring platform down (RIGHT Arrow)
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)){
          platform.set_value(false);
          on_rings = false;
        }
        // Ring platform up (Y)
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_Y)){
          platform.set_value(true);
          on_rings = true;
        }

        // Auto Climb (DOWN Arrow)
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)){
          if (lift.get_position() >= -1900){
            printf("Running lift task\n");
            lift_task_enabled = true;
          }
          else if (lift.get_position() < -1900){
            printf("Running climb task\n");
            climb_task_enabled = true;
          }
        }

        // Ladder arm Retract (UP)
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)){
          ladder_arm.move_velocity(100);
        }
        else{
          if (!lift_task_enabled){
            ladder_arm.brake();
          }    
        }

        //DeCLimb (X)
      if(master.get_digital(pros::E_CONTROLLER_DIGITAL_X)){
        deClimb_task_enabled = true;
         isGearLocked = false;
        }


    }

    // Second Layer (when holding B)
    else {
      // Ladder arm extend (Y)
      if (master.get_digital(pros::E_CONTROLLER_DIGITAL_Y)){
        ladder_arm.move_velocity(-100);
      }
      else{
        if (!lift_task_enabled){
          ladder_arm.brake();
        }    
      }

      // Lock ratchet (Left)
      if (master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)){
        lift_brake.set(false);
        isGearLocked = true;
      }

      // Unlock gears (Up)
      if (master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)){
        lift_brake.set(true);
        unclimbing = true;
        lift.move_relative(-150, 50);
        isGearLocked = false;
      }
      else{
        unclimbing = false;
      }
      
      

      //Mobile goal grabber (Down ARROW)
        //mogo.button_toggle(master.get_digital(pros::E_CONTROLLER_DIGITAL_B)&&(master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)));

      // Drive safe (L1)
      if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
        lift.set_current_limit_all(2500);
        lift.move_absolute(-1000, 60); 
      }
      // ALL stakes height (L2)
      if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
          lift.set_current_limit_all(2500);
          lift.move_absolute(-2000, 75); 
      }
      else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)){
              dock.set_value(false);
      }
      else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)){
              dock.set_value(true);
      }
    }
  }
}


//Connors Controls - END
// void opcontrol() {
//   // This is preference to what you like to drive on
//   chassis.drive_brake_set(MOTOR_BRAKE_COAST);
//   bool mogo_toggle = false;
//   bool mogo_pressed = false;
  
//   pros::Controller master(pros::E_CONTROLLER_MASTER);
  
//   while (true) {
//     // Gives you some extras to make EZ-Template easier
//     //ez_template_etxras();

//     //chassis.opcontrol_tank();  // Tank control
//     chassis.opcontrol_arcade_standard(ez::SPLIT);   // Standard split arcade
//     // chassis.opcontrol_arcade_standard(ez::SINGLE);  // Standard single arcade
//     // chassis.opcontrol_arcade_flipped(ez::SPLIT);    // Flipped split arcade
//     // chassis.opcontrol_arcade_flipped(ez::SINGLE);   // Flipped single arcade
    
//     //printf("DT Motor Amps: %2.f", chassis.drive_mA_left());
//     // . . .
//     // Put more user control code here!
//     // . . .


// //CONNOR'S Controls

//     if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
//       lift.set_current_limit_all(2500);
//       lift_task_enabled = false;
//       move_arm(50);
//     }
//     else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
//       lift.set_current_limit_all(2500);
//       lift_task_enabled = false;
//       move_arm(-50);
//     }
//     else{
//       if (!lift_task_enabled && !climb_task_enabled){
        
//         lift.set_current_limit_all(2500);
//         stop_arm();
//         chassis.drive_current_limit_set(2500);
//       }    
//     }
//     // Macro button
//     //AutoClimb Button
//     if (master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)){
//       if (lift.get_position() >= -1900){
//         printf("Running lift task\n");
//         lift_task_enabled = true;
//       }
//       else if (lift.get_position() < -1900){
//         printf("Running climb task\n");
//         climb_task_enabled = true;
//       }
//     }

//     //Intakes stay on??? 
//     if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){ 
//       intake.set_current_limit(2500);
//       intake.move_velocity(200);
//     }
//     else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1 )){
//       intake.set_current_limit(2500);
//       intake.move_velocity(-200);
//     }
//     else{
//       intake.brake();
//       intake.set_current_limit(0);
//     }

//     //Unlocks gears - Working
//     if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A)){
//       lift_brake.set(true);
//       unclimbing = true;
//       lift.move_relative(-150, 50);
//     }
//     else{
//       unclimbing = false;
//     }
//     //X Locks Gears - Working
//     if (master.get_digital(pros::E_CONTROLLER_DIGITAL_X)){
//       lift_brake.set(false);
//     }
//     //Working for Up and Left
//     if (master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)){
//       platform.set_value(true);
//       on_rings = false;
//     }
//     else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)){
//       //dock.set_value(false);
//       platform.set_value(false);
//       on_rings = true;
//     }
//     // Working for Y and Right 
//     if (master.get_digital(pros::E_CONTROLLER_DIGITAL_Y)){
//       ladder_arm.move_velocity(100);
//     }
//     else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)){
//       ladder_arm.move_velocity(-100);
//     }
//     else{
//       if (!lift_task_enabled){
//         ladder_arm.brake();
//       }   
//     }


//     if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B) && master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)){
//       dock.set_value(false);
//     }
//     if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B) && master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)){
//       dock.set_value(true);
//     }
//     //Working 
//     mogo.button_toggle(master.get_digital(pros::E_CONTROLLER_DIGITAL_B));
  // // master.print(0, 0, "Val: %f", test_rev.get_value());
    
    //master.clear();
   // printf("Val: %f \n", left_tracker.get_raw());
   // printf("Pros val: %i\n", test_rev.get_value());

   
    //pros::delay(ez::util::DELAY_TIME);  // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
//  }
//}