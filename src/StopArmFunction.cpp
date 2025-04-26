// //Adriana's Version
// void stop_arm() {
//   // Set the current limit to prevent excessive current draw when stopping.
//   lift.set_current_limit(200);

//   // Set brake mode to HOLD to keep the arm in place only if needed.
//   lift.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD); // Hold the arm in position

//   // Prevents arm from moving down when below the lower threshold
//   if (lift.get_position(2) > -30 && arm_dir == 1) {
//     // Arm is near the bottom and trying to move down, stop movement and don't brake
//     return;
//   }

//   // Prevents arm from moving up when above the upper threshold
//   if (lift.get_position(2) < -3400 && arm_dir == -1) {
//     // Arm is near the upper limit and trying to move up, stop movement
//     return;
//   }

//   // If the arm is moving and slew is enabled, handle the deceleration
//   if (arm_moving && arm_slew_active) {
//     // Configure variables for deceleration
//     if (!arm_decelerating) {
//       arm_decelerating = true;
//       arm_decel_velocity = lift.get_actual_velocity(2);  // Get current velocity
//       start_time = end_time - 1;  // Small start time offset to avoid high initial acceleration
//     }

//     // Record time and calculate duration since last update
//     end_time = pros::millis();
//     duration = end_time - start_time;

//     // If velocity is under 30, stop decelerating and stop the arm
//     if (abs(arm_decel_velocity) < 30) {
//       arm_decel_velocity = 0;
//       arm_decelerating = false;
//       arm_moving = false;
//       lift.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);  // Hold position when done moving
//     }

//     // Limit velocity to a maximum value if it exceeds max RPM
//     if (abs(arm_decel_velocity) > 200) {
//       arm_decel_velocity = 200;
//     }

//     // Calculate direction and decrease velocity for smooth deceleration
//     arm_dir = arm_decel_velocity / abs(arm_decel_velocity);
//     arm_decel_velocity -= 20.0 * arm_dir * (duration / 30.0);  // Apply deceleration

//     // Move the arm at the calculated deceleration velocity
//     lift.move_velocity(arm_decel_velocity);

//     // Update start time for the next loop
//     start_time = pros::millis();
//   }
//   // If slew is not active, simply stop the arm
//   // else {
//   //   if (unclimbing) {
//   //     return;  // Do nothing if unclimbing flag is set
//   //   }
//   //   // Keep brake mode as HOLD and stop the arm if no movement is happening
//   //   lift.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);  // Make sure the arm doesn't move if no button pressed
//   //   lift.move_velocity(0); // Actually stop the arm movement
//   // }
// }


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