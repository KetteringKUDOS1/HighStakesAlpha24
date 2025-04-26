
      // Purdue Controls
      // Both Layer Controls

    // // MoGo Up (L1)
    // if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
    //   mogo.set(true);
    // }
    // //MoGo Down (L2)
    // else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
    //   mogo.set(false);
    // }

    // // Lift Down (R2)
    // if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
    //   lift.set_current_limit_all(2500);
    //     if (!isGearLocked) {  // Only move down if gears are unlocked
    //         lift_task_enabled = false;
    //         // Set brake mode to COAST when moving down for smoother motion
    //         lift.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);

    //         // Stop arm at drive safe height on layer 1
    //         if (!master.get_digital(pros::E_CONTROLLER_DIGITAL_B) && lift.get_position() < -500){
    //           move_arm(50);
    //         }
    //         // Allow arm to go to floor on layer 2
    //         if(master.get_digital(pros::E_CONTROLLER_DIGITAL_B)){
    //           move_arm(50);
    //         }
    //           // Smooth movement down
    //     }
    //     else {  // Gears are locked, do not move the lift
    //         lift.move_velocity(0);  // Stop the lift
    //     }
    // }

    // // Lift up (R1)
    // else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
    //   lift.set_current_limit_all(2500);
    //     if (!isGearLocked) {  // Only move up if gears are unlocked
    //         lift_task_enabled = false;
    //         // Set brake mode to COAST when moving up for smoother motion
    //         lift.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
    //         move_arm(-50);  // Smooth movement up
    //     }
    //     else {  // Gears are locked, do not move the lift
    //         lift.move_velocity(0);  // Stop the lift
    //     }
    // }
    // else {
    //     // When no buttons are pressed, hold the arm in place
    //     if (!lift_task_enabled && !climb_task_enabled) {
    //         lift.set_current_limit_all(-2500); //TODO: CHANGE TO POSITIVE?
    //         stop_arm();
    //         chassis.drive_current_limit_set(2500);
    //     }
        
    //     // Set brake mode to HOLD to keep the arm in place
    //     lift.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    // }

//  First Layer
    // if (!master.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
    
    //   // Intake rings (A)
    //   if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) { 
    //     if (abs(intake.get_target_velocity()) < 1){
    //       intake.set_current_limit(2500);
    //       intake.move_velocity(-200);
    //     }
    //     else{
    //                 // Set brake mode to HOLD to keep the intake in position and prevent any free movement
    //     intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);  // Hold position
    //     intake.brake();
    //     pros::delay(10);
    //     // Set current limit to 0 to prevent overcurrent draw when stationary
    //     intake.set_current_limit(0);  // Disable current limit
    //     }
    //   }
    //   // Outtake rings (Left2)
    //   if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)){
    //       if (abs(intake.get_target_velocity()) < 1){
    //         intake.set_current_limit(2500);
    //         intake.move_velocity(200);
    //       }
    //       else{
    //                   // Set brake mode to HOLD to keep the intake in position and prevent any free movement
    //       intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);  // Hold position
    //       intake.brake();
    //       pros::delay(10);
    //       // Set current limit to 0 to prevent overcurrent draw when stationary
    //       intake.set_current_limit(0);  // Disable current limit
    //       }
            
    //   }


    //     // Ring platform down (RIGHT Arrow)
    //     if (master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)){
    //       platform.set_value(false);
    //       on_rings = false;
    //     }
    //     // Ring platform up (Y)
    //     else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_Y)){
    //       platform.set_value(true);
    //       on_rings = true;
    //     }

    //     // Auto Climb (DOWN Arrow)
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

    //     // Ladder arm Retract (UP)
    //     if (master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)){
    //       ladder_arm.move_velocity(100);
    //     }
    //     else{
    //       if (!lift_task_enabled){
    //         ladder_arm.brake();
    //       }    
    //     }

    //     //DeCLimb (X)
    //   if(master.get_digital(pros::E_CONTROLLER_DIGITAL_X)){
    //     deClimb_task_enabled = true;
    //      isGearLocked = false;
    //     }
// Second Layer
      // // Ladder arm extend (Y)
      // if (master.get_digital(pros::E_CONTROLLER_DIGITAL_Y)){
      //   ladder_arm.move_velocity(-100);
      // }
      // else{
      //   if (!lift_task_enabled){
      //     ladder_arm.brake();
      //   }    
      // }

      // // Lock ratchet (Left)
      // if (master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)){
      //   lift_brake.set(false);
      //   isGearLocked = true;
      // }

      // // Unlock gears (Up)
      // if (master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)){
      //   lift_brake.set(true);
      //   unclimbing = true;
      //   lift.move_relative(-150, 50);
      //   isGearLocked = false;
      // }
      // else{
      //   unclimbing = false;
      // }
      
      

      // //Mobile goal grabber (Down ARROW)
      //   //mogo.button_toggle(master.get_digital(pros::E_CONTROLLER_DIGITAL_B)&&(master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)));

      // // Drive safe (L1)
      // if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
      //   lift.set_current_limit_all(2500);
      //   lift.move_absolute(-1000, 60); 
      // }
      // // ALL stakes height (L2)
      // if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
      //     lift.set_current_limit_all(2500);
      //     lift.move_absolute(-2000, 75); 
      // }
      // else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)){
      //         dock.set_value(false);
      // }
      // else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)){
      //         dock.set_value(true);
      // }