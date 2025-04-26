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

