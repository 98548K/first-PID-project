#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);

// Drivetrain
/*motor LF = motor(PORT20, ratio18_1, true);
motor LM = motor(PORT19, ratio18_1, true);
motor LB = motor(PORT10, ratio18_1, true);
motor_group LeftDriveSmart = motor_group(LF, LM, LB);
motor RF = motor(PORT15, ratio18_1, false);
motor RM = motor(PORT1, ratio18_1, false);
motor RB = motor(PORT2, ratio18_1, false);
motor_group RightDriveSmart = motor_group(RF, RM, RB);*/

//rotation tracking1 = rotation(PORT5);
//rotation tracking2 = rotation(PORT7);

//drivetrain Drivetrain = drivetrain(LeftDriveSmart, RightDriveSmart, 319.19, 295, 40, mm, 1);
motor intake = motor(PORT9, ratio6_1, false);

// Pistons
digital_out Clamp = digital_out(Brain.ThreeWirePort.H);
digital_out PTO = digital_out(Brain.ThreeWirePort.B);


/*// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;
// define variables used for controlling motors based on controller inputs
bool DrivetrainLNeedsToBeStopped_Gay = true;
bool DrivetrainRNeedsToBeStopped_Gay = true;

// define a task that will handle monitoring inputs from Gay
int rc_auto_loop_function_Gay() {
  // process the controller input every 20 milliseconds
  // update the motors based on the input values
  while(true) {
    if(RemoteControlCodeEnabled) {
      // calculate the drivetrain motor velocities from the controller joystick axies
      // left = Axis3
      // right = Axis2
      int drivetrainLeftSideSpeed = Gay.Axis3.position();
      int drivetrainRightSideSpeed = Gay.Axis2.position();
      
      // check if the value is inside of the deadband range
      if (drivetrainLeftSideSpeed < 5 && drivetrainLeftSideSpeed > -5) {
        // check if the left motor has already been stopped
        if (DrivetrainLNeedsToBeStopped_Gay) {
          // stop the left drive motor
          LeftDriveSmart.stop();
          // tell the code that the left motor has been stopped
          DrivetrainLNeedsToBeStopped_Gay = false;
        }
      } else {
        // reset the toggle so that the deadband code knows to stop the left motor nexttime the input is in the deadband range
        DrivetrainLNeedsToBeStopped_Gay = true;
      }
      // check if the value is inside of the deadband range
      if (drivetrainRightSideSpeed < 5 && drivetrainRightSideSpeed > -5) {
        // check if the right motor has already been stopped
        if (DrivetrainRNeedsToBeStopped_Gay) {
          // stop the right drive motor
          RightDriveSmart.stop();
          // tell the code that the right motor has been stopped
          DrivetrainRNeedsToBeStopped_Gay = false;
        }
      } else {
        // reset the toggle so that the deadband code knows to stop the right motor next time the input is in the deadband range
        DrivetrainRNeedsToBeStopped_Gay = true;
      }
      
      // only tell the left drive motor to spin if the values are not in the deadband range
      if (DrivetrainLNeedsToBeStopped_Gay) {
        LeftDriveSmart.setVelocity(drivetrainLeftSideSpeed, percent);
        LeftDriveSmart.spin(forward);
      }
      // only tell the right drive motor to spin if the values are not in the deadband range
      if (DrivetrainRNeedsToBeStopped_Gay) {
        RightDriveSmart.setVelocity(drivetrainRightSideSpeed, percent);
        RightDriveSmart.spin(forward);
      }
    }
    // wait before repeating the process
    wait(20, msec);
  }
  return 0;
}*/

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
/*void vexcodeInit( void ) {
  task rc_auto_loop_task_Gay(rc_auto_loop_function_Gay);
}*/
