/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Mon Sep 11 2001                                           */
/*    Description:   attack plans                                             */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----   //
// Robot Configuration:                         //
// [Name]               [Type]        [Port(s)] //
// controller            controller             //
// Drivetrain           drivetrain    1, 2, 3, 4//    
// ---- END VEXCODE CONFIGURED DEVICES ----     //

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

//robot config.cpp stuff
motor LF = motor(PORT8, ratio18_1, true);
motor LM = motor(PORT10, ratio18_1, true);
motor LB = motor(PORT2, ratio18_1, true);
motor_group LeftDriveSmart = motor_group(LF, LM, LB);
motor RF = motor(PORT3, ratio18_1, false);
motor RM = motor(PORT15, ratio18_1, false);
motor RB = motor(PORT1, ratio18_1, false);
motor_group RightDriveSmart = motor_group(RF, RM, RB);
drivetrain Drivetrain = drivetrain(LeftDriveSmart, RightDriveSmart, 319.19, 295, 40, mm, 1);
inertial Inertial1 = inertial(PORT20);


void PID_turn(double LeftVelocity,double RightVelocity,double inches_traveled) {
  LeftDriveSmart.setVelocity(LeftVelocity, pct);
  RightDriveSmart.setVelocity(RightVelocity,pct);
  Drivetrain.driveFor(inches_traveled, inches,false);
  wait(1,sec);
  Drivetrain.stop();
}

void Reset_Both_Sides(double same_velocity) {
  LeftDriveSmart.setVelocity(same_velocity, pct);
  RightDriveSmart.setVelocity(same_velocity,pct);
}
// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;
// define variables used for controlling motors based on controller inputs
bool DrivetrainLNeedsToBeStopped_Controller1 = true;
bool DrivetrainRNeedsToBeStopped_Controller1 = true;

// define a task that will handle monitoring inputs from Controller1
int rc_auto_loop_function_Controller1() {
  // process the controller input every 20 milliseconds
  // update the motors based on the input values
  while(true) {
    if(RemoteControlCodeEnabled) {
      // calculate the drivetrain motor velocities from the controller joystick axies
      // left = Axis3
      // right = Axis2
      
      int drivetrainLeftSideSpeed = Controller1.Axis3.position();
      int drivetrainRightSideSpeed = Controller1.Axis2.position();

      // check if the value is inside of the deadband range
      if (drivetrainLeftSideSpeed < 5 && drivetrainLeftSideSpeed > -5) {
        // check if the left motor has already been stopped
        if (DrivetrainLNeedsToBeStopped_Controller1) {
          // stop the left drive motor
          LeftDriveSmart.stop();
          // tell the code that the left motor has been stopped
          DrivetrainLNeedsToBeStopped_Controller1 = false;
        }
      } else {
        // reset the toggle so that the deadband code knows to stop the left motor nexttime the input is in the deadband range
        DrivetrainLNeedsToBeStopped_Controller1 = true;
      }
      // check if the value is inside of the deadband range
      if (drivetrainRightSideSpeed < 5 && drivetrainRightSideSpeed > -5) {
        // check if the right motor has already been stopped
        if (DrivetrainRNeedsToBeStopped_Controller1) {
          // stop the right drive motor
          RightDriveSmart.stop();
          // tell the code that the right motor has been stopped
          DrivetrainRNeedsToBeStopped_Controller1 = false;
        }
      } else {
        // reset the toggle so that the deadband code knows to stop the right motor next time the input is in the deadband range
        DrivetrainRNeedsToBeStopped_Controller1 = true;
      }
      
      // only tell the left drive motor to spin if the values are not in the deadband range
      if (DrivetrainLNeedsToBeStopped_Controller1) {
        LeftDriveSmart.setVelocity(drivetrainLeftSideSpeed, percent);
        LeftDriveSmart.spin(forward);
      }
      // only tell the right drive motor to spin if the values are not in the deadband range
      if (DrivetrainRNeedsToBeStopped_Controller1) {
        RightDriveSmart.setVelocity(drivetrainRightSideSpeed, percent);
        RightDriveSmart.spin(forward);
      }
    }
    // wait before repeating the process
    wait(20, msec);
  }
  return 0;
}

void vexcodeInit( void ) {
  task rc_auto_loop_task_Controller1(rc_auto_loop_function_Controller1);
}


//robot config.cpp stuff



bool Clamping = false;

// define your global instances of motors and other devices here

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/
//settings:
double kp = 0.0;
double ki = 0.0;
double kd = 0.0;
double turnkp = 0.0;
double turnki = 0.0;
double turnkd = 0.0;

//autonomous settings
int desiredValue = 200;
int desiredTurnValue = 0;

int Error; 
//sensor value - desired value : position
int PrevError = 0; 
//position 20 miliseconds ago
int Derivative; 
//Error - PrevError : speed
int TotalError; 
//TotalError = TotalError + Error

int turnError;
int turnPrevError = 0;
int turnDerivative;
int turnTotalError;

bool ResetDriveSensors = true;

//varibles modified for use:
bool enablePID_Drive = true;
int PID_Drive (){

while (enablePID_Drive) {
  if (ResetDriveSensors) {
    ResetDriveSensors = false;
    LeftDriveSmart.setPosition(0,degrees);
    RightDriveSmart.setPosition(0,degrees);
  }

  //get the position of both motors
  int leftMotorPosition = LeftDriveSmart.position(degrees);
  int rightMotorPosition = RightDriveSmart.position(degrees);
  

  ////////////////////////////////////////////////////////////////////////////////
  //lateral movement PID
  ///////////////////////////////////////////////////////////////////////////////

  //get average of the two motors
  int averagePosition = (leftMotorPosition + rightMotorPosition)/2;

    //potential
    Error = averagePosition - desiredValue;

    //derivative
    Error = Error - PrevError;

    //velocity -> position -> absement
    TotalError += Error;

    //integral
    TotalError += Error;

    double lateralMotorPower = Error * kp + Derivative * kd + TotalError * ki;

   ////////////////////////////////////////////////////////////////////////////////



  ////////////////////////////////////////////////////////////////////////////////
  //turning movement PID
  ///////////////////////////////////////////////////////////////////////////////
  //get average of the two motors
  int turnDifference = (leftMotorPosition + rightMotorPosition)/2;

    //potential
    turnError = turnDifference - desiredTurnValue;

    //derivative
    turnDerivative = turnError - turnPrevError;

    //velocity -> position -> absement
    TotalError += Error;

    //integral
    //turnTotalError += turnError

    double turnMotorPower = turnError * turnkp + turnDerivative * turnkd;


  ////////////////////////////////////////////////////////////////////////////////

    LeftDriveSmart.spin(forward, lateralMotorPower + turnMotorPower, velocityUnits::pct);
    RightDriveSmart.spin(forward, lateralMotorPower + turnMotorPower, velocityUnits::pct);





  PrevError = Error;
  turnPrevError = Error;
}

return 1;
}
void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  Drivetrain.setDriveVelocity(100,percent);
  intake.setVelocity(100, percent);
  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  /*vex::task DrivetrainPID(PID_Drive);

  desiredValue = 300;
  desiredTurnValue = 600;

  vex::task::sleep(1000);

  ResetDriveSensors = true;
  desiredValue = 300;
  desiredTurnValue = 300;*/

  //chris this ones for you: first value is left side velocity, second value is right side velocity, third value is inches traveled.
  /*PID_turn(0, 100, 12);
  intake.spin(forward);
  intake.stop();
  PID_turn(100, 0, 10);
  Reset_Both_Sides(100);
  Drivetrain.driveFor(reverse,20,inches);*/
  Drivetrain.setDriveVelocity(70, percent);
  Inertial1.calibrate();
  intake.spinFor(forward,1000,degrees,false);
  wait(1.5,sec);
  Drivetrain.driveFor(forward,6.9,inches);
  Drivetrain.turnFor(left,54.5,degrees);
  Clamp.set(true);
  Drivetrain.setDriveVelocity(50, percent);
  Drivetrain.driveFor(reverse,10,inches);
  Clamp.set(false);
} 

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  //enablePID_Drive = false;
  // User control code here, inside the loop
  while (1) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................



    // Clamp toggle
    if (Controller1.ButtonY.pressing()) {
      if (Clamping == false){
        Clamping = true;
      }
       else if (Clamping == true){
        Clamping = false;
      }
    }
    Clamp.set(Clamping);
    
    if (Controller1.ButtonL1.pressing()) {
      intake.spin(forward);
    }else if (Controller1.ButtonL2.pressing()) {
      intake.spin(reverse);
    }else if (true){
      intake.stop();
    }
  //LeftDriveSmart.setVelocity(100, pct);
  //RightDriveSmart.setVelocity(100,pct);
  


    wait(.2, sec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.

  }
}



//kasen did not miss a semicolon - mckay

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
