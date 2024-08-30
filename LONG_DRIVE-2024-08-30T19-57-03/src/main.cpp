/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// Drivetrain           drivetrain    1, 2, 3, 4      
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;
motor LF = motor(PORT20, ratio18_1, true);
motor LM = motor(PORT19, ratio18_1, true);
motor LB = motor(PORT10, ratio18_1, true);
motor_group LeftDriveSmart = motor_group(LF, LM, LB);
motor RF = motor(PORT15, ratio18_1, false);
motor RM = motor(PORT1, ratio18_1, false);
motor RB = motor(PORT2, ratio18_1, false);
motor_group RightDriveSmart = motor_group(RF, RM, RB);
drivetrain Drivetrain = drivetrain(LeftDriveSmart, RightDriveSmart, 319.19, 295, 40, mm, 1);


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

int Error; //sensor value - desired value : position
int PrevError = 0; //position 20 miliseconds ago
int Derivative; //Error - PrevError : speed
int TotalError; //TotalError = TotalError + Error

int turnError;
int turnPrevError = 0;
int turnDerivative;
int turnTotalError;

bool ResetDriveSensors = false;

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
  vex::task::sleep(20);
}

return 1;
}
void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

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
  vex::task billWiTheScienceFi(PID_Drive);

  ResetDriveSensors = true;
  desiredValue = 300;
  desiredTurnValue = 600;

  vex::task::sleep(1000);

  ResetDriveSensors = true;
  desiredValue = 300;
  desiredTurnValue = 300;

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

    wait(125, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

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
