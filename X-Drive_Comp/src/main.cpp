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
// FL_Drive             motor         6               
// BL_Drive             motor         2               
// FR_Drive             motor         3               
// BR_Drive             motor         4               
// LeftEncoder          encoder       A, B            
// RightEncoder         encoder       C, D            
// HorizEncoder         encoder       E, F            
// Inertial1            inertial      5               
// Intake               motor         8               
// Flywheel             motor         9               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

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

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}
/////////////////////////////////////////////////////////////////////////////////////
//      ////////////////////      /////////////////     //////////
//      //               ///             ///            //        //
//      //               ///             ///            //          //
//      //               ///            ///             //            //
//      //               ///            ///             //              //
//      ////////////////////            ///             //              //
//      //                              ///             //              //
//      //                              ///             //              // 
//      //                              ///             //           //
//      //                              ///             //         //
//      //                         ////////////////     //////////

////////////////////////////////////////////////////////////////////////////////////
//variables
    double FrontLeftSpeed;
    double BackLeftSpeed;
    double FrontRightSpeed;
    double BackRightSpeed;
    double Clock;
    double Gyro1;
    //bool ResetTimer = false;
    bool mogoIn = false;
    bool DrivetrainHold = true;
    bool enableCameraPID = true;
    bool resetGyroHeading = false;

  //setting
  // these are the constants
    double kP = 0.00065;
    double kI = 0;
    double kD = 0.0006;

    //reverse
    double rkP = 0.000651;
    double rkI = 0;
    double rkD = 0.0006;

    //turn right
    double turnkP = 0.08;
    double turnkI = 0;
    double turnkD = 0.025;

  //Value for encoders
    double desiredForwardValue;
    double desiredReverseValue;
    double desiredLateralValue;
    double desiredTurnValue;
    double desiredRightValue;
    double desiredLeftValue;
    double desiredVelocity; 

  //autonomous setting
  double targetDist;
  double targetLateralDist;
  double targetTurnDist;

  //forward variables
    int error; // sensor value - desired value ; positional value
    int prevError = 0; // position 20 miliseconds ago
    int derivative; // error - previous error : calculates speed
    int totalError = 0; // totalError = totalError + error

  //Lateral variables
    double latError; // sensor value - desired value ; positional value
    double latPrevError = 0; // position 20 miliseconds ago
    double latDerivative; // error - previous error : calculates speed
    double latTotalError = 0; // totalError = totalError + error
  //reverse
    double revError; // sensor value - desired value ; positional value
    double revPrevError = 0; // position 20 miliseconds ago
    double revDerivative; // error - previous error : calculates speed
    double revTotalError = 0; // totalError = totalError + error



  //right
    double turnError; // sensor value - desired value ; positional value
    double turnPrevError = 0; // position 20 miliseconds ago
    double turnDerivative; // error - previous error : calculates speed
    double turnTotalError = 0; // totalError = totalError + error




  //reset encoders to 0
  bool resetDriveSensors = false;

  // variables modified for use
  bool enableDrivePID = true;

// this will constanly run even during driver and auton
// so you will need to tell it to stop
// while with regular the program will stop it on its own
int drivePID(){

  while(enableDrivePID){

    ///////////////////////////////////////////////
    // Lateral (forward) movement PID
    /////////////////////////////////////////////////////////////////////////////////////

    // 360 tics(degrees)  = 2.75 inches
    //get the average of the motors
    
    desiredForwardValue = targetDist * (360/2.75);
    double averagePosition = (LeftEncoder.position(degrees) + RightEncoder.position(degrees)) / 2;

    

    // Potential
    // calculates how far off of the target robot is
    if(averagePosition < desiredForwardValue){
      error = desiredForwardValue - averagePosition;      
    }
    else{
      error = 0;
    }

    // Derivative
    // calculates how fast the robot needs to go
    // if it is going too fast it will slow it down
    // if it is going too slow it will speed it up
    if(error > prevError){
      derivative = error - prevError;
    }
    else{
      derivative = 0;
    }

    // Integral
    // Velocity -> Position -> Absence(position X time)
    // if the robot does not make it to target, it will add
    // degrees so that it does reach target
    if(error > totalError){
      totalError += error;
    }
    else{
      totalError = 0;
    }
    

    // add all of it up for the motor power output
    double lateralMotorPower = error * kP + derivative * kD + totalError * kI;
    /////////////////////////////////////////////////////////////////////////////////////

    ///////////////////////////////////////////////
    // Lateral (reverse) movement PID
    /////////////////////////////////////////////////////////////////////////////////////

    //get the average of the motors
    int averageRPosition = (LeftEncoder.position(degrees) + RightEncoder.position(degrees)) / 2;

    // Potential
    // calculates how far off of the target robot is
    if(averageRPosition > desiredReverseValue){
      revError = averageRPosition - desiredReverseValue;      
    }
    else{
      revError = 0;
    }

    // Derivative
    // calculates how fast the robot needs to go
    // if it is going too fast it will slow it down
    // if it is going too slow it will speed it up
    if(revError > revPrevError){
      revDerivative = revError - revPrevError;
    }
    else{
      revDerivative = 0;
    }

    // Integral
    // Velocity -> Position -> Absence(position X time)
    // if the robot does not make it to target, it will add
    // degrees so that it does reach target
    if(revError > revTotalError){
      revTotalError += revError;
    }
    else{
      revTotalError = 0;
    }

    // add all of it up for the motor power output
    //double reverseMotorPower; //= revError * rkP + revDerivative * rkD + revTotalError * rkI;;
    ///////////////////////////////////////////
    
    ///////////////////////////////////////////////
    // Lateral (horiz) movement PID
    /////////////////////////////////////////////////////////////////////////////////////

    //get the average of the motors
    int averagehorizPosition = HorizEncoder.position(degrees);

    

    // Potential
    // calculates how far off of the target robot is
    if(averagehorizPosition < desiredLateralValue){
      latError = desiredLateralValue - averagehorizPosition;      
    }
    else{
      latError = 0;
    }

    // Derivative
    // calculates how fast the robot needs to go
    // if it is going too fast it will slow it down
    // if it is going too slow it will speed it up
    if(latError > latPrevError){
      latDerivative = latError - latPrevError;
    }
    else{
      latDerivative = 0;
    }

    // Integral
    // Velocity -> Position -> Absence(position X time)
    // if the robot does not make it to target, it will add
    // degrees so that it does reach target
    if(latError > latTotalError){
      latTotalError += latError;
    }
    else{
      latTotalError = 0;
    }
    

    // add all of it up for the motor power output
    double horizMotorPower = error * kP + derivative * kD + totalError * kI;
    /////////////////////////////////////////////////////////////////////////////////////

    /////////////////////////////
    //Turning Right movement PID
    /////////////////////////////////////////////////////////////////////////////////////
    //get the average of the motors
    int turnDifference = Inertial1.heading(degrees);

    // Potential
    // calculates how far off of the target robot is
    if(turnDifference < desiredTurnValue){
      turnError = desiredTurnValue - turnDifference;      
    }
    else if(turnDifference > desiredTurnValue){
      turnError = desiredTurnValue - turnDifference;
    }
    else{
      turnError = 0;
    }

    // Derivative
    // calculates how fast the robot needs to go
    // if it is going too fast it will slow it down
    // if it is going too slow it will speed it up
    if(turnError > turnPrevError){
    turnDerivative = turnError - turnPrevError;
    }
    else if(turnError > turnPrevError){
    turnDerivative = turnError - turnPrevError;
    }
    else{
      turnDerivative = 0;
    }


    // Integral
    // Velocity -> Position -> Absence(position X time)
    // if the robot does not make it to target, it will add
    // degrees so that it does reach target
    if(turnTotalError > turnError){
    turnTotalError += turnError;
    }
    else{
      turnTotalError = 0;
    }
    turnTotalError += turnError;

    // add all of it up for the motor power output
    double turnMotorPower = turnError * turnkP + turnDerivative * turnkD + turnTotalError * turnkI;
    /////////////////////////////////////////////////////////////////////////////////////


     



    //LeftMotors.spin(forward, lateralMotorPower + turnMotorPower, voltageUnits::volt);
    //RightMotors.spin(forward, lateralMotorPower - turnMotorPower, voltageUnits::volt);
    FL_Drive.spin(forward, lateralMotorPower  + horizMotorPower + turnMotorPower, voltageUnits::volt);
    BL_Drive.spin(forward, lateralMotorPower - horizMotorPower + turnMotorPower, voltageUnits::volt);
    FR_Drive.spin(forward, lateralMotorPower - horizMotorPower - turnMotorPower, voltageUnits::volt);
    BR_Drive.spin(forward, lateralMotorPower + horizMotorPower - turnMotorPower, voltageUnits::volt);
    
 
  Gyro1 = (Inertial1.heading(degrees));
 Clock = Brain.Timer.value();
 
 Controller1.Screen.setCursor(1,1);
 Controller1.Screen.clearLine();
 Controller1.Screen.print("RightEncoder is  %f", RightEncoder.position(degrees), " degrees.");
 Controller1.Screen.newLine();
 Controller1.Screen.print("LeftEncoder is %f", LeftEncoder.position(degrees), " degrees.");
 Controller1.Screen.newLine();
 Controller1.Screen.print("Gyro1 at is %f", Gyro1, " degrees.");
  
 

 vex::task::sleep(20);
 }
return 1;
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

void Drive(int dist, int Heading, int lateral){
  targetDist = dist;
  desiredTurnValue = Heading;
  desiredLateralValue = lateral;
}

void autonomous(void) {
  vex::task MaxScore(drivePID);
  //forward 10 inches
  Drive(100, 0, 0);
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
  enableDrivePID = false;
  // User control code here, inside the loop
  while (1) {
    FR_Drive.spin(forward, Controller1.Axis3.value() - Controller1.Axis1.value() - Controller1.Axis4.value(), percent);
    BR_Drive.spin(forward, Controller1.Axis3.value() - Controller1.Axis1.value() + Controller1.Axis4.value(), percent);
    FL_Drive.spin(forward, Controller1.Axis3.value() + Controller1.Axis1.value() + Controller1.Axis4.value(), percent);
    BL_Drive.spin(forward, Controller1.Axis3.value() + Controller1.Axis1.value() - Controller1.Axis4.value(), percent);

    
     //code for intake 
  bool intake;
  bool revIntake;
  //bool intakeRev;
  // intake in
if(Controller1.ButtonX.pressing()){
    intake = false;
    revIntake = true;
    }
    //intake out
    if(Controller1.ButtonY.pressing()){
      revIntake = false;
      intake = true;
    }
    // stop intake 
     if(Controller1.ButtonB.pressing()){
    intake = true;
    revIntake = true;
    }
    
//move intake
if(intake == false && revIntake == true){ 
 Intake.setVelocity(90, percent);
 Intake.spin(forward);
  }
  if(revIntake == false && intake == true){ 
 Intake.setVelocity(90, percent);
 Intake.spin(reverse);
  }
  //coast
  if(intake == true && revIntake == true){
    Intake.stop(coast);
  }

        //code for flywheel 
  bool flywheel;
  // flywheel spin
if(Controller1.ButtonR1.pressing()){
  flywheel = true;
    }
    //flywheel stop
    if(Controller1.ButtonR2.pressing()){
      flywheel = false;
    }
  
    
//start flywheel
if(flywheel == true){ 
 Flywheel.setVelocity(60, percent);
 Flywheel.spin(forward);
  }
  //coast
  if(flywheel == false){
    Flywheel.stop(coast);
  }
    
    
    wait(20, msec); // Sleep the task for a short amount of time to
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
