// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Drive1               motor         20              
// Drive2               motor         19              
// Drive3               motor         17              
// Drive4               motor         12              
// Drive5               motor         13              
// Drive6               motor         14              
// Controller1          controller                    
// InertialSensor       inertial      16              
// clamp                digital_out   A               
// Intake               motor         10              
// doink                digital_out   B               
// Rotation3            rotation      3               
// lift                 motor_group   2, 4            
// ---- END VEXCODE CONFIGURED DEVICES ----
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Drive1               motor         20              
// Drive2               motor         19              
// Drive3               motor         17              
// Drive4               motor         12              
// Drive5               motor         13              
// Drive6               motor         14              
// Controller1          controller                    
// InertialSensor       inertial      16              
// clamp                digital_out   A               
// Intake               motor         10              
// doink                digital_out   B               
// Rotation             rotation      3               
// lift                 motor_group   2, 4            
// ---- END VEXCODE CONFIGURED DEVICES ----
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Drive1               motor         11              
// Drive2               motor         12              
// Drive3               motor         14              
// Drive4               motor         18              
// Drive5               motor         19              
// Drive6               motor         20              
// Controller1          controller                    
// InertialSensor       inertial      13              
// Linkage              digital_out   E               
// Tilt                 digital_out   C               
// Claw                 digital_out   D               
// Puncher              motor_group   16, 15          
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
using namespace vex;
competition Competition;
bool ClampState = false;


void Driver()
{
  startDriver();
  driveCoast();

}


void whenStarted()
{
  InertialSensor.startCalibration();
  waitUntil(!InertialSensor.isCalibrating());
  lift.setStopping(brake);
  rotation Rotation3 = rotation(PORT3,true);
  leftDriveMotors(Drive3, Drive2, Drive1);
  rightDriveMotors(Drive6, Drive5, Drive4);
  enableDrivetrain();
}


void IntakeIn(){Intake.spin(forward, 11 , voltageUnits::volt);}
void IntakeOut(){Intake.spin(forward, -11 , voltageUnits::volt);}
void IntakeStop(){Intake.spin(forward, 0 , voltageUnits::volt);}

void lifting(){
  lift.spin(forward, -11 , voltageUnits::volt);
}
void liftdescend(){
  lift.spin(forward, 11 , voltageUnits::volt);
}
void liftingstop(){
  lift.stop(brake);
}

void liftPloop(){
  double kp = 0.03055555555;
  double Fdeg = 90;
  double tolerance = 0.1; // Set a tolerance to avoid overshooting

  double Lerror;
  do{
    double currentDeg = Rotation3.angle(degrees);
    Brain.Screen.print(currentDeg);
    Lerror = Fdeg - currentDeg;
    double Lvolts = kp * Lerror;
    lift.spin(forward,  Lvolts, voltageUnits::volt);
  }while(abs(Lerror) > tolerance);
  lift.stop(brake);
  
  /*double currentDeg = Rotation3.angle(degrees);
  Brain.Screen.print(currentDeg);*/

}

void extendDoink(){
  doink.set(true);
}
void retractDoink(){
  doink.set(false);
}

void Clamping(){
  if(ClampState == false){
    clamp.set(true);
    ClampState = true;
  }
  else if(ClampState == true){
    clamp.set(false);
    ClampState = false;
  }
};



void Auton()
{

}

void skillsAuton()
{
  
}


int main() 
{
  vexcodeInit();
  whenStarted();
  
  Controller1.ButtonR1.pressed(IntakeIn);
  Controller1.ButtonL1.pressed(IntakeOut);
  Controller1.ButtonL1.released(IntakeStop);
  Controller1.ButtonR1.released(IntakeStop);

  Controller1.ButtonRight.pressed(extendDoink);
  Controller1.ButtonRight.released(retractDoink);
  Controller1.ButtonY.pressed(Clamping);

  Controller1.ButtonR2.pressed(lifting);
  Controller1.ButtonL2.pressed(liftdescend);
  Controller1.ButtonR2.released(liftingstop);
  Controller1.ButtonL2.released(liftingstop);

  Controller1.ButtonLeft.pressed(liftPloop);

  Competition.drivercontrol(Driver);
  Competition.autonomous(Auton);
}