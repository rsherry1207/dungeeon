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
// InertialSensor       inertial      15              
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
  lift.setStopping(hold);
  rotation Rotation3 = rotation(PORT3,false);
  Rotation3.resetPosition();
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


int liftPloop(){
  double kp = .3;//0.03055555555
  double Fdeg = -34;
  double tolerance = 1; // Set a tolerance to avoid overshooting
  double runTimesec= .5;
  int exitTimer = 0;


  double Lerror;
  do{
    double currentDeg = Rotation3.position(deg);
    
    //Brain.Screen.print(currentDeg);
    printf("\n %lf ", currentDeg);
    Lerror = Fdeg - currentDeg;
    double Lvolts = kp * Lerror;
    lift.spin(forward,  Lvolts, voltageUnits::volt);
    wait(5, msec);
    exitTimer +=1;
  }while(fabs(Lerror) > tolerance && exitTimer < 200*runTimesec );
  //lift.stop(brake);
  
  /*double currentDeg = Rotation3.angle(degrees);
  Brain.Screen.print(currentDeg);*/

  return 0;

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


//positive side blue solo AWP no alliance stake
void Auton1()
{
  driveHold();
  setMoveConstants(2.4,0.4);
  //setMoveThreshold(0.2);
  move(-24);
  clamp.set(true);
  IntakeIn();
  wait(0.5, sec);
  IntakeStop();
  turn(105);
  IntakeIn();
  move(15);
  wait(1150, msec);
  IntakeStop();
  turn(-90);
  move(-10);
  wait(100, msec);
  clamp.set(false);
  wait(300, msec);
  turn(0);
  move(-22);
  clamp.set(true);
  turn(0);
  IntakeIn();
  wait(100, msec);
  move(40);
  turn(45);
  move(16);
  wait(800, msec);
  move(-24);
  turn(-115);
  move(30);
  
}

//blue negative side full goal touches bar
void Auton2()
{
  driveHold();
  setMoveConstants(2.4,0.4);
  move(-24);
  clamp.set(true);
  IntakeIn();
  move(-6);
  turn(-90);
  move(24);
  turn(-200);
  move(12);
  move(-6);
  turn(-170);
  move(6);
  wait(850, msec);
  move(-30);
  turn(-45);
  move(21);
  wait(500, msec);
  move(-24);
  turn(105);
  move(30);
}

void skillsAuton()
{
  
}



void ploopyActivate()
{
  task liftPloopyTask(liftPloop);
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

  Controller1.ButtonLeft.pressed(ploopyActivate);

  Competition.drivercontrol(Driver);
  Competition.autonomous(Auton1);
}