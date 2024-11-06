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
// Optical              optical       9               
// Potentiometer        potV2         H               
// hang                 digital_out   G               
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
// Optical              optical       9               
// Potentiometer        potV2         H               
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
bool HangState = false;
bool autonRingdetector =true;
bool colorStop = false;

color detectColor = Optical.color();

void Driver()
{
  startDriver();
  driveCoast();

}


void whenStarted()
{
  InertialSensor.startCalibration();
  waitUntil(!InertialSensor.isCalibrating());
  leftDriveMotors(Drive3, Drive2, Drive1);
  rightDriveMotors(Drive6, Drive5, Drive4);
  lift.setStopping(hold);
  rotation Rotation3 = rotation(PORT3,false);
  Rotation3.resetPosition();
  enableDrivetrain();
}

int colorstop()
{
  while(true){
  if(Optical.hue() >= 0 && Optical.hue() <= 45 && colorStop == true){
    Intake.stop(brake);
    printf("WORK");
  }
  else
  {
    return 0;
  }
}
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
  double Fdeg = -25;//-24//-21//
  double tolerance = 1; // Set a tolerance to avoid overshooting
  double runTimesec= .5;
  int exitTimer = 0;


  double Lerror;
  do{
    double currentDeg = Rotation3.position(deg);
    
    //Brain.Screen.print(currentDeg);
    //printf("\n %lf ", currentDeg);
    Lerror = Fdeg - currentDeg;
    double Lvolts = kp * Lerror;
    lift.spin(forward,  Lvolts, voltageUnits::volt);
    wait(5, msec);
    exitTimer +=1;
  }while(fabs(Lerror) > tolerance && exitTimer < 200*runTimesec );
  //lift.stop(brake);
  
  //double currentDeg = Rotation3.angle(degrees);
  //Brain.Screen.print(currentDeg);

  return 0;

}

int resetLift(){
  double kp = .3;//0.03055555555
  double Fdeg = 0;
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
  
  //double currentDeg = Rotation3.angle(degrees);
  //Brain.Screen.print(currentDeg);

  return 0;

}

int scoreLift(){
  double kp = .3;//0.03055555555
  double Fdeg = -144;
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
  
  //double currentDeg = Rotation3.angle(degrees);
  //Brain.Screen.print(currentDeg);

  return 0;

}

void ploopyActivate()
{
  task liftPloopyTask(liftPloop);
}

void ploopyReset()
{
  task setLift(0);
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
}

void Hanging(){
  if(HangState == false){
    hang.set(true);
    HangState = true;
  }
  else if(HangState == true){
    hang.set(false);
    HangState = false;
  }
}

int segregateRed()
{
  //blue ring is 200-210 hyue
  //red  is 5-20 ue
  if(detectColor >= 1 ||  detectColor <=30){
   wait( 880, msec);
  IntakeOut();
  wait( 200, msec);
  IntakeStop();
  }

  return 0;
}

int senseRing(){
  int curRing = detectColor;
  printf("\n %d ", curRing);
  if(autonRingdetector == true){
    if(curRing >= 1 &&  curRing <=22)
    IntakeStop();
  }
  else{
    return 0;
  }

  return 0;
}


void segregateBlue(){


  wait( 0.5, sec);
  IntakeStop();
}


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
  turn(105);
  IntakeIn();
  move(15);
  wait(1150, msec);
  turn(-90);
  IntakeStop();
  move(-12);
  wait(100, msec);
  clamp.set(false);
  wait(300, msec);
  move(2);
  turn(0);
  move(-22);
  clamp.set(true);
  IntakeIn();
  turn(0);
  wait(100, msec);
  IntakeOut();
  move(42);
  move(-2);
  turn(45);
  setTimeoutTime(1500);
  IntakeIn();
  move(20.5);
  //setMoveThreshold(0.2);
  wait(1600, msec);
  IntakeStop();
  move(-24);
  turn(-115);
  move(30);
}
//positive side blue solo AWP no alliance stake string doink
void Auton12()
{
  driveHold();
  setMoveConstants(2.4,0.4);
  //setMoveThreshold(0.2);
  move(-24);
  clamp.set(true);
  IntakeIn();
  wait(0.7, sec);
  turn(105);
  IntakeIn();
  move(15);
  wait(1150, msec);
  turn(-90);//
  IntakeStop();
  move(-12);
  //wait(100, msec);
  clamp.set(false);
  wait(300, msec);
  move(2);
  turn(-2);//
  move(-22);
  clamp.set(true);
  IntakeIn();
  turn(-47);
  extendDoink();
  move(60);
  wait(100, msec);
  retractDoink();
  wait(300, msec);
  setMovePowerLimit(40);
  move(-10);
  turn(-137);
  //move(10);
  // wait(100, msec);
  // IntakeOut();
  // move(42);
  // move(-2);
  // turn(45);
  // setTimeoutTime(1500);
  // IntakeIn();
  // move(20.5);
  // //setMoveThreshold(0.2);
  // wait(1600, msec);
  // IntakeStop();
  // move(-24);
  // turn(-115);
  // move(30);
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
  move(13);
  move(-7);
  turn(-170);
  move(8);
  wait(850, msec);
  move(-32);
  turn(-45);
  move(21);
  wait(500, msec);
  move(-24);
  turn(105);
  //move(33);
}

//positive side red solo AWP no alliance stake
void Auton3()
{
  setMovePowerLimit(100);
  driveHold();
  setMoveConstants(2.4,0.4);
  //setMoveThreshold(0.2);
  move(-24);
  clamp.set(true);
  IntakeIn();
  wait(0.5, sec);
  turn(-105); ///
  IntakeIn();
  move(15);
  wait(1150, msec);
  turn(90); ///
  IntakeStop();
  move(-12);
  wait(100, msec);
  clamp.set(false);
  wait(300, msec);
  move(2);
  turn(0);
  move(-22);
  clamp.set(true);
  IntakeIn();
  turn(0);
  wait(100, msec);
  IntakeOut();
  move(42);
  move(-2);
  turn(-45);///
  setTimeoutTime(1500);
  IntakeIn();
  move(20.5);
  //setMoveThreshold(0.2);
  wait(1600, msec);
  IntakeStop();
  move(-24);
  turn(115);///
  move(30);
}

//positive side red solo AWP no alliance stake string doink
void Auton32()
{
  driveHold();
  setMoveConstants(2.4,0.4);
  //setMoveThreshold(0.2);
  move(-24);
  clamp.set(true);
  IntakeIn();
  wait(0.7, sec);
  turn(-105);
  IntakeIn();
  move(15);
  wait(1150, msec);
  turn(90);//
  IntakeStop();
  move(-12);
  //wait(100, msec);
  clamp.set(false);
  wait(300, msec);
  move(2);
  turn(1);//
  move(-21);
  clamp.set(true);
  IntakeIn();
  turn(47);
  extendDoink();
  move(60);
  wait(100, msec);
  retractDoink();
  wait(300, msec);
  setMovePowerLimit(40);
  move(-10);
  turn(137);
  //move(10);
}

//red negative side full goal touches bar
void Auton4()
{
  driveHold();
  setMoveConstants(2.4,0.4);
  move(-24);/////
  clamp.set(true);
  IntakeIn();
  move(-6);
  turn(90);///
  move(20);//22
  IntakeStop();
  turn(190);///200
  IntakeIn();//
  move(13);
  move(-11);//weee
  turn(169);////
  move(11);
  wait(850, msec);
  move(-32);
  turn(265);///
  move(26);
  clamp.set(false);
  IntakeStop();
  // wait(500, msec);
  // move(-24);
  // turn(-105);///
  // turn(180);
 // move(30);
}

void BNeg4(){
  driveHold();
  setMoveConstants(2.4,0.4);
  move(-24);/////
  clamp.set(true);
  IntakeIn();
  move(-6);
  turn(-90);///
  move(20);//22
  IntakeStop();
  turn(-190);///200
  IntakeIn();//
  move(13);  //2 piece
  move(-11);//weee
  turn(-170);////
  move(11);
  wait(850, msec);
  move(-35);
  turn(-266);///
  move(27);
  clamp.set(false);
  // wait(500, msec);
  // move(-24);
  // turn(-105);///
  // turn(180);
 // move(30);
}

void skillsAuton()
{
  //task colorstop;
  IntakeIn();
  wait(500, msec);
  swingRight(36);
  move(34);
  wait(500, msec);
  IntakeStop();
  turn(67);
  //colorStop = true;
  move(-45);  //need to figure out way to stop intake b4 here or make it back b4 going back
  //colorStop = false;
  clamp.set(true);
  IntakeIn();
  turn(-8);
  move(25);
  turn(-89);
  move(23);
  turn(-178);
  //setMovePowerLimit(66);
  move(38);
  //wait(300, msec);
  //move(12);
  //move(-12);
  turn(-45);
  move(12);
  //setMovePowerLimit(90);
  turn(24);//46
  move(-11);
  //turn(45);
  IntakeOut();
  //2ND stake
  //turn(-90);
  // //swingLeft(.2);//later change this bith so that after it gets the 2nd ring in line it just turns to the 3rd grabs its and rotate
  // turn(45);
  // move(-9);
  clamp.set(false);
  turn(48);//news
  IntakeIn();
  move(83);
  IntakeStop();
  turn(-23);//center to stake rjsn
  //IntakeStop();//try right after move80 
  move(-51);
  clamp.set(true);
  IntakeIn();
  turn(45);
  move(36);
  turn(30);
  move(25);
  move(-27);
  turn(178);
  move(35);
  //wait(200, msec);
  turn(45);
  move(12);
  turn(-24);
  move(-11);
  clamp.set(false);
  turn(-25);
  //move(50);
  //turn(-22);
  //move();
  //turn(-46);
  //IntakeOut();
  //clamp.set(false);
  wait(300, msec);
  //turn(30);
  IntakeIn();
  //enable friction mech
  //ploopyActivate();
  move(100);
  //wall stake shit
  // wait(200, msec);
  // move(-25);
  // turn(90); 
  // move(9.5);
  // IntakeStop();
  // scoreLift();
  // wait(250, msec);
  // resetLift();
  // turn(-59);
  // IntakeIn();
  // move(42);
  // turn(140);
  //move(-24);
  //clamp.set(true);
  //turn(-120);
  IntakeStop();
  turn(90);
  wait(300, msec);
  IntakeIn();
  move(26);
  IntakeStop();
  turn(-270);
  //IntakeStop();
  move(-29);
  
  //wait(100, msec);
  clamp.set(true);
  IntakeIn();
  turn(-127);
  setMovePowerLimit(75);
  move(72);//71
  wait(200, msec);
  turn(10);
  move(40);
  wait(200, msec);
  move(-12);
  turn(-20);
  move(15);
  turn(-38);
  extendDoink();
  move(8);
  turn(90);
  turn(145);
  move(-7);
  IntakeOut();
  clamp.set(false);
  turn(-95);
  move(-111);
}



void testAuton(){
  ploopyActivate();
  IntakeIn();
  wait(2, sec);
  IntakeStop();
  scoreLift();
  wait(200, msec);
  resetLift();
}

void AutonSelector(){
  //do the 515r methood nice lasercut circle with numbers
  int selectedAuton = Potentiometer.angle(degrees);
  int degA1 = 1;
  int degA2 = 20;
  int degA3 = 40;
  int degA4 = 60;
  printf("%d",selectedAuton);

  if(selectedAuton  >> degA1 && selectedAuton << degA2){
    Auton1();
  }
  else if(selectedAuton>> degA2 && selectedAuton << degA3){
    Auton2();
  }
  else if(selectedAuton>> degA3 && selectedAuton << degA4){
    Auton3();
  }
  else if(selectedAuton>> degA4){
    Auton4();
  }
}

// void ploopyActivate()
// {
//   task liftPloopyTask(liftPloop);
// }



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

  Controller1.ButtonB.pressed(Hanging);
  //Controller1.ButtonA.pressed(testTurn);
  Competition.drivercontrol(Driver);
  Competition.autonomous(BNeg4);
  //Competition.autonomous(AutonSelector); //for potentiometer
}