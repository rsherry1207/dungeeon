/*
//            _____                    _____                _____                      _____            _____                    _____          
//           /\    \                  /\    \              /\    \                    /\    \          /\    \                  /\    \         
//          /::\____\                /::\    \            /::\    \                  /::\____\        /::\    \                /::\    \        
//         /::::|   |               /::::\    \           \:::\    \                /:::/    /        \:::\    \              /::::\    \       
//        /:::::|   |              /::::::\    \           \:::\    \              /:::/    /          \:::\    \            /::::::\    \      
//       /::::::|   |             /:::/\:::\    \           \:::\    \            /:::/    /            \:::\    \          /:::/\:::\    \     
//      /:::/|::|   |            /:::/__\:::\    \           \:::\    \          /:::/    /              \:::\    \        /:::/__\:::\    \    
//     /:::/ |::|   |           /::::\   \:::\    \          /::::\    \        /:::/    /               /::::\    \      /::::\   \:::\    \   
//    /:::/  |::|___|______    /::::::\   \:::\    \        /::::::\    \      /:::/    /       ____    /::::::\    \    /::::::\   \:::\    \  
//   /:::/   |::::::::\    \  /:::/\:::\   \:::\    \      /:::/\:::\    \    /:::/    /       /\   \  /:::/\:::\    \  /:::/\:::\   \:::\ ___\ 
//  /:::/    |:::::::::\____\/:::/  \:::\   \:::\____\    /:::/  \:::\____\  /:::/____/       /::\   \/:::/  \:::\____\/:::/__\:::\   \:::|    |
//  \::/    / ~~~~~/:::/    /\::/    \:::\  /:::/    /   /:::/    \::/    /  \:::\    \       \:::\  /:::/    \::/    /\:::\   \:::\  /:::|____|
//   \/____/      /:::/    /  \/____/ \:::\/:::/    /   /:::/    / \/____/    \:::\    \       \:::\/:::/    / \/____/  \:::\   \:::\/:::/    / 
//               /:::/    /            \::::::/    /   /:::/    /              \:::\    \       \::::::/    /            \:::\   \::::::/    /  
//              /:::/    /              \::::/    /   /:::/    /                \:::\    \       \::::/____/              \:::\   \::::/    /   
//             /:::/    /               /:::/    /    \::/    /                  \:::\    \       \:::\    \               \:::\  /:::/    /    
//            /:::/    /               /:::/    /      \/____/                    \:::\    \       \:::\    \               \:::\/:::/    /     
//           /:::/    /               /:::/    /                                   \:::\    \       \:::\    \               \::::::/    /      
//          /:::/    /               /:::/    /                                     \:::\____\       \:::\____\               \::::/    /       
//          \::/    /                \::/    /                                       \::/    /        \::/    /                \::/____/        
//           \/____/                  \/____/                                         \/____/          \/____/                  ```````                            
*/                                                                                                                   

//Drivetrain control with trapezoidal motion profiles and PD, made by Matthew Estevez.

double gearRatio = 1.33333333333;            // (Driven Gear Teeth) / (Driving Gear Teeth), values less than 1 are torque ratios, greater than 1 are speed ratios.
double wheelDiameter = 2.75;       // Wheel diameter in inches. Old vex 4" wheels are actually 4.125".
double startingAngle = 0;          // Angle which the robot starts at during autonomous. Typically, you want the direction your alliance is facing to be 0 degrees.

////Move PD

double moveKp = 1.8;               // Proportional constant value, should be around 1.8. Higher values make the movement more accurate, but cause oscillations.
double moveKd = 0.9;               // Derivative constant value, should be around half of moveKp. Higher values dampen oscillations, but might cause the robot to react to strongly. It will make a jitter or rattling sound.
double moveStraightKp = 4.0;       // Alignment constant value, determines how much power to put in to keep the drivetrain straight. Should be around 1;
double moveThreshold = 0.1;        // The + or - amount of inches the move can be off by to exit the loop. should be around 0.1.
double movePowerLimit = 70;        // The maximum % voltage the drivetrain motors can be applied. Should be around 50 - 80.
//Turn PD Timing
double moveThresholdTime = 120;    // The amount of time in miliseconds it takes to exit the PD loop when it is within the threshold. Never less than 100.
double moveTimeoutTime = 5000;     // The amount of time in miliseconds it takes to interrupt the PD loop if it is not within the threshold.
//Move Motion Profile              // If you do not want to do motion profiled movements, set all these values to 100. I recommend not changing anything.
double movePowerStart = 40;        // The % voltage the drivetrain starts off at. Should be around 30 - 60. 
double movePowerEnd = 15;          // The maximum % voltage the drivetrain ends the movement with. Should be around 15 - 30.
double moveAcceleration = 10;      // The change in % voltage per inch when the robot is first accelerating. Should be around 10.
double moveDeceleration = 10;      // The change in % voltage per inch when the robot is decelerating. Should be around 10. Higher values will make stops faster.

////Turn PD

double turnKp = 1.4;               // Proportional value, should be around 1.4. Higher values make the movement more accurate, but cause oscillations.
double turnKd = 0.8;               // Derivative value, should be around half of turnKp. Higher values dampen oscillations, but might cause the robot to react too strongly.
double turnThreshold = 1.0;        // The + or - amount of degrees the turn can be off by to exit the loop. Should be around 1.
double turnPowerLimit = 70;        // The maximum % voltage the drivetrain motors can be applied. Should be around 50 - 80.
//Turn PD Timing
double turnThresholdTime = 120;    // The amount of time in miliseconds it takes to exit the PD loop when it is within the threshold. Never less than 100.
double turnTimeoutTime = 3000;     // The amount of time in miliseconds it takes to interrupt the PD loop if it is not within the threshold.
//Turn Motion Profile              // If you do not want to do motion profiled turns, set all these values to 100. I recommend not changing anything.
double turnPowerEnd = 15;          // The maximum % voltage the drivetrain ends the turn with. Should be around 15.
double turnDeceleration = 0.6;     // The change in % voltage per degree of rotation when the robot is decelerating. Should be around 0.5. Higher values will make stops faster.

////Swing PD

double swingKp = 1.9;              // Proportional value, should be around 1.8. Higher values make the movement more accurate, but cause oscillations.
double swingKd = 0.5;              // Derivative value, should be around a fourth of swingKp. Higher values dampen oscillations, but might cause the robot to react too strongly, "death rattle".
double swingThreshold = 1.0;       // The + or - amount of degrees the turn can be off by to exit the loop. Should be around 1.
double swingPowerLimit = 70;       // The maximum % voltage the drivetrain motors can be applied. Should be around 50 - 80.
//Swing PD Timing
double swingThresholdTime = 120;   // The amount of time in miliseconds it takes to exit the PD loop when it is within the threshold. Never less than 100.
double swingTimeoutTime = 3000;    // The amount of time in miliseconds it takes to interrupt the PD loop if it is not within the threshold.
//Swing Motion Profile             // If you do not want to have motion profiled swings, set all these values to 100. I recommend not changing anything.
double swingPowerEnd = 15;         // The maximum % voltage the drivetrain ends the swing with. Should be around 15.
double swingDeceleration = 0.8;    // The change in % voltage per degree of rotation when the robot is decelerating. Should be around 0.6. Higher values will make stops faster.

double inertialDerivativeGain = 4; // Decreases derivative when within x degrees of target by a factor of 1/x. Stops the derivative from oscillating when using the inertial sensor due to latency. This should be 4.

/* ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                                  Below are the things you don't change unless you know what you are doing.
*/ ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "vex.h"

const double voltCon = 0.12;
const double inchConversion = (360/(wheelDiameter*3.14159265358979))/gearRatio;
double currentAngle = startingAngle;
double Inertial = startingAngle;
motor* leftDrive[4];
motor* rightDrive[4];
timer timerPD;
int motorCountLeft, motorCountRight;
double leftAutonPower, rightAutonPower, leftDriverPower, rightDriverPower = 0;
bool timerActive, withinThreshold = false;
bool driverControl;
bool cubicArcadeControl = true;
bool cubicTankControl = false;
bool arcadeControl = false;
bool tankControl = false;
double target;
int driveMode = 0;
#define STOP 0
#define MOVE 1
#define TURN 2
#define SWINGLEFT 3
#define SWINGRIGHT 4

void leftDriveMotors(motor&drive1){leftDrive[0]=&drive1; motorCountLeft = 1;} // Set up for all configurations of driver control, condensed in an ugly way.
void leftDriveMotors(motor&drive1,motor&drive2){leftDrive[0]=&drive1;leftDrive[1]=&drive2; motorCountLeft = 2;}
void leftDriveMotors(motor&drive1,motor&drive2,motor&drive3){leftDrive[0]=&drive1;leftDrive[1]=&drive2;leftDrive[2]=&drive3; motorCountLeft = 3;}
void leftDriveMotors(motor&drive1,motor&drive2,motor&drive3,motor&drive4){leftDrive[0]=&drive1;leftDrive[1]=&drive2;leftDrive[2]=&drive3;leftDrive[3]=&drive4; motorCountLeft = 4;}

void rightDriveMotors(motor&drive1){rightDrive[0]=&drive1; motorCountRight = 1;}
void rightDriveMotors(motor&drive1,motor&drive2){rightDrive[0]=&drive1;rightDrive[1]=&drive2; motorCountRight = 2;}
void rightDriveMotors(motor&drive1,motor&drive2,motor&drive3){rightDrive[0]=&drive1;rightDrive[1]=&drive2;rightDrive[2]=&drive3; motorCountRight = 3;}
void rightDriveMotors(motor&drive1,motor&drive2,motor&drive3,motor&drive4){rightDrive[0]=&drive1;rightDrive[1]=&drive2;rightDrive[2]=&drive3;rightDrive[3]=&drive4; motorCountRight = 4;}

void driveLeft(double volts) // Sets left drive motors to a voltage.
{
  for (int i = 0; i < (motorCountLeft); i++){ leftDrive[i]->spin(forward, volts, voltageUnits::volt); }
}

void driveRight(double volts) // Sets right drive motors to a voltage.
{
  for (int i = 0; i < (motorCountRight); i++){ rightDrive[i]->spin(forward, volts, voltageUnits::volt); }
}

void driveLeftVelocity(double velocity) // Sets the left drive motors on the left side to spin at a specified % velocity.
{
  for (int i = 0; i < (motorCountLeft); i++){ leftDrive[i]->spin(forward); }
  for (int i = 0; i < (motorCountLeft); i++){ leftDrive[i]->setVelocity(velocity, percent); }
}

void driveRightVelocity(double velocity) // Sets the right drive motors on the left side to spin at a specified % velocity.
{
  for (int i = 0; i < (motorCountRight); i++){ rightDrive[i]->spin(forward); }
  for (int i = 0; i < (motorCountRight); i++){ rightDrive[i]->setVelocity(velocity, percent); }
}

void driveTorque(double Torque) // Sets all the drive motors to a certain torque, only affects setVelocity() function.
{
  for (int i = 0; i < (motorCountLeft); i++){ leftDrive[i]->setMaxTorque(Torque, percent); }
  for (int i = 0; i < (motorCountRight); i++){ rightDrive[i]->setMaxTorque(Torque, percent); }
}

void driveCoast() // Sets stopping of all drive motors to "coast".
{
  for (int i = 0; i < (motorCountLeft); i++){ leftDrive[i]->setStopping(coast); }
  for (int i = 0; i < (motorCountRight); i++){ rightDrive[i]->setStopping(coast); }
}

void driveBrake() // Sets stopping of all drive motors to "brake".
{
  for (int i = 0; i < (motorCountLeft); i++){ leftDrive[i]->setStopping(brake); }
  for (int i = 0; i < (motorCountRight); i++){ rightDrive[i]->setStopping(brake); }
}

void driveHold() // Sets stopping of all drive motors to "hold".
{
  for (int i = 0; i < (motorCountLeft); i++){ leftDrive[i]->setStopping(hold); }
  for (int i = 0; i < (motorCountRight); i++){ rightDrive[i]->setStopping(hold); }
}

void driveStop() // Stops all drive motors and interupts any PD loop that is running.
{
  for (int i = 0; i < (motorCountLeft); i++){ leftDrive[i]->stop(); }
  for (int i = 0; i < (motorCountRight); i++){ rightDrive[i]->stop(); }
  driveMode = STOP;
}

void driveLeftStop() // Stops left drive motors.
{
  for (int i = 0; i < (motorCountLeft); i++){ leftDrive[i]->stop(); }
}

void driveRightStop() // Stops right drive motors.
{
  for (int i = 0; i < (motorCountRight); i++){ rightDrive[i]->stop(); }
}

void drivePositionReset() // Sets measuring motors to position 0.
{
  leftDrive[0]->resetPosition();
  rightDrive[0]->resetPosition();
}

double motionProfile(double x, double endPoint, double slope1, double slope2, double yMin1, double yMin2, double yMax) // Creates a trapezoidal function for motion profiling.
{
  double y = 0;
  double y1 = (slope1 * x) + yMin1;              
  double y2 = (slope2 * (x - endPoint)) + yMin2; 
  if (y1 <= y2) 
  {
    y = y1;
    if (y < yMin1){y = yMin1;}
  }
  else if(y1 > y2) 
  {
    y = y2;
    if (y < yMin2){y = yMin2;}
  }
  if (y > yMax){y = yMax;}
  return y;
}

void updateHeading() // Called everytime inertial sensor updates. Adds starting angle. Updates the variable inertial.
{ 
  Inertial = InertialSensor.rotation(degrees) + startingAngle;
}

void exitThresholdPD() // Checks if PD is in the threshold after a certain time, automatically called by a timer. Exits PD loop.
{ 
  if(withinThreshold){
    withinThreshold = false;
    driveMode = STOP;
  }
}

void checkExitConditions(double error, double threshold, double thresholdTime, double timeoutTime) // Called continously, checks if PD has reached threshold, if so, a timer is started. If enough time has elapsed, it interupts the loop.
{
  if (fabs(error) < threshold)
  {
    withinThreshold = true;
    if (!timerActive){ timer().event(exitThresholdPD, thresholdTime); }
  }
    else{ withinThreshold = false; }
    if (timerPD.time(msec) > timeoutTime)
  {
    printf("\n Error: Timed out before reaching target");
    printf("\n Timed out at %lf", timerPD.time(msec));
    withinThreshold = false;
    driveMode = STOP;
  }
}

int drivetrainControl() // Drivetrain handler, runs PD loops and driver control.
{
  
  InertialSensor.changed(updateHeading);
  while (true)
  {
    if(driverControl)
    {
      if (cubicArcadeControl) // arcade joystick style, with cubic output curves.
      {
        leftDriverPower = pow(Controller1.Axis3.position(percent), 3)/(10000) + pow(Controller1.Axis1.position(percent), 3)/(10000);
        rightDriverPower = pow(Controller1.Axis3.position(percent), 3)/(10000) - pow(Controller1.Axis1.position(percent), 3)/(10000);
      }
      if (arcadeControl) // arcade joystick style.
      {
        leftDriverPower = Controller1.Axis3.position(percent) + Controller1.Axis1.position(percent);
        rightDriverPower = Controller1.Axis3.position(percent) - Controller1.Axis1.position(percent);
      }
      if (cubicTankControl)  // tank joystick style, with cubic output curves.
      {
        leftDriverPower = pow(Controller1.Axis3.position(percent), 3)/(10000);
        rightDriverPower = pow(Controller1.Axis2.position(percent), 3)/(10000);
      }
      if (tankControl) // tank joystick style.
      {
        leftDriverPower = Controller1.Axis3.position(percent);
        rightDriverPower = Controller1.Axis2.position(percent);
      }
    }else{leftDriverPower = 0; rightDriverPower = 0;}
    
    if(driveMode == MOVE) //PD loop and motion profile for moving.
    {
      double error = (target - ((rightDrive[0]->position(degrees) + leftDrive[0]->position(degrees))/2))/inchConversion;
      double power = ((error * (moveKp / voltCon)) + (-(rightDrive[0]->velocity(percent) + leftDrive[0]->velocity(percent))/2) * moveKd);
      double powerLimit = motionProfile(fabs(((rightDrive[0]->position(degrees) + leftDrive[0]->position(degrees))/2) / inchConversion), fabs(target/inchConversion), moveAcceleration, -moveDeceleration, movePowerStart, movePowerEnd, movePowerLimit);
      double anglePower = (currentAngle - Inertial) * moveStraightKp; 
      if (fabs(power) > powerLimit){ if(power > 0){power = powerLimit;} else{power = -powerLimit;} } 
      leftAutonPower = power + anglePower;
      rightAutonPower = power - anglePower;
      checkExitConditions(error, moveThreshold*inchConversion, moveThresholdTime, moveTimeoutTime);
    }

    if(driveMode == TURN) //PD loop and motion profile for turning.
    {
      double error = target - Inertial;
      double derivative = (rightDrive[0]->velocity(percent) + rightDrive[0]->velocity(percent))/2;
      if(fabs(error) < inertialDerivativeGain){ derivative *= fabs(error)/inertialDerivativeGain; };
      double power = ((error * (turnKp / voltCon)) + (derivative) * (turnKd / voltCon));
      double powerLimit = motionProfile(0, fabs(error), 100, -turnDeceleration, 100, turnPowerEnd, turnPowerLimit);
      if (fabs(power) > powerLimit){ if(power > 0){power = powerLimit;} else{power = -powerLimit;} }
      leftAutonPower = power;
      rightAutonPower = -power;
      checkExitConditions(error, turnThreshold, turnThresholdTime, turnTimeoutTime);
    }

    if(driveMode == SWINGLEFT) //PD loop and motion profile for swinging left.
    {
      double error = target - Inertial;
      double derivative = rightDrive[0]->velocity(percent);
      if(fabs(error) < inertialDerivativeGain){ derivative *= fabs(error)/inertialDerivativeGain; };
      double power = ((error * (swingKp / voltCon)) + (derivative) * (swingKd / voltCon));
      double powerLimit = motionProfile(0, fabs(error), 100, -swingDeceleration, 100, swingPowerEnd, swingPowerLimit);
      if (fabs(power) > powerLimit){ if(power > 0){power = powerLimit;} else{power = -powerLimit;} }
      rightAutonPower = -power;
      leftAutonPower = 0;
      checkExitConditions(error, swingThreshold, swingThresholdTime, swingTimeoutTime);
    }

    if(driveMode == SWINGRIGHT) //PD loop and motion profile for swinging right.
    {
      double error = target - Inertial;
      double derivative = -leftDrive[0]->velocity(percent);
      if(fabs(error) < inertialDerivativeGain){ derivative *= fabs(error)/inertialDerivativeGain; };
      double power = ((error * (swingKp / voltCon)) + (derivative) * (swingKd / voltCon));
      double powerLimit = motionProfile(0, fabs(error), 100, -swingDeceleration, 100, swingPowerEnd, swingPowerLimit);
      if (fabs(power) > powerLimit){ if(power > 0){power = powerLimit;} else{power = -powerLimit;} }
      leftAutonPower = power;
      rightAutonPower = 0;
      checkExitConditions(error, swingThreshold, swingThresholdTime, swingTimeoutTime);
    }

    if (driveMode == STOP) //Stops auton control.
    {
      leftAutonPower = 0;
      rightAutonPower = 0;
    }

    if (leftAutonPower != 0 || leftDriverPower != 0){ driveLeft((leftAutonPower + leftDriverPower) * voltCon); } else {driveLeftStop();} //Apply Power if auton or driver power is not zero, other wise stop motors.
    if (rightAutonPower != 0 || rightDriverPower != 0){ driveRight((rightAutonPower + rightDriverPower) * voltCon); } else {driveRightStop();}
    wait(10, msec);
  }
  return 0;
}

double coterm(double angle) //Calculates the closest coterminal angle to the parameter in the range -180 - 180.
{
  while(true)
  {
    if(angle - Inertial >= 180){angle -= 360;}
    else if(angle - Inertial <= -180){angle += 360;}
    else{break;}
  }
  return angle;
}

void setStartingAngle(double angle) //Sets the starting angle, should only be called before drivetrain control task is running.
{
  startingAngle = angle;
  currentAngle = startingAngle;
  Inertial = startingAngle;
}

double heading() //Returns inertial reading.
{
  return Inertial;
}

void moveAsync(double Distance) //Starts move PD loop in drivecontrol task and sets a target in inches, resets measuring motors to position 0.
{
  driveMode = STOP;
  drivePositionReset();
  target = Distance * inchConversion;
  driveMode = MOVE;
  timerPD.reset();
}

void move(double Distance) //Calls and waits on asynchronous PD loop.
{
  moveAsync(Distance);
  while(driveMode == MOVE){wait(10, msec);}
}

bool isMoving() // Returns true when a movement PD loop is running.
{
if (driveMode == MOVE){return true;}else{return false;}
}

void turnAsync(double Angle){ //Starts turn PD loop in drivecontrol task and sets a target in degrees, sets angle correction to the target angle.
  driveMode = STOP;
  currentAngle = Angle;
  target = Angle;
  driveMode = TURN;
  timerPD.reset();
}

void turn(double Angle) // Calls and waits on asynchronous PD loop.
{
  turnAsync(Angle);
  while(driveMode == TURN){wait(10, msec);}
}

bool isTurning() // Returns true when a turn PD loop is running.
{
  if (driveMode == TURN){return true;}else{return false;}
}

void swingLeftAsync(double Angle){
  driveMode = STOP;
  currentAngle = Angle;
  target = Angle;
  driveMode = SWINGLEFT;
  timerPD.reset();
}

void swingLeft(double Angle) // Waits on asynchronous PD loop.
{
  swingLeftAsync(Angle);
  while(driveMode == SWINGLEFT){wait(10, msec);}
}

void swingRightAsync(double Angle){ // Swings 
  driveMode = STOP;
  currentAngle = Angle;
  target = Angle;
  driveMode = SWINGRIGHT;
  timerPD.reset();
}

void swingRight(double Angle) // Waits on asynchronous PD loop.
{
  swingRightAsync(Angle);
  while(driveMode == SWINGRIGHT){wait(10, msec);}
}

bool isSwinging() // Returns true when a swing PD loop is running.
{
  if (driveMode == SWINGLEFT || driveMode == SWINGRIGHT){return true;}else{return false;}
}

void setCubicArcadeControl() // Activates arcade joystick style, with exponential input curve for turning.
{
  cubicArcadeControl = true;
}

void setCubicTankControl() // Activates tank joystick style, with exponential input curves.
{
  cubicTankControl = true;
}

void setTankControl() // Activates tank joystick style.
{
  tankControl = true;
}

void setArcadeControl() // Activates arcade joystick style.
{
  arcadeControl = true;
}

void setMoveConstants(double P, double D) // Sets move values.
{
  moveKp = P;
  moveKd = D;
}

void setMovePowerLimit(double limit) // Sets move values.
{
  movePowerLimit = limit;
}

void setMoveThreshold(double thres) // Sets move values.
{
  moveThreshold = thres;
}

void setMoveProfile(double start, double end, double accel, double decel) // Sets move values.
{
  movePowerStart = start;
  movePowerEnd = end;
  moveAcceleration = accel;
  moveDeceleration = decel;
}

void setTurnConstants(double P, double D) // Sets turn values.
{
  turnKp = P;
  turnKd = D;
}

void setTurnPowerLimit(double limit) // Sets turn values.
{
  turnPowerLimit = limit;
}

void setTurnThreshold(double thres) // Sets turn values.
{
  turnThreshold = thres;
}

void setTurnProfile(double end, double decel) // Sets turn values.
{
  turnPowerEnd = end;
  turnDeceleration = decel;
}

void setSwingConstants(double P, double D) // Sets swing values.
{
  swingKp = P;
  swingKd = D;
}

void setSwingPowerLimit(double limit) // Sets swing values.
{
  swingPowerLimit = limit;
}

void setSwingThreshold(double thres) // Sets swing values.
{
  swingThreshold = thres;
}

void setSwingProfile(double end, double decel) // Sets swing values.
{
  swingPowerEnd = end;
  swingDeceleration = decel;
}

void enableDrivetrain() // Starts drivetrain control task.
{
  task chassis(drivetrainControl);
}

void startDriver() // Starts driver control
{
  driverControl = true;
}