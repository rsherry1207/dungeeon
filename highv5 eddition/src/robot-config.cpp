#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor Drive1 = motor(PORT20, ratio6_1, true);
motor Drive2 = motor(PORT19, ratio6_1, true);
motor Drive3 = motor(PORT17, ratio6_1, true);
motor Drive4 = motor(PORT12, ratio6_1, false);
motor Drive5 = motor(PORT13, ratio6_1, false);
motor Drive6 = motor(PORT14, ratio6_1, false);
controller Controller1 = controller(primary);
inertial InertialSensor = inertial(PORT16);
digital_out clamp = digital_out(Brain.ThreeWirePort.A);
motor Intake = motor(PORT10, ratio6_1, false);
digital_out doink = digital_out(Brain.ThreeWirePort.B);
rotation Rotation3 = rotation(PORT3, false);
motor liftMotorA = motor(PORT2, ratio18_1, false);
motor liftMotorB = motor(PORT4, ratio18_1, true);
motor_group lift = motor_group(liftMotorA, liftMotorB);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}