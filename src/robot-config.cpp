#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

brain Brain;

motor top_R = motor(PORT9, ratio6_1, false);
motor bottom_R = motor(PORT10, ratio6_1, false);
motor top_L = motor(PORT1, ratio6_1, false);
motor bottom_L = motor(PORT2, ratio6_1, false);
motor intakeLeft = motor(PORT5, ratio6_1, false);
motor intakeRight = motor(PORT6, ratio6_1, true);
motor arm_L = motor(PORT4, ratio18_1, true);
motor arm_R = motor(PORT3, ratio18_1, false);
motor ringIntake = motor(PORT7, ratio6_1, true);
motor tilter = motor(PORT8, ratio36_1, true);
digital_out piston = digital_out(Brain.ThreeWirePort.C);

inertial inertial_Up = inertial(PORT11);
inertial inertial_Down = inertial(PORT20);
encoder leftTracker = encoder(Brain.ThreeWirePort.G); //flipped 
encoder rightTracker = encoder(Brain.ThreeWirePort.A);

motor_group   leftDrive( top_L, bottom_L);
motor_group   rightDrive( top_R, bottom_R);
motor_group   intake( intakeLeft, intakeRight);
motor_group   lift(arm_R, arm_L); 

controller Controller1 = controller(primary);

bool RemoteControlCodeEnabled = true;

void vexcodeInit(void) {
  // nothing to initialize
}