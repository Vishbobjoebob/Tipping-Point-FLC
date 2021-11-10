using namespace vex;

extern brain Brain;

// VEXcode devices
extern motor top_L;
extern motor bottom_L;
extern motor top_R;
extern motor bottom_R;
extern motor ringIntake;
extern motor intakeLeft;
extern motor intakeRight;
extern pot potentiometer ;
extern inertial inertial_Up;
extern inertial inertial_Down;
extern controller Controller1;
extern encoder leftTracker; 
extern encoder rightTracker;
extern motor_group leftDrive; 
extern motor_group intake;
extern motor_group rightDrive;
extern motor_group lift;
extern digital_out piston;
extern digital_out pistonTilter;
extern motor tilter;


/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Text.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void);