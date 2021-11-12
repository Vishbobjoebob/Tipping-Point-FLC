/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       71811N                                                    */
/*    Created:      11/6/2021                                                  */
/*    Description:  71811N Tipping Point Code                                 */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "opControl.h"

#include "auton.h"

#include "vex.h"

using namespace vex;

competition Competition;

void pre_auton(void) {
  vexcodeInit();

}

void autonomous(void) { 
  inertialCalibration();
  homeRowAuton();
}

void usercontrol(void) {
  task::sleep(500);
  resetEncoders();
  //piston.set(false);
  //inertialCalibration();
  while (1) {
    joyStickControl() ;
    //armMacro() ;
    intakeControl() ;
    conveyorControl();
    pistonControl();
    if(leftDrive.temperature(pct) > 60 && rightDrive.temperature(pct) > 60){
      Controller1.rumble("----");
    }
    
    //printf("Base temp: %f\n", front_L.temperature(pct));
    ///printf("Base temp: %f\n", back_L.temperature(pct));
    //float robotDirection = (-inertial_Up.rotation(deg) - inertial_Down.rotation(deg)) / 2;
    //printf("heading %f\n", robotDirection);
    //float position = (-(leftTracker.rotation(rev)) + (rightTracker.rotation(rev))) / 2;
    //printf("position %f\n", position);
    //printf("left %f\n", leftTracker.rotation(rev));
    //printf("right %f\n", rightTracker.rotation(rev));
    //printf("heading %f\n", get_average_inertial());
    //printf("encoder: %f\n", angleConvertor(get_average_encoder_deg_turn())); // 6.5, 
    /*printf("leftback: %f\n", back_L.velocity(pct));
    printf("rightfront: %f\n", front_R.velocity(pct));
    printf("rightback: %f\n", back_R.velocity(pct));*/
    /*printf("Light Sensor Top %ld\n", LineTrackerOuttake.reflectivity());*/
    wait(10, msec); 

  }
}

int main() {
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  while (true) {
    wait(100, msec);
  }
}