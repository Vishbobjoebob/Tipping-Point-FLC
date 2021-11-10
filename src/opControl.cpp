#include "vex.h"

#include "opControl.h"

using namespace vex;

double leftDriveSpeed, rightDriveSpeed, leftDriveCalculation, rightDriveCalculation;
double angle, prevAngle = 0;
bool manual = false;

/*-----------------------------------------------------------------------------*/
/** @brief     Base Control */
/*-----------------------------------------------------------------------------*/

void joyStickControl() {
    //printf("motor tmep: %.2f\n: ", leftDrive.temperature(pct)) ;
    leftDriveCalculation = (Controller1.Axis1.position() - (Controller1.Axis3.position()));
    rightDriveCalculation = (Controller1.Axis1.position() + (Controller1.Axis3.position()));

  
    
    if(fabs(leftDriveCalculation) >= 40){
      leftDriveSpeed = leftDriveCalculation; 
    }
    else {
      leftDriveSpeed = (leftDriveCalculation * 0.1);
    }

    if(fabs(rightDriveCalculation) >= 40){
      rightDriveSpeed = rightDriveCalculation; 
    }
    else {
      rightDriveSpeed = (rightDriveCalculation * 0.1);
    }
    leftDriveSpeed = sgn(leftDriveCalculation) * ( 0.01 *(pow(leftDriveCalculation, 2))); 
    rightDriveSpeed = sgn(rightDriveCalculation) * ( 0.01 *(pow(rightDriveCalculation, 2))); 


    /*
    while (angle > prevAngle) {
      leftDriveSpeed -= 0.1 ;
      rightDriveSpeed -= 0.1 ;
    }
    while (angle < prevAngle) {
      leftDriveSpeed += 0.1 ;
      rightDriveSpeed += 0.1 ;
    }
  */
    //printf("leftDrive %ld\n", Controller1.Axis3.position());
    //printf("rightDrive %ld\n", Controller1.Axis1.position());
    
    //leftDrive.spin (fwd, (Controller1.Axis2.position() + (Controller1.Axis1.position())), pct);
    //rightDrive.spin(fwd, (Controller1.Axis2.position() - (Controller1.Axis1.position())), pct);
    //printf("Left:  %2f\n", leftDriveSpeed);
    //printf("Right: %2f\n", rightDriveSpeed) ;

    if ((leftDriveSpeed > 0 && rightDriveSpeed > 0) || (leftDriveSpeed < 0 && rightDriveSpeed < 0)) {
      leftDrive.spin(fwd, -leftDriveSpeed * 0.50, pct);
      rightDrive.spin(fwd, -rightDriveSpeed * 0.50, pct);
    }
    else {
      leftDrive.spin(fwd, -leftDriveSpeed * 0.85, pct);
      rightDrive.spin(fwd, -rightDriveSpeed * 0.85, pct);
    }
    task tilter = task(tilterControl);
    task arm = task(armControl);
    task manual = task(manualControl);
    task conveyor = task(conveyorControl);
    task::sleep(10);
}

void intakeControl() {
  if (Controller1.ButtonL1.pressing())
  {
    intake.spin(fwd, 100, pct);
  }
       
  else if (Controller1.ButtonL2.pressing())
  {
    intake.spin(fwd, -60, pct);
  }

  else
  {
    intake.stop(brake) ;
  }
}

int armControl() {
  if (manual == false) {
    if (Controller1.ButtonR1.pressing()) {
      lift.rotateTo(-2.65, rotationUnits::rev, 200, velocityUnits::rpm, true);
    }

    else if (Controller1.ButtonR2.pressing()) {
      lift.rotateTo(0.0, rotationUnits::rev, 200, velocityUnits::rpm, true);
    }
  }
  else {
    if (Controller1.ButtonR1.pressing()) {
      lift.spin(fwd, -90, pct) ;
    }

    else if (Controller1.ButtonR2.pressing()) {
      lift.spin(fwd, 90, pct) ;
    }
    else {
      lift.stop(hold);
    }
  }

  return 1;
}

int conveyorControl() {
  if (manual == false) {
    if (Controller1.ButtonL1.pressing()) {
      ringIntake.spin(fwd, -90, pct) ;
    }

    else if (Controller1.ButtonL2.pressing()) {
      ringIntake.spin(fwd, 90, pct) ;
    }
  }
  else {
    if (Controller1.ButtonL1.pressing()) {
      ringIntake.spin(fwd, -90, pct) ;
    }

    else if (Controller1.ButtonL2.pressing()) {
      ringIntake.spin(fwd, 90, pct) ;
    }
    else {
      ringIntake.stop(hold);
    }
  }
  return 1;
}

void pistonControl() {
  if (Controller1.ButtonUp.pressing()) {
    piston.set(true);
  }
  else if (Controller1.ButtonDown.pressing()) {
    piston.set(false);
  }
}

int tilterControl(void) {
  if (manual==false) {
    if (Controller1.ButtonA.pressing()) {
      tilter.rotateTo(0.0, rotationUnits::rev, 100, velocityUnits::rpm, true) ;
    }

    else if (Controller1.ButtonB.pressing()) {
      tilter.rotateTo(-2.0, rotationUnits::rev, 100, velocityUnits::rpm, true) ;
    }
  }
  else {
    if (Controller1.ButtonA.pressing()) {
      tilter.spin(fwd, -90, pct) ;
    }

    else if (Controller1.ButtonB.pressing()) {
      tilter.spin(fwd, 90, pct) ;
    }
    else {
      tilter.stop(hold);
    }
  }
  return 1;
}

int manualControl() {
  if (Controller1.ButtonX.pressing()) {
    manual = true;
  }
  else if (Controller1.ButtonY.pressing()) {
    manual = false;
  }

  return 1;
}

void resetEncoders() {
  tilter.setPosition(0, rev);
  lift.setPosition(0, rev);
}
