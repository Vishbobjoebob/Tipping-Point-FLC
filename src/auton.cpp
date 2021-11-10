#include "vex.h"

#include "auton.h"

#include "tracking.h"

using namespace vex;

Tracking tracking;

int intakeSpeedPCT = 100;

int ballFinal = 1;

bool checkingI = true;

bool checkingT = true;

bool goingDown = false;

void inertialCalibration(){
  inertial_Up.calibrate();
   while (inertial_Up.isCalibrating()) {
     wait(100, msec);
  }

  inertial_Down.calibrate();
   while (inertial_Down.isCalibrating()) {
     wait(100, msec);
  }
}

void resetFunction() {
  bottom_L.resetRotation();
  bottom_R.resetRotation();
  top_L.resetRotation();
  top_R.resetRotation();
  leftTracker.resetRotation();
  rightTracker.resetRotation();
}

int rainbowlogo() {

  while (1 == 1) {
    Brain.Screen.drawImageFromFile("BlueGroup1.png", 100, 0); // each picture is 270 by 258
    task::sleep(1000);
    Brain.Screen.drawImageFromFile("BlueGroup2.png", 100, 0);
    task::sleep(1000);
    Brain.Screen.drawImageFromFile("BlueGroup3.png", 100, 0);
    task::sleep(1000);
    Brain.Screen.drawImageFromFile("BlueGroup4.png", 100, 0);
    task::sleep(1000);
    Brain.Screen.drawImageFromFile("BlueGroup5.png", 100, 0);
    task::sleep(1000);
    Brain.Screen.drawImageFromFile("BlueGroup6.png", 100, 0);
    task::sleep(1000);
  }
  task::sleep(1000);
}

/*-----------------------------------------------------------------------------*/
/** @brief     Auton Selector */
/*-----------------------------------------------------------------------------*/

int autonomousSelection = -1;

typedef struct _button {
  int xpos;
  int ypos;
  int width;
  int height;
  bool state;
  vex::color offColor;
  vex::color onColor;
  const char *label;
} button;

// Button array definitions for each software button. The purpose of each button
// data structure is defined above.  The array size can be extended, so you can
// have as many buttons as you wish as long as it fits.

button buttons[] = {{30, 30, 60, 60, false, 0xE00000, 0x0000E0, "Ally"},
                    //{150, 30, 60, 60, false, 0x303030, 0xD0D0D0, ""},
                    //{270, 30, 60, 60, false, 0x303030, 0xF700FF, ""},
                    {390, 30, 60, 60, false, 0x303030, 0xDDDD00, "Back"},
                    {30, 150, 60, 60, false, 0x404040, 0xC0C0C0, "ReRun"},
                    // {150, 150, 60, 60, false, 0x404040, 0xC0C0C0, ""},
                    //{270, 150, 60, 60, false, 0x404040, 0xC0C0C0, ""},
                    {390, 150, 60, 60, false, 0x404040, 0xC0C0C0, "Skill"}};

// forward ref
void displayButtonControls(int index, bool pressed);

/*-----------------------------------------------------------------------------*/
/** @brief      Check if touch is inside button */
/*-----------------------------------------------------------------------------*/
int findButton(int16_t xpos, int16_t ypos) {
  int nButtons = sizeof(buttons) / sizeof(button);

  for (int index = 0; index < nButtons; index++) {
    button *pButton = &buttons[index];
    if (xpos < pButton->xpos || xpos > (pButton->xpos + pButton->width))
      continue;

    if (ypos < pButton->ypos || ypos > (pButton->ypos + pButton->height))
      continue;

    return (index);
  }
  return (-1);
}

/*-----------------------------------------------------------------------------*/
/** @brief      Init button states */
/*-----------------------------------------------------------------------------*/
void initButtons() {
  int nButtons = sizeof(buttons) / sizeof(button);

  for (int index = 0; index < nButtons; index++) {
    buttons[index].state = false;
  }
}

/*-----------------------------------------------------------------------------*/
/** @brief      Screen has been touched */
/*-----------------------------------------------------------------------------*/
void userTouchCallbackPressed() {
  int index;
  int xpos = Brain.Screen.xPosition();
  int ypos = Brain.Screen.yPosition();

  if ((index = findButton(xpos, ypos)) >= 0) {
    displayButtonControls(index, true);
  }
}

/*-----------------------------------------------------------------------------*/
/** @brief      Screen has been (un)touched */
/*-----------------------------------------------------------------------------*/
void userTouchCallbackReleased() {
  int index;
  int xpos = Brain.Screen.xPosition();
  int ypos = Brain.Screen.yPosition();

  if ((index = findButton(xpos, ypos)) >= 0) {
    // clear all buttons to false, ie. unselected
    //      initButtons();

    // now set this one as true
    if (buttons[index].state == true) {
      buttons[index].state = false;
    } else {
      buttons[index].state = true;
    }

    // save as auton selection
    autonomousSelection = index;

    displayButtonControls(index, false);
  }
}

/*-----------------------------------------------------------------------------*/
/** @brief      Draw all buttons */
/*-----------------------------------------------------------------------------*/
void displayButtonControls(int index, bool pressed) {
  vex::color c;
  Brain.Screen.setPenColor(vex::color(0xe0e0e0));

  for (int i = 0; i < sizeof(buttons) / sizeof(button); i++) {

    if (buttons[i].state)
      c = buttons[i].onColor;
    else
      c = buttons[i].offColor;

    Brain.Screen.setFillColor(c);

    // button fill
    if (i == index && pressed == true) {
      Brain.Screen.drawRectangle(buttons[i].xpos, buttons[i].ypos,
                                 buttons[i].width, buttons[i].height, c);
    } else
      Brain.Screen.drawRectangle(buttons[i].xpos, buttons[i].ypos,
                                 buttons[i].width, buttons[i].height);

    // outline
    Brain.Screen.drawRectangle(buttons[i].xpos, buttons[i].ypos,
                               buttons[i].width, buttons[i].height,
                               vex::color::transparent);

    // draw label
    if (buttons[i].label != NULL)
      Brain.Screen.printAt(buttons[i].xpos + 8,
                           buttons[i].ypos + buttons[i].height - 8,
                           buttons[i].label);
  }
}

void setDriveSpeed(int leftSpeed, int rightSpeed){
  leftDrive.spin(fwd, leftSpeed, velocityUnits::pct);
  rightDrive.spin(fwd, rightSpeed, velocityUnits::pct);
}

int debugging(){
  while(true){
    printf("frontL %f\n", top_L.velocity(pct));
    printf("frontR %f\n", top_R.velocity(pct));
    printf("backL %f\n", bottom_L.velocity(pct));
    printf("backR %f\n", bottom_R.velocity(pct));
    task::sleep(100);
  }
  task::sleep(10);
}

void holdDrive(){
  leftDrive.stop(hold);
  rightDrive.stop(hold);
}

void brakeDrive(){
  leftDrive.stop(brake);
  rightDrive.stop(brake);
}

void coastDrive(){
  leftDrive.stop(coast);
  rightDrive.stop(coast);
}

void setIntakeSpeed(int speed){
  intake.spin(fwd, speed, pct);
}

void brakeIntake(){
  intake.stop(brake);
}


int ballC = 0;

bool senseTop = false;
bool senseBottom = false;





void brainRGB() {
  int hue = 0;
  int count = 0;
  int max = 325;
  Brain.Screen.setPenColor(black);

  while(count < max) {
    // Brain.Screen.printAt(1, 20, "Hue value: %d ", hue);
    // Brain.Screen.printAt(1, 40, "counter: %d ", count);
    Brain.Screen.render();
    Brain.Screen.setFillColor(hue);
    Brain.Screen.drawRectangle(0,0,480,280);
    hue++;
    count++;
  }
  while(count > 0) {
    // Brain.Screen.printAt(1, 20, "Hue value: %d ", hue);
    // Brain.Screen.printAt(1, 40, "counter: %d ", count);
    Brain.Screen.render();
    Brain.Screen.setFillColor(hue);
    Brain.Screen.drawRectangle(0,0,480,280);
    hue--;
    count--;
  }
}

void moveForward(int speed, double distanceToTravel) {
  double wheelDiameterIN = 4.125;
  double travelTargetCM = distanceToTravel; // this is the distance it goes which is set as a variable
  double circumfrence = wheelDiameterIN * 3.141592;
  double degreesToRotate = ((360 * travelTargetCM) / circumfrence);

  // bottom_L.setVelocity(speed, vex::velocityUnits::pct);
  // bottom_R.setVelocity(speed, vex::velocityUnits::pct);
  // top_L.setVelocity(speed, vex::velocityUnits::pct);
  // top_R.setVelocity(speed, vex::velocityUnits::pct);
  leftDrive.setVelocity(speed, vex::velocityUnits::pct);
  rightDrive.setVelocity(speed,vex::velocityUnits::pct);

  // bottom_L.rotateFor(degreesToRotate, vex::rotationUnits::deg, false);
  // bottom_R.rotateFor(degreesToRotate, vex::rotationUnits::deg, false);
  // top_L.rotateFor(degreesToRotate, vex::rotationUnits::deg, false);
  // top_R.rotateFor(degreesToRotate, vex::rotationUnits::deg, true);
  leftDrive.rotateFor(degreesToRotate, vex::rotationUnits::deg);
  rightDrive.rotateFor(degreesToRotate, vex::rotationUnits::deg);
}

void moveForwardSimple(int speed) {
  bottom_L.spin(fwd, speed, pct);
  bottom_R.spin(fwd, speed, pct);
  top_L.spin(fwd, speed, pct);
  top_R.spin(fwd, speed, pct);
}

void strafeSimpleRight(int speed) {
  bottom_R.spin(fwd, speed, pct);
  top_L.spin(fwd, speed, pct);
}

void strafeSimpleLeft(int speed) {
  bottom_L.spin(fwd, speed, pct);
  top_R.spin(fwd, speed, pct);
}

float angleConvertor(double ticks) {
  double ticksPerTurn = -1120; //2050
  double angleOfRobot = (ticks * 360) / ticksPerTurn;
  return angleOfRobot;
}

int conversion(double degree) {
  double ticksPerTurn = 1810; //2050
  double ticks = (degree * ticksPerTurn) / 360;
  double degreesToRotate = ticks;
  return degreesToRotate;

}

float get_average_encoder() {
  float position = (-(leftTracker.rotation(rev)) + (rightTracker.rotation(rev))) / 2;
  return position;
}

float get_average_encoder_deg_turn() {
  float position = ((leftTracker.rotation(degrees)) + (rightTracker.rotation(degrees))) / 2;
  return position;
}

float get_average_encoder_deg() {
  float position = (-(leftTracker.rotation(degrees)) + (rightTracker.rotation(degrees))) / 2;
  return position;
}

float get_average_inertial() {
  float robotDirection = (inertial_Up.rotation(deg) + inertial_Down.rotation(deg))/2 ;
  return robotDirection;
}

double calculateLeftSpeed(double speed, int turningRadius){
 double angularSpeed = ((speed * 2) * (2 * M_PI)) / (60);
 double leftSpeed = (angularSpeed) * (turningRadius - 11.5);
 leftSpeed = leftSpeed / 2;
 return leftSpeed;
}

double calculateRightSpeed(double speed, int turningRadius){
 double angularSpeed = ((speed * 2) * (2 * M_PI)) / (60);
 double rightSpeed = (angularSpeed) * (turningRadius + 11.5); 
 rightSpeed = rightSpeed / 2; 
 return rightSpeed;
}

static const double circumference = 3.14159 * 2.77;

//slope based y2 - y1 / x2 - x1 
// y = speed 
// x = distance
// returns acceleration speed

double increasing_speed(double currentPosition, double endPosition, double maxSpeed) {
  double acceleration_constant;
  endPosition = ((360 * endPosition) / circumference);
  double minimumVelocity = 50; 
  acceleration_constant =  (maxSpeed - minimumVelocity) / (endPosition - 0);
  return acceleration_constant * fabs(currentPosition) + (minimumVelocity);
}

//heading PID simple P loop also added a D loop
//helps dealing with variablistic headings more reactive compared to a simple p loop

PID sHeadingPid;

int iHeadingPid(float target) {
  sHeadingPid.kP = 4;
  sHeadingPid.kI = 0;
  sHeadingPid.kD = 0.01 ;

  sHeadingPid.current = get_average_inertial();
  sHeadingPid.error = target - sHeadingPid.current;
  sHeadingPid.integral += sHeadingPid.error;
  sHeadingPid.derivative = sHeadingPid.error - sHeadingPid.lastError;
  sHeadingPid.lastError = sHeadingPid.error;

  return (((sHeadingPid.error) * (sHeadingPid.kP)) + ((sHeadingPid.derivative) * (sHeadingPid.kD)) + ((sHeadingPid.integral) * (sHeadingPid.kI)));
}

//move pid for your drive
//pid runs seperatly on right drive and left drive so they are not averages of one another

PID sMovePid;

int iMovePid(float target, double currentValue) {
  sMovePid.kP = 0.2; //0.13
  sMovePid.kI = 0;
  sMovePid.kD = 0.2;

  sMovePid.current = currentValue;
  sMovePid.error = target - sMovePid.current;
  sMovePid.integral += sMovePid.error;
  sMovePid.derivative = sMovePid.error - sMovePid.lastError;
  sMovePid.lastError = sMovePid.error;

  return (((sMovePid.error) * (sMovePid.kP)) + ((sMovePid.derivative) * (sMovePid.kD)) + ((sMovePid.integral) * (sMovePid.kI)));
}

PID sSpeedPid;

//velocity pid to change and make sure velcotiy was at what we wanted it to be at

/*float iSpeedPid(float target, bool leftSide) {
  sSpeedPid.kP = 0.16; //2
  sSpeedPid.kD = 0;
  sSpeedPid.kI = 0;

  if(leftSide == true){
  sSpeedPid.current = leftDrive.velocity(pct);
  }
  else{
  sSpeedPid.current = rightDrive.velocity(pct);
  }
  sSpeedPid.error = target - sSpeedPid.current;
  if(leftSide == true){
  printf("error left %f\n", sSpeedPid.error);
  }
  else{
  printf("error right %f\n", sSpeedPid.error);
  }
  sSpeedPid.integral += sSpeedPid.error;
  sSpeedPid.derivative = sSpeedPid.error - sSpeedPid.lastError;
  sSpeedPid.lastError = sSpeedPid.error;

  return (((sSpeedPid.error) * (sSpeedPid.kP)) + ((sSpeedPid.derivative) * (sSpeedPid.kD)) + ((sSpeedPid.integral) * (sSpeedPid.kI)));
}*/

bool switchStatement = false; 

double headingError = 0;
double headingErrorTest = 0;
double pogChamp = 0;  
double rightTrackerError = 0;
double driftLeftError = 0, driftRightError = 0, combinedDriftError = 0, combinedDriftErrorMultiply = 0;  

void moveForwardWalk(double distanceIn, double maxVelocity, double headingOfRobot, double multiply, double endOfSlew, bool cancel=true, bool slew=true){ //7 first distance, second max velocity, third desired heading, fourth max value you want for heading, fifth distance you want to accelerate for, sixth cancel function, seventh do you want to slew
  static const double circumference = 3.14159 * 4.125;
  if (distanceIn == 0)
    return;
  double directionLeft = distanceIn > 0 ? 1.0 : -1.0;
  double direction = distanceIn > 0 ? 1.0 : -1.0;
  double wheelRevsDegree = ((360 * distanceIn) / circumference);
  resetFunction();
  double leftStartPoint = (leftTracker.rotation(rotationUnits::rev));
  double rightStartPoint = (rightTracker.rotation(rotationUnits::rev));
  double leftSpeed, rightSpeed; 
  double offset = 0;
  
  int sameEncoderValue = 0;
  double distanceTraveledRight = 0;
  double distanceTraveledLeft = 0;
  double PIDPowerL, PIDPowerR, PIDPowerHeading; 
  bool rightDriveComplete = false;
  bool leftDriveComplete = false;

  while (true) {
    
    distanceTraveledLeft =  leftTracker.rotation(degrees);
    distanceTraveledRight = (rightTracker.rotation(degrees));
    printf("Distance Left %.4f\n", distanceTraveledLeft);
    printf("Distance Right %.4f\n", distanceTraveledRight);

    if (((fabs(leftDrive.velocity(pct)) < 1) || (fabs(rightDrive.velocity(pct)) < 1)) && (cancel == true) && fabs(distanceTraveledRight) > 0.1) { //cancel function loop
      ++sameEncoderValue;
    }

    if(sameEncoderValue > 10){
      break;
    }
    

    //y\ =\ 2\cos\left(\frac{x}{20}\right)\ +2

    PIDPowerHeading = iHeadingPid(headingOfRobot); //pid heading calculated
    headingError = fabs(PIDPowerHeading) < multiply ? PIDPowerHeading : multiply * (PIDPowerHeading / fabs(PIDPowerHeading));

    if (direction == fabs(direction)) {
      headingError = headingError;   //can remove this statment if you want
    } else {
      headingError = headingError;
    }

    //printf("heading %f\n", get_average_inertial());
   // printf("headingError %f\n", PIDPowerHeading); 
    //printf("distanceTraveled %f\n", distanceTraveled);

      if (direction * (distanceTraveledLeft - 0) < direction * wheelRevsDegree) {
        if (increasing_speed(distanceTraveledLeft, endOfSlew, maxVelocity) < maxVelocity && slew == true) {
          leftSpeed = increasing_speed(distanceTraveledLeft, endOfSlew, maxVelocity);
          leftSpeed = (directionLeft * (leftSpeed - headingError));
          leftDrive.spin(fwd, leftSpeed, pct);
        } else {
          PIDPowerL = iMovePid(wheelRevsDegree, distanceTraveledLeft);
          PIDPowerL = fabs(PIDPowerL) < maxVelocity ? PIDPowerL : maxVelocity * (PIDPowerL / fabs(PIDPowerL));
          PIDPowerL = ((PIDPowerL - headingError));
          //printf("left Speed %f\n", PIDPowerL);
          leftDrive.spin(fwd, (PIDPowerL * 0.01) * 12, volt);
        }
        //printf("left Speed Real %f\n", leftDrive.velocity(pct));
        /*if ((direction * (wheelRevs - 0.1)) > fabs(distanceTraveled) && fabs(distanceTraveled) > 0.1) {
          //leftSpeed += iSpeedPid(leftSpeed, true);
        }
        //leftDrive.spin(fwd, leftSpeed, pct);*/
      } else {  
        leftDrive.stop(brake);
        leftDriveComplete = true;
      }
      if (direction * (distanceTraveledRight - 0) < direction * wheelRevsDegree) {
        if (increasing_speed(distanceTraveledRight, endOfSlew, maxVelocity) < maxVelocity && slew == true){
          rightSpeed = increasing_speed(distanceTraveledRight, endOfSlew, maxVelocity);
          rightSpeed = (directionLeft * (rightSpeed + headingError));
          rightDrive.spin(fwd, -rightSpeed, pct);
        } else {
          PIDPowerR = iMovePid(wheelRevsDegree, distanceTraveledRight);
          PIDPowerR = fabs(PIDPowerR) < maxVelocity ? PIDPowerR : maxVelocity * (PIDPowerR / fabs(PIDPowerR));
          PIDPowerR = ((PIDPowerR + headingError));
          //printf("right Speed %f\n", PIDPowerR);
          rightDrive.spin(fwd, -(PIDPowerR * 0.01) * 12, volt);
        }
       // printf("right Speed Real %f\n", rightDrive.velocity(pct));
        /*if ((direction * (wheelRevs - 0.1)) > fabs(distanceTraveled) && fabs(distanceTraveled) > 0.1) {
          //rightSpeed += iSpeedPid(rightSpeed, false);
        }
        //rightDrive.spin(fwd, rightSpeed, pct);*/
      } else {
        rightDrive.stop(brake);
        rightDriveComplete = true; 
      }
      if(leftDriveComplete == true && rightDriveComplete == true){
        break;
      }
      //distanceTraveledlast = distanceTraveled; 
      task::sleep(10);
    }
  brakeDrive();
  switchStatement = false; 
}

void forwardPID(double target, double headingVal, double counterThresh, double accuracy) {
  // Constants
  double kP = 0.13;
  double kPAngle = 4;//4
  double kI = 0.09;//.09
  double kD = 0.3;//.3
  double kDAngle = 6;//6

  double errorRight = 0;
  double errorLeft = 0;
  double errorInertial = 0;
  double totalErrorRight = 0;
  double totalErrorLeft = 0;
  double prevErrorRight = 0;
  double prevErrorLeft = 0;
  double derivativeRight = 0;
  double derivativeLeft = 0;
  double counterLeft = 0;
  double counterRight = 0;
  double counterLeft2 = 0;
  double counterRight2 = 0;
  double prevErrorInertial = 0;
  double derivativeInertial = 0;
  double limit = 0;
  double accuracy2 = 180;
  double counterThresh2 = 300;
  bool run = true;

  // Resets the sensor values and then sets the current sensor values to the
  // sensors
  resetFunction();
  double trackingWheelRight = fabs(rightTracker.rotation(deg));
  double trackingWheelLeft = fabs(leftTracker.rotation(deg));
  errorInertial = prevErrorInertial;


  while (run) {
    //Update sensor values
    
    trackingWheelRight = fabs(rightTracker.rotation(deg));
    trackingWheelLeft = fabs(leftTracker.rotation(deg));
    errorInertial = headingVal - (-get_average_inertial());

    // Update the limit
    limit += 7;

    // Proportional
    errorLeft = target - trackingWheelRight;
    errorRight = target - trackingWheelLeft;

    // Integral
    totalErrorRight += errorRight;
    totalErrorLeft += errorLeft;


    // introduces I term when needed

    if (errorRight > 150) {
      totalErrorRight = 0;
    }

    if (fabs(errorRight) < 150) {
      totalErrorRight = 20;
    }

    // Derivative
    derivativeRight = errorRight - prevErrorRight;

    if (errorLeft > 150) {
      totalErrorLeft = 0;
    }

    if (fabs(errorLeft) < 150) {
      totalErrorLeft = 20;
    }

    // Derivative
    derivativeLeft = errorLeft - prevErrorLeft;

    // Derivative Inertial
    derivativeInertial = errorInertial - prevErrorInertial;

   

    // Find the speed of chassis based of the sum of the constants
    double motorPowerRight = (kP * errorRight) + (kI * totalErrorRight) + (kD * derivativeRight);
    double motorPowerLeft = (kP * errorLeft) + (kI * totalErrorLeft) + (kD * derivativeLeft);
    double heading = (kPAngle * errorInertial) + (kDAngle * derivativeInertial);


    // If the motorPower is larger then the limit, the motor power will equal

    // the limit

    if (motorPowerLeft > 90) {
      motorPowerLeft = 90;
    }

    if (motorPowerRight > 90) {
      motorPowerRight = 90;
    }

    if (limit < motorPowerRight) {
      motorPowerRight = limit;
    }
 
    if (fabs(motorPowerRight) < 10) {
      motorPowerRight = 10;
    }

    if (limit < motorPowerLeft) {
      motorPowerLeft = limit;
    }
 
    if (fabs(motorPowerLeft) < 10) {
      motorPowerLeft = 10;
    }

    // Sets the speed of the drive
    top_L.spin(directionType::fwd, 110 * (motorPowerLeft + heading),
            voltageUnits::mV);
    bottom_L.spin(directionType::fwd, 110 * (motorPowerLeft + heading),
            voltageUnits::mV);
    top_R.spin(directionType::fwd, 110 * (motorPowerRight - heading),
            voltageUnits::mV);
    bottom_R.spin(directionType::fwd, 110 * (motorPowerRight - heading),
            voltageUnits::mV);

    prevErrorRight = errorRight;
    prevErrorLeft = errorLeft;
    prevErrorInertial = errorInertial;

    if (fabs(errorRight) <= accuracy) {
      counterRight += 10;
    }
    if (fabs(errorRight) >= accuracy) {
      counterRight = 0;
    }
    if (fabs(errorLeft) <= accuracy) {
      counterLeft += 10;
    }
    if (fabs(errorLeft) >= accuracy) {
      counterLeft = 0;
    }

    if (fabs(errorRight) <= accuracy2) {
      counterRight2 += 10;
    }
    if (fabs(errorRight) >= accuracy2) {
      counterRight2 = 0;
    }
    if (fabs(errorLeft) <= accuracy2) {
      counterLeft2 += 10;
    }
    if (fabs(errorLeft) >= accuracy2) {
      counterLeft2 = 0;
    }

    if (counterLeft2 > counterThresh2 && counterRight2 > counterThresh2){
      run = false;
    }

    if (counterLeft > counterThresh && counterRight > counterThresh) {
      run = false;
    }

    

    task::sleep(10);
  }
  // When the loop ends, the motors are set to brake for less uncertainty and
  // then set the coast for drive control
  //printf("tracker%f\n", tracker.rotation(degrees));
  brakeDrive();
}

bool exit_function = false;

PID sRotatePid;
double;

int iRotatePid(int target, double kP, double kI, double kD) {
  sRotatePid.kP = kP; //0.48
  sRotatePid.kI = kI;  //0.1
  sRotatePid.kD = kD; //0.01

  sRotatePid.current = get_average_inertial();
  sRotatePid.error = target - sRotatePid.current;
  if(fabs(sRotatePid.error) > 3){
    sRotatePid.integral = 0;
  }
  else{ 
     sRotatePid.integral = 50;
  }
  sRotatePid.derivative = sRotatePid.error - sRotatePid.lastError;
  sRotatePid.lastError = sRotatePid.error;

  return (((sRotatePid.error) * (sRotatePid.kP)) + ((sRotatePid.derivative) * (sRotatePid.kD)) + ((sRotatePid.integral) * (sRotatePid.kI)));
}

void wait_until_drive_settled(int angle) {
  wait(10, msec);
  int maxPower = 10;
  int maxError = 10;
  //int waiting;
  wait(10, msec);
  //waiting++;
  while (1 == 1) {
    if (fabs(angle - get_average_inertial()) < maxError) {
      break;
    } else {
      int PIDPower = iRotatePid(angle);
      int power = abs(PIDPower) < maxPower ? PIDPower : maxPower * (PIDPower / abs(PIDPower));
      top_R.spin(fwd, -power, velocityUnits::pct);
      bottom_R.spin(fwd, -power, velocityUnits::pct);
      top_L.spin(fwd, power, velocityUnits::pct);
      bottom_L.spin(fwd, power, velocityUnits::pct);
      wait(10, msec);
    }
    wait(10, msec);
  }
  top_R.stop(hold);
  bottom_R.stop(hold);
  top_L.stop(hold);
  bottom_L.stop(hold);
}

void rotatePID(int angle) {
  int maxError = 20;
  int maxPower = 80;
  exit_function = false;

  sRotatePid.integral = 0;

  while (fabs(get_average_inertial() - angle) > maxError && !exit_function) {
    int PIDPower = iRotatePid(angle);
    int power = abs(PIDPower) < maxPower ? PIDPower : maxPower * (PIDPower / abs(PIDPower));
    top_R.spin(fwd, -power, velocityUnits::pct);
    bottom_R.spin(fwd, -power, velocityUnits::pct);
    top_L.spin(fwd, power, velocityUnits::pct);
    bottom_L.spin(fwd, power, velocityUnits::pct);
    wait_until_drive_settled(angle);
    wait(10, msec);
  }

  top_R.stop(hold);
  bottom_R.stop(hold);
  top_L.stop(hold);
  bottom_L.stop(hold);

}

void rotatePID(int angle, int maxPower, double kP, double kI, double kD) {
  resetFunction();
  double maxError = 3;
  static uint64_t c;
  int timer = 0;
  double cancelStart = false;
  double minVelocity = 0.5;
  exit_function = false;
  while (!exit_function) {
    int PIDPower = iRotatePid(angle, kP, kI, kD);
    printf(" turn heading %f\n", get_average_inertial());
    printf(" up inertial %f\n", inertial_Up.rotation(deg));
    printf(" down inertial %f\n", inertial_Down.rotation(deg));
    int power = abs(PIDPower) < maxPower ? PIDPower : maxPower * (PIDPower / abs(PIDPower));
    leftDrive.spin(fwd,  -power, velocityUnits::pct);
    rightDrive.spin(fwd, -power, velocityUnits::pct);
    if (fabs(get_average_inertial() - angle) < maxError || cancelStart == true){
      if(c++ == 0){
      cancelStart = true;
      timer = 0;
      printf("timer");
      }
      if(timer > 350){
      exit_function = true;
      }
    }
    wait(10, msec);
    timer += 10;
  }
  brakeDrive();
}

void arcPID(int angle, int maxPower, double kP, double kI, double kD, double leftProp, double rightProp) {
  resetFunction();
  double maxError = 3;
  static uint64_t c;
  int timer = 0;
  double cancelStart = false;
  double minVelocity = 0.5;
  exit_function = false;
  while (!exit_function) {
    int PIDPower = iRotatePid(angle, kP, kI, kD);
    printf(" turn heading %f\n", get_average_inertial());
    printf(" up inertial %f\n", inertial_Up.rotation(deg));
    printf(" down inertial %f\n", inertial_Down.rotation(deg));
    int power = abs(PIDPower) < maxPower ? PIDPower : maxPower * (PIDPower / abs(PIDPower));
    leftDrive.spin(fwd,  -power*leftProp, velocityUnits::pct);
    rightDrive.spin(fwd, power*rightProp, velocityUnits::pct);
    if (fabs(get_average_inertial() - angle) < maxError || cancelStart == true){
      if(c++ == 0){
      cancelStart = true;
      timer = 0;
      printf("timer");
      }
      if(timer > 350){
      exit_function = true;
      }
    }
    wait(10, msec);
    timer += 10;
  }
  brakeDrive();
}
void rotatePIDWack(int angle, int maxPower, bool leftSide) {
  double maxError = 0.1;
  int timer = 0;
  double minVelocity = 4;
  exit_function = false;
  while (fabs(get_average_inertial() - angle) > maxError && !exit_function) {
    int PIDPower = iRotatePid(angle);
    printf("heading average  %f\n", get_average_inertial());
    int power = abs(PIDPower) < maxPower ? PIDPower : maxPower * (PIDPower / abs(PIDPower));
    if(leftSide == 1){
    rightDrive.stop(hold);
    leftDrive.spin(fwd,  -power, velocityUnits::pct);
    }
    else{
    rightDrive.spin(fwd, power, velocityUnits::pct);
    }
    if (timer > 100 && fabs(leftDrive.velocity(pct)) < minVelocity) {
      exit_function = true;
    }
    wait(10, msec);
    timer += 10;
  }
  brakeDrive();
}

int counter = 0 ;
int armMacro() {
  double timer = 0 ;
  bool holding = true;
  while (Controller1.ButtonA.pressing()) {
    wait(10, msec) ;
    timer += 0.01;  
  }

  if (timer < 0.5 && timer != 0) {
    holding = false ;
  }
  else {
    holding = true ;
  }

  if (holding == false && counter % 2 == 0) {
    while (potentiometer.angle(degrees)  > 0.6714) {
      lift.spin(fwd, 90, pct) ;
    }
    counter += 1;
  }
  else if (holding == false && counter % 2 == 1) {
    while (potentiometer.angle(degrees)  < 60) {
      lift.spin(fwd, -90, pct) ;
    }
    counter += 1;
  }
  else {
    lift.stop(hold) ;
  }
  task::sleep(10) ;
  return 1;
}

void rightPivotTurn(int speed, int angle, double turningRadius){ 
double angularSpeed = ((speed * 2) * (2 * M_PI)) / (60);
double leftSpeed = (angularSpeed) * (turningRadius - 6);
double rightSpeed = (angularSpeed) * (turningRadius + 6) ;  
 while(get_average_inertial() < angle){
  top_R.spin(fwd, rightSpeed, rpm);
  top_L.spin(fwd, leftSpeed, rpm);
  bottom_R.spin(fwd , rightSpeed, rpm);
  bottom_L.spin(fwd , leftSpeed, rpm);
  printf("left speed  %f\n", leftSpeed);
  printf("right speed  %f\n", rightSpeed);
  task::sleep(10);
 }
 holdDrive();
}

bool linedUp = false;
/*-----------------------------------------------------------------------------*/
/** @brief      Turn Bot towards object */
/*-----------------------------------------------------------------------------*/

bool reached = false;
/*-----------------------------------------------------------------------------*/
/** @brief      Go toward set color */
/*-----------------------------------------------------------------------------*/




bool whenIntakingPrime = false; 
bool startConveyorToGoDown = false;
int counterForSigs = 0; 

int timeKeeper;

bool waitTillOver = false; 
int threshold = 40;
bool cancel = false;


void strafeWhileTurning(int speed, double distance){
 while(get_average_inertial() < 89){ 
  bottom_L.spin(fwd, speed, pct); 
  top_R.spin(fwd, speed * 4, pct); 
  top_L.spin(fwd, -speed * 4, pct); 
  printf("heading average  %f\n", get_average_inertial()); 
  task::sleep(10); 
} 
strafeWalk(-10, 80, 90, 0.6, 0); 
}

int intakeOn() {
  while (true) {
    intake.spin(directionType::fwd, intakeSpeedPCT, voltageUnits::volt);
    }
  }


int whenToStop = 0;

bool canceled = false;
bool tip = false;
bool booga = false;
bool skill = false;





/*void createPrimeTask(){
  task poop = task(primeShoot);
}

void stopPrimeTask(){
  task::stop(primeShoot);
}*/

void createIntakeOnTask(){
  task ughh = task(intakeOn);
}

void stopIntakeOn(){
  task::stop(intakeOn);
  brakeIntake();
}


int unfoldProcedure() {
  tilter.rotateTo(-2, rotationUnits::rev, 100, velocityUnits::rpm, true) ;
  return 1;
}

int dumpRing() {
  tilter.rotateTo(0.2, rotationUnits::rev, 100, velocityUnits::rpm, true);
  ringIntake.spin(fwd, -90, pct) ;

  return 1;
}

void brakeTilter() {
  tilter.stop(hold);
  ringIntake.stop(hold);
}

void createTilterUnfoldTask() {
  task tilter = task(unfoldProcedure);
}

void stopUnfold() {
  task::stop(unfoldProcedure);
  brakeTilter();
}




int stopIntakeFunction2nd(){ 
  int x = 0; 
  int timeLimit = 1000;
  while(x < timeLimit){
    x+= 10 ; 
    task::sleep(10);
  }
  if(x >= timeLimit){
    stopIntakeOn();
  }
  return 1;
}

void outtakeIntakes(double revolutions, int speed){ 
  intake.rotateFor(fwd, revolutions, rev, speed, velocityUnits::pct, false);
}

void preAuton() {

  inertial_Up.calibrate();
  while(inertial_Up.isCalibrating()){
    wait(100, msec);
  }

  inertial_Down.calibrate();
  while(inertial_Down.isCalibrating()){
    wait(100,msec);
  }

  Controller1.Screen.setCursor(1, 0);
  Controller1.Screen.print("IMU done calibrating");

  displayButtonControls(0, true);
  Brain.Screen.pressed(userTouchCallbackPressed);
  Brain.Screen.released(userTouchCallbackReleased);
  rainbowlogo();
  resetFunction();
}

/*void center4GoalAuton(){
  createBallCountTask();
  setIntakeSpeed(-100);
  indexer.spin(fwd, 100, pct);
  task::sleep(75);
  moveForwardWalk(34, 90, 0, 1000, false);
  brakeIntake();
  indexer.stop(); 
  createIntakeOnTask();
  rotatePID(90, 90);
  task sto = task(stopIntakeFunction);
  moveForwardWalk(36, 100, 90, 1000);
  outtake1BallAuton();
  task::stop(sto);
  indexer.stop();
  intake.spin(reverse, 100, pct);
  task::sleep(75);
  moveForwardWalk(-32, 90, 90, 2);
  rotatePID(-45, 80);
  task::sleep(50);
  createIntakeOnTask();
  moveForwardWalk(23, 90, -45, 1000);
  task::sleep(75);
  stopIntakeOn();
  rotatePID(44, 90);
  moveForwardWalk(-24, 40, 44, 0.6);
  rotatePID(-45, 90);
  moveForwardWalk(-0.5, 90, -45, 1000);
  rotatePID(-88, 90);
  moveForwardWalk(12, 90, -86, 1000);
  outtake1BallAuton();
  moveForwardWalk(-10, 100, -86, 1000);

}

void centerGoalAuton(){
  createBallCountTask();
  setIntakeSpeed(-100);
  indexer.spin(fwd, 100, pct);
  moveForwardWalk(32.5, 90, 0, 1000);
  brakeIntake();
  indexer.stop(); 
  createIntakeOnTask();
  rotatePID(90, 80);
  task sto = task(stopIntakeFunction);
  moveForwardWalk(36, 100, 90, 1000);
  outtake2BallAuton();
  task::stop(sto);
  indexer.stop();
  intake.spin(reverse, 100, pct);
  task::sleep(75);
  ballC = 0;
  moveForwardWalk(-32, 90, 90, 10);
  rotatePID(-45, 90);
  createIntakeOnTask();
  moveForwardWalk(24, 90, -45, 1000);
  task::sleep(75);
  stopIntakeOn();
  moveForwardWalk(-22, 100, -45, 0.6);
  task::sleep(75);
  rotatePID(-90, 80);
  intakeRight.spin(fwd, -100, pct);
  intakeLeft.spin(fwd, 100, pct);
  moveForwardWalk(40, 100, -90, 1000);
  outtake2BallAuton();
  brakeIntake();
  ballC = 0;
  moveForwardWalk(-4, 100, -90, 1000);
  //createIntakeOnTask();
  rotatePID(-168, 80);
  //task stop = task(stopIntakeFunction);
  moveForwardWalk(82, 100, -163, 1000);
  //outtake1BallAuton();
  //task::stop(sto);
  //indexer.stop();
  //moveForwardWalk(-24, 100, -163, 1000);

}

void skills(){
  createBallCountTask();
  setIntakeSpeed(-100);
  indexer.spin(fwd, 100, pct);
  task::sleep(500);
  brakeIntake();
  indexer.stop(); 
  createIntakeOnTask();
  moveForwardWalk(12, 90, 0, 1000);
  task::sleep(50);
  rotatePID(45, 80);
  moveForwardWalk(26, 90, 45, 1000);
  task::sleep(50);
  rotatePID(90, 80);
  moveForwardWalk(24, 90, 90, 1000);
  while(ballC < 2){
    task::sleep(10);
  }
  stopIntakeOn();
  task::sleep(50);
  outtake1BallAuton(); // 1
  createFlushOutIntake();
  moveForwardWalk(-16, 50, 90, 10);
  task::sleep(50);
  rotatePID(-45, 80);
  stopFlushOutIntake();
  ballC = 0;
  createIntakeOnTask();
  moveForwardWalk(46.5, 90, -45, 1000);
  task::sleep(50);
  rotatePID(46, 80);
  moveForwardWalk(8, 90, 45, 1000);
  while(ballC < 2){
  task::sleep(10);
  }
  stopIntakeOn();
  task::sleep(50);
  outtake1BallAuton(); //2
  task::sleep(100);
  createFlushOutIntake();
  moveForwardWalk(-19, 50, 45, 10);
  task::sleep(50);
  rotatePID(-45, 80);
  ballC = 0;
  stopFlushOutIntake();
  createIntakeOnTask();
  moveForwardWalk(48, 90, -45, 1000);
  task::sleep(50);
  moveForwardWalk(-14, 90, -45, 10);
  task::sleep(50);
  rotatePID(0, 80);
  moveForwardWalk(38, 70, 0, 1000);
  while(ballC < 2){
  task::sleep(10);
  }
  stopIntakeOn();
  task::sleep(100);
  outtake1BallAuton(); //3
  createFlushOutIntake();
  moveForwardWalk(-47, 50, 0, 10);
  task::sleep(50);
  rotatePID(-135, 80);
  stopFlushOutIntake();
  ballC = 0;
  createIntakeOnTask();
  moveForwardWalk(25.5, 90, -135, 1000);
  task::sleep(50);
  rotatePID(-44, 80);
  moveForwardWalk(36, 70, -45, 1000);
  while(ballC < 2){
  task::sleep(10);
  }
  stopIntakeOn();
  task::sleep(50);
  outtake1BallAuton(); //4
  createFlushOutIntake();
  moveForwardWalk(-7, 85, -45, 0.6);
  task::sleep(50);
  rotatePID(-135, 80);
  stopFlushOutIntake();
  ballC = 0;
  createIntakeOnTask();
  moveForwardWalk(43, 85, -135, 1000);
  task::sleep(50);
  rotatePID(-90, 80);
  moveForwardWalk(26, 85, -90, 1000);
  while(ballC < 2){
  task::sleep(10);
  }
  stopIntakeOn();
  task::sleep(200);
  outtake1BallAuton(); //5
  createFlushOutIntake();
  moveForwardWalk(-14, 85, -90, 2);
  task::sleep(50);
  rotatePID(-225, 80);
  stopFlushOutIntake();
  ballC = 0;
  createIntakeOnTask();
  moveForwardWalk(47, 85, -225, 1000);
  task::sleep(50);
  rotatePID(-135, 80);
  moveForwardWalk(7, 60, -135, 1000);
  while(ballC < 2){
  task::sleep(10);
  }
  stopIntakeOn();
  task::sleep(50);
  outtake1BallAuton();
  createFlushOutIntake();
  moveForwardWalk(-19, 50, -135, 10);
  task::sleep(50);
  rotatePID(-225, 80);
  stopFlushOutIntake();
  ballC = 0;
  createIntakeOnTask();
  moveForwardWalk(70, 85, -225, 1000);
  moveForwardWalk(-29, 80, -225, 10);
  task::sleep(50);
  rotatePID(-180, 80);
  moveForwardWalk(39, 85, -180, 1000);
  while(ballC < 2){
  task::sleep(10);
  }
  stopIntakeOn();
  task::sleep(50);
  outtake1BallAuton();
  createFlushOutIntake();
  moveForwardWalk(-50, 50, -180, 10); 
  task::sleep(50);
  rotatePID(-315, 80); //-225
  stopFlushOutIntake();
  ballC = 0;
  createIntakeOnTask();
  moveForwardWalk(20, 85, -315, 1000); //-225
  task::sleep(50);
  rotatePID(-405, 80); //-45
  stopIntakeOn();
  setIntakeSpeed(-100);
  moveForwardWalk(50, 40, -405, 0.001);
  task::sleep(500);
  outtake1BallAutonCenter();
  task::sleep(250);
  moveForwardWalk(-24, 40, 0, 0.1);


  moveForwardWalk(-4, 90, 0, 0.1);
  moveForwardWalk(7, 40, 0, 0.1); //-405
  moveForwardWalk(-4, 90, -405, 0.1); //-405
  rotatePID(-315, 80); // -315
  moveForwardWalk(-4, 50, -315, 0.1); // -315
  rotatePID(45, 80); // -360
  moveForwardWalk(15, 50, -360, 0.1); //-360

}*/


void testAuton(){ 
//moveForward(40, 48);
rotatePID(60, 90, 0.49, 0.1, 0.01);
task::sleep(250);
moveForwardWalk(20, 90, 0, 0.6, 5);
//moveForwardWalk(48, 90, 90, 10, 5);
//forwardPID(2020,0,3,20);
}

void homeRowAuton(){
  //180: 0.445, 0 0.7
  //90: 
  //45: 
  //moveForward(50,5);
  moveForwardWalk(5,75,0,0.6,5);
  wait(0.5, sec);
  arcPID(177, 90,1.7 , 0.25, 0.01, 1, 0);
  wait(0.5, sec);
  createTilterUnfoldTask();
  moveForwardWalk(-150,90,0,0.6,5);
  stopUnfold();
  wait(0.5, sec);
  dumpRing();
  moveForwardWalk(15,90,0,0.6,5);
  stopUnfold();
}

//rotate PID values (0.8, 0.1, 0.01)
