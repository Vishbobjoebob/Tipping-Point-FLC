#ifndef OPCONTROL_H
#define OPCONTROL_H

void joyStickControl(void);

void intakeControl(void) ;

int armControl(void) ;

int conveyorControl(void) ;

void pistonControl(void) ;
//void armMacro(void) ;

int tilterControl(void) ;

int manualControl(void) ;

void resetEncoders(void) ;
template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

extern double front_left, front_right, back_left, back_right;

#endif 