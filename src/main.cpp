/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       kyleliu                                                   */
/*    Created:      11/22/2025, 8:51:49 PM                                    */
/*    Description:  IQ2 project                                               */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"

using namespace vex;

// A global instance of vex::brain used for printing to the IQ2 brain screen
vex::brain Brain;

// VEXcode device constructors
controller Controller = controller();
motor LeftMotor = motor(PORT5, false);
motor RightMotor = motor(PORT12, true);
motor Motor3 = motor(PORT3, false);
motor motor2 = motor(PORT2, false);
motor motor9 = motor(PORT9, false);
motor motor8 = motor(PORT8, false);
pneumatic P1 = pneumatic(PORT1);
// from front view left
pneumatic P2 = pneumatic(PORT7);
pneumatic P3 = pneumatic(PORT11);
distance distancesensorfourbar = distance(PORT6);
bool P1C1extend;
bool P1C2extend;
bool P2C1extend;
bool P2C2extend;
bool P3C1extend;
bool P3C2extend;

float Rmotorspeed, Lmotorspeed, Rightstick, Leftstick, deadband;
thread drivetrainthread = thread();
// ==============================================================================
// HELPER FUNCTIONS
// ==============================================================================
void p1c1extend() {
  P1.extend(cylinder1);
  P1C1extend = true;
}

void p1c1retract() {
  P1.retract(cylinder1);
  P1C1extend = false;
}

void p1c2extend() {
  P1.extend(cylinder2);
  P1C2extend = true;
}

void p1c2retract() {
  P1.retract(cylinder2);
  P1C2extend = false;
}

void p2c1extend() {
  P2.extend(cylinder1);
  P2C1extend = true;
}

void p2c1retract() {
  P2.extend(cylinder1);
  P2C1extend = true;
}

void p2c2extend() {
  P2.extend(cylinder2);
  P2C2extend = true;
}
void p2c2retract() {
  P2.retract(cylinder2);
  P2C2extend = false;
}

void p3c1extend() {
  P3.extend(cylinder1);
  P3C1extend = true;
}

void p3c1retract() {
  P3.retract(cylinder1);
  P3C1extend = false;
}

void p3c2retract() {
  P3.retract(cylinder2);
  P3C1extend = false;
}

void p3c2extend() {
  P3.extend(cylinder2);
  P3C1extend = false;
}

void pumpson() {
  P1.pumpOn();
  P2.pumpOn();
  P3.pumpOn();
}
// ==============================================================================
// BASIC FUNCTIONS
// ==============================================================================

void leftClawOpen() { p1c1retract(); }
void leftClawClose() { p1c1extend(); }
void rightClawOpen() { p2c2retract(); }
void rightClawclose() { p2c2extend(); }
void fingerClose() { p2c1retract(); }
void fingerOpen() { p2c1extend(); }
void activeateFourBarBrake() { p1c2extend(); }
void deactiveateFourBarBrake() { p1c2retract(); }
void guidedeploy() { p3c2extend(); }
void guideretract() { p3c2retract(); }
// ==============================================================================
// DRIVER FUNCTIONS
// ==============================================================================

void driveControl() {
  deadband = 10.0;
  while (true) {
    LeftMotor.spin(forward);
    RightMotor.spin(forward);
    LeftMotor.setStopping(coast);
    RightMotor.setStopping(coast);
    Leftstick = Controller.AxisA.position();
    Rightstick = Controller.AxisC.position();
    Lmotorspeed = Rightstick + Leftstick;
    Rmotorspeed = Leftstick - Rightstick;
    RightMotor.setVelocity(Rmotorspeed, percent);
    LeftMotor.setVelocity(Lmotorspeed, percent);

    wait(20, msec);
  }
}
// ==============================================================================
// MAIN PROGRAM
// ==============================================================================

void disconnectionfunc() {
  // Check motors
  if (LeftMotor.installed() == false) {
    Brain.playSound(siren);
    Brain.Screen.print("leftmotor port7 disconnected");
    wait(0.2, seconds);
  }
  if (RightMotor.installed() == false) {
    Brain.playSound(siren);
    Brain.Screen.print("rightmotor port1 disconnected");
    wait(0.2, seconds);
  }
}

int main() {

  Brain.playSound(tada);
  drivetrainthread = thread(driveControl);

  // Run main drive control loop
  while (true) {
    disconnectionfunc();
    printf("positioning of A is %d\n", Controller.AxisA.position());
    printf("positioning of B is %d\n", Controller.AxisB.position());
    printf("\033[2J\n");
    // printf("\n");

    wait(20, msec);
  }
}
