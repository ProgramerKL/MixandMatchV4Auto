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
inertial BrainInertial = inertial();
controller Controller = controller();
motor LeftMotor = motor(PORT9, false);
motor RightMotor = motor(PORT3, true);
// motor motor2 = motor(PORT2, true);
touchled touchled5 = touchled(PORT5);
motor BackArmMotor1 = motor(PORT2, true);
motor BackArmMotor2 = motor(PORT8, false);
motor FrontArmMotor1 = motor(PORT4, true);
motor FrontArmMotor2 = motor(PORT10, false);
pneumatic P1 = pneumatic(PORT11);
// from front view left
pneumatic P2 = pneumatic(PORT12);
pneumatic P3 = pneumatic(PORT6);
float kp = 1.5;
int velocity = 60;
float Rmotorspeed, Lmotorspeed, Rightstick, Leftstick, deadband = 10;
bool crawlmodestate;
bool isfrontclawrightopen;
bool isfrontclawleftopen;
bool isfingeropen;
bool isbackarmup;
bool isclawsensorsdetecting;
bool ispusherextended;
int distancedetection = 80;
int backarmstatecounter;
bool isstandoffgoalstacking;
bool isfrontclawup;
bool startingpingrabstate;
bool touchledstate;
bool clawsensorstate = false;
int start = 0;
int now = 0;
int timerstart = 0;
int crawlspeed = 35;
void lowerfromstandoffgoal();
void splitdrivewithcrawlmode();
event eventfrontclawgodownforbeam = event();
thread drivetrainthread = thread();
event eventraisebackarmtogroundystack = event();
event eventraisebackarmtoministandoffstack = event();
event eventraisebackarmtobigstandoffystack = event();
event eventraisebackarmtonest = event();
event eventfrontclawgodown = event();
thread raisebackarmthread = thread();
// ==============================================================================
// HELPER FUNCTIONS
// ==============================================================================

void touchledcolourselection() {
  if (touchledstate) {
    touchled5.setColor(green);
  } else {
    touchled5.setColor(red);
  }
}

void deployguide() {
  // P3.extend(cylinder1);
  P3.extend(cylinder1);
}

void retractguide() {
  // P3.retract(cylinder1);
  P3.retract(cylinder1);
}

void pumpon() {
  P1.pumpOn();
  P2.pumpOn();
  P3.pumpOn();
}
void pumpoff() {
  P1.pumpOff();
  P2.pumpOff();
  P3.pumpOff();
}
void extendclawbalancer() { P2.extend(cylinder1); }
void retractclawbalancer() { P2.retract(cylinder1); }

void extendpusher() { P1.extend(cylinder2); }
void retractpusher() { P1.retract(cylinder2); }

void frontclawleftclose() {
  P1.retract(cylinder1);
  isfrontclawleftopen = false;
}

void frontclawleftopen() {
  P1.extend(cylinder1);
  isfrontclawleftopen = true;
}

void frontclawrightclose() {
  P1.retract(cylinder2);
  isfrontclawrightopen = false;
}
void frontclawrightopen() {
  P1.extend(cylinder2);
  isfrontclawrightopen = true;
}

void frontclawopen() {
  frontclawleftopen();
  frontclawrightopen();
}

void closefinger() {
  P2.extend(cylinder2);
  isfingeropen = false;
}

void openfinger() {
  P2.retract(cylinder2);
  isfingeropen = true;
}

void spinbackarmup() {
  BackArmMotor1.setStopping(hold);
  BackArmMotor2.setStopping(hold);
  BackArmMotor1.spin(forward);
  BackArmMotor2.spin(forward);
}

void spinbackarmdown() {
  BackArmMotor1.setStopping(coast);
  BackArmMotor2.setStopping(coast);
  BackArmMotor1.spin(reverse);
  BackArmMotor2.spin(reverse);
}

void backarmstop() {
  BackArmMotor1.stop();
  BackArmMotor2.stop();
}

void movefrontclawup() {
  isfrontclawup = true;
  FrontArmMotor1.spin(forward);
  FrontArmMotor2.spin(forward);
}

void movefrontclawdown() {
  isfrontclawup = false;
  FrontArmMotor1.spin(reverse);
  FrontArmMotor2.spin(reverse);
}

void frontclawstop() {
  FrontArmMotor1.stop();
  FrontArmMotor2.stop();
}

void fingercontrol() {
  // retractguide();
  if (isfingeropen) {
    spinbackarmdown();
    drivetrainthread.interrupt();
    LeftMotor.setVelocity(100, percent);
    RightMotor.setVelocity(100, percent);
    RightMotor.spin(reverse);
    LeftMotor.spin(reverse);
    wait(100, msec);
    closefinger();
    wait(0.3, seconds);
    drivetrainthread = thread(splitdrivewithcrawlmode);
    // eventraisebackarmtonest.broadcast();
    // wait(0.2, seconds);
    spinbackarmup();
    while (BackArmMotor1.position(degrees) < 65) {
      wait(20, msec);
    }
    BackArmMotor1.setStopping(hold);
    BackArmMotor2.setStopping(hold);
    backarmstop();
  } else {
    BackArmMotor1.setStopping(coast);
    BackArmMotor2.setStopping(coast);
    BackArmMotor1.setVelocity(50, percent);
    BackArmMotor2.setVelocity(50, percent);
    spinbackarmdown();
    wait(0.45, seconds);
    backarmstop();
    BackArmMotor1.setVelocity(100, percent);
    BackArmMotor2.setVelocity(100, percent);
    openfinger();
    if (BackArmMotor1.position(degrees) > 250) {
      spinbackarmup();
      wait(0.45, seconds);
      backarmstop();
      drivetrainthread.interrupt();
      LeftMotor.setVelocity(100, percent);
      RightMotor.setVelocity(100, percent);
      RightMotor.spin(forward);
      LeftMotor.spin(forward);
      wait(0.4, seconds);
      drivetrainthread = thread(splitdrivewithcrawlmode);
    }
    spinbackarmdown();
    while (BackArmMotor1.position(degrees) > 20) {
      wait(20, msec);
    }
    BackArmMotor1.setStopping(coast);
    BackArmMotor2.setStopping(coast);
    backarmstop();
    backarmstatecounter = 0;
  }
}

void frontarmgodownforbeam() {
  spinbackarmdown();
  wait(0.5, seconds);
  spinbackarmup();
  while (BackArmMotor1.position(degrees) < 75) {
    wait(20, msec);
  }
  BackArmMotor1.setStopping(hold);
  BackArmMotor2.setStopping(hold);
  backarmstop();
}

void dumppinsontobeam() {
  closefinger();
  eventfrontclawgodownforbeam.broadcast();
  movefrontclawup();
  while (FrontArmMotor1.position(degrees) < 500) { // 625
    wait(20, msec);
  }
  FrontArmMotor1.setStopping(brake);
  FrontArmMotor2.setStopping(brake);
  FrontArmMotor1.setVelocity(60, percent);
  FrontArmMotor2.setVelocity(60, percent);
  while (FrontArmMotor1.position(degrees) < 720) {
    wait(20, msec);
  }
  frontclawopen();
  frontclawstop();
  isclawsensorsdetecting = false;
  FrontArmMotor1.setVelocity(100, percent);
  FrontArmMotor2.setVelocity(100, percent);
  wait(0.2, seconds);
  eventfrontclawgodown.broadcast();
  extendclawbalancer();
  wait(0.2, seconds);
  // spinbackarmdown();
  // wait(0.2, seconds);
  while (FrontArmMotor2.position(degrees) > 5) {
    wait(20, msec);
  }
  BackArmMotor1.setStopping(hold);
  BackArmMotor2.setStopping(hold);
  backarmstop();
  FrontArmMotor1.setStopping(coast);
  FrontArmMotor2.setStopping(coast);
  frontclawstop();
  eventraisebackarmtonest.broadcast();
  isclawsensorsdetecting = true;
  retractclawbalancer();
}

// ==============================================================================
// ARM FUNCTIONS
// ==============================================================================

void lowerbackarmfromstandoff() {
  spinbackarmdown();
  while (BackArmMotor1.position(degrees) > 1080) {
    wait(20, msec);
  }
  openfinger();
  while (BackArmMotor1.position(degrees) > 10) {
    wait(20, msec);
  }
  backarmstop();
}

void raisebackarmtogroundystack() {
  if (BackArmMotor1.position(degrees) > 300) {
    spinbackarmdown();
    while (BackArmMotor1.position(degrees) > 300) {
      wait(20, msec);
    }
    BackArmMotor1.setStopping(hold);
    BackArmMotor2.setStopping(hold);
    backarmstop();
    deployguide();
  } else {
    spinbackarmup();
    while (BackArmMotor1.position(degrees) < 300) {
      wait(20, msec);
    }
    backarmstop();
    deployguide();
  }
}

void raisebackarmtoministandoffstack() {
  if (BackArmMotor1.position(degrees) > 400) {
    spinbackarmdown();
    while (BackArmMotor1.position(degrees) > 400) {
      wait(20, msec);
    }
    BackArmMotor1.setStopping(hold);
    BackArmMotor2.setStopping(hold);
    backarmstop();
  } else {
    spinbackarmup();
    while (BackArmMotor1.position(degrees) < 400) {
      wait(20, msec);
    }
    backarmstop();
  }
}

void raisebackarmtobigstandoffystack() {
  if (BackArmMotor1.position(degrees) > 500) {
    spinbackarmdown();
    while (BackArmMotor1.position(degrees) > 750) {
      wait(20, msec);
    }
    BackArmMotor1.setStopping(hold);
    BackArmMotor2.setStopping(hold);
    wait(0.3, seconds);
    backarmstop();
  } else {
    spinbackarmup();
    while (BackArmMotor1.position(degrees) < 500) {
      wait(20, msec);
    }
    wait(0.3, seconds);
    backarmstop();
  }
}

void backarmcontrol() {
  backarmstatecounter++;
  raisebackarmthread.interrupt();
  if (backarmstatecounter % 3 == 1) { // raise to 91 height
    closefinger();
    raisebackarmthread = thread(raisebackarmtogroundystack);
    touchled5.setColor(green);
  } else if (backarmstatecounter % 3 == 2) { // raise to 110 height
    raisebackarmthread = thread(raisebackarmtoministandoffstack);
    touchled5.setColor(yellow);
    // retractguide();
    closefinger();
  } else if (backarmstatecounter % 3 == 0) { // raise to 121 height
    raisebackarmthread = thread(raisebackarmtobigstandoffystack);
    touchled5.setColor(red);
  }
}

// ==============================================================================
// DRIVER FUNCTIONS
// ==============================================================================
// ==============================================================================
// MAIN PROGRAM
// ==============================================================================

void inital() {
  isbackarmup = false;
  isclawsensorsdetecting = true;
  isfrontclawleftopen = true;
  isfrontclawrightopen = true;
  isfingeropen = true;
  openfinger();
  frontclawopen();
  BackArmMotor1.setStopping(hold);
  BackArmMotor2.setStopping(hold);
  LeftMotor.setMaxTorque(100, percent);
  RightMotor.setMaxTorque(100, percent);
  FrontArmMotor1.setVelocity(100, percent);
  FrontArmMotor2.setVelocity(100, percent);
  FrontArmMotor1.setMaxTorque(100, percent);
  FrontArmMotor2.setMaxTorque(100, percent);
  BackArmMotor1.setVelocity(100, percent);
  BackArmMotor2.setVelocity(100, percent);
  BackArmMotor1.setMaxTorque(100, percent);
  BackArmMotor2.setMaxTorque(100, percent);
  FrontArmMotor1.setPosition(0, degrees);
  FrontArmMotor2.setPosition(0, degrees);
  wait(0.2, seconds);
  retractguide();
}

void stackpins() {
  FrontArmMotor2.setStopping(hold);
  FrontArmMotor1.setStopping(hold);
  LeftMotor.setStopping(brake);
  RightMotor.setStopping(brake);
  if (isfrontclawup) {
    extendclawbalancer();
    movefrontclawdown();
    wait(0.1, seconds);
    while (FrontArmMotor1.position(degrees) > 170) {
      wait(20, msec);
    }
    frontclawopen();
    while (FrontArmMotor1.position(degrees) > 5) {
      wait(20, msec);
    }
    frontclawstop();
    wait(1, seconds);
    isclawsensorsdetecting = true;
  } else {
    extendclawbalancer();
    isclawsensorsdetecting = false;
    movefrontclawup();
    while (FrontArmMotor1.position(degrees) < 215) {
      wait(20, msec);
    }
    frontclawstop();
  }
  isclawsensorsdetecting = true;
}

void raisebackarmtonest() {
  spinbackarmup();
  BackArmMotor1.setVelocity(50, percent);
  BackArmMotor2.setVelocity(50, percent);
  // while (motor1.position(degrees) < 145) {
  //   wait(20, msec);
  // }
  backarmstop();
  BackArmMotor1.setVelocity(100, percent);
  BackArmMotor2.setVelocity(100, percent);
}

void grabstartingpin() {
  isclawsensorsdetecting = false;
  startingpingrabstate = !startingpingrabstate;
  if (startingpingrabstate) {
    frontclawopen();
    FrontArmMotor1.setStopping(hold);
    FrontArmMotor2.setStopping(hold);
    movefrontclawup();
    while (FrontArmMotor1.position(degrees) < 160) {
      wait(20, msec);
    }
    frontclawstop();
  } else {
    // extendpusher();
    // wait(0.5, seconds);
    // retractpusher();
    frontclawleftclose();
    frontclawrightclose();
    movefrontclawup();
    while (FrontArmMotor1.position(degrees) < 160) {
      wait(20, msec);
    }
    drivetrainthread.interrupt();
    LeftMotor.setVelocity(100, percent);
    RightMotor.setVelocity(100, percent);
    LeftMotor.spin(reverse);
    RightMotor.spin(reverse);
    wait(0.25, seconds);
    frontclawstop();
    drivetrainthread = thread(splitdrivewithcrawlmode);
    // movefrontclawdown();
    // while (motor6.position(degrees) > 175) {
    //   wait(20, msec);
    // }
    // isfrontclawup = true;
    // frontclawstop();
    // extendpusher();
    // wait(0.25, seconds);
    // retractpusher();
    isclawsensorsdetecting = true;
  }
}

void grabpins() {
  clawsensorstate = !clawsensorstate;
  if (clawsensorstate) {
    frontclawleftclose();
    frontclawrightclose();
  } else {
    frontclawleftopen();
    frontclawrightopen();
  }
}

void stackpinsontostandoff() {
  isstandoffgoalstacking = !isstandoffgoalstacking;
  isclawsensorsdetecting = false;
  if (isstandoffgoalstacking) {
    movefrontclawup();
    LeftMotor.setStopping(brake);
    RightMotor.setStopping(brake);
    extendclawbalancer();
    while (FrontArmMotor1.position(degrees) < 345) { // 300
      wait(20, msec);
    }
    FrontArmMotor2.setStopping(hold);
    FrontArmMotor1.setStopping(hold);
    frontclawstop();
    wait(0.1, seconds);
    // extendpusher();
  } else {
    // LeftMotor.setStopping(hold);
    // RightMotor.setStopping(hold);
    // LeftMotor.stop();
    // RightMotor.stop();
    FrontArmMotor1.setVelocity(60, percent);
    FrontArmMotor2.setVelocity(60, percent);
    movefrontclawdown();
    // drivetrainthread. = thread(splitdrivewithcrawlmode);
    while (FrontArmMotor1.position(degrees) > 295) {
      wait(20, msec);
    }
    frontclawopen();
    FrontArmMotor1.setVelocity(-80, percent);
    FrontArmMotor2.setVelocity(-80, percent);
    while (FrontArmMotor1.position(degrees) > 5) {
      wait(20, msec);
    }
    FrontArmMotor1.setStopping(hold);
    FrontArmMotor2.setStopping(hold);
    frontclawstop();
    LeftMotor.setStopping(coast);
    RightMotor.setStopping(coast);
    FrontArmMotor1.setVelocity(100, percent);
    FrontArmMotor2.setVelocity(100, percent);
    // retractclawbalancer();
  }
  LeftMotor.setStopping(brake);
  RightMotor.setStopping(brake);
  wait(0.5, seconds);
  isclawsensorsdetecting = true;
}

void stackpinsincornergoal() {
  isclawsensorsdetecting = false;
  movefrontclawup();
  while (FrontArmMotor1.position(degrees) < 115) {
    wait(20, msec);
  }
  FrontArmMotor2.setStopping(hold);
  FrontArmMotor1.setStopping(hold);
  frontclawstop();
}

void pushercontrol() {
  ispusherextended = !ispusherextended;
  if (ispusherextended) {
    retractpusher();
  } else {
    extendpusher();
  }
}

void buttonlogic() {
  if (Controller.ButtonLUp.pressing()) {
    grabpins();
  }
  if (Controller.ButtonRDown.pressing()) {
    fingercontrol();
  }
  if (Controller.ButtonFUp.pressing()) {
    backarmcontrol();
  }
  if (Controller.ButtonR3.pressing()) {
    extendclawbalancer();
  }
  if (Controller.ButtonEUp.pressing()) {
    dumppinsontobeam();
  }
  if (Controller.ButtonEDown.pressing()) {
    stackpinsontostandoff();
  }
  if (Controller.ButtonLDown.pressing()) {
    stackpins();
  }
  if (Controller.ButtonFDown.pressing()) {
    wait(0.5, seconds);
    isclawsensorsdetecting = false;
    if (Controller.ButtonFDown.pressing()) {
      stackpinsincornergoal();
    } else {
      grabstartingpin();
    }
  }
}

void disconnectionfunc() {
  if (FrontArmMotor1.installed() == false) {
    Brain.playSound(siren);
    Brain.Screen.print("frontarmotor is disconnected port1");
  }
  if (FrontArmMotor2.installed() == false) {
    Brain.playSound(siren);
    Brain.Screen.print("motor2 is disconnected port2");
  }
  if (BackArmMotor1.installed() == false) {
    Brain.playSound(siren);
    Brain.Screen.print("motor6 is disconnected port6");
  }
  if (BackArmMotor2.installed() == false) {
    Brain.playSound(siren);
    Brain.Screen.print("motor7 is disconnected port7");
  }
  if (LeftMotor.installed() == false) {
    Brain.playSound(siren);
    Brain.Screen.print("leftmotor is disconnected port8");
  }
  if (RightMotor.installed() == false) {
    Brain.playSound(siren);
    Brain.Screen.print("rightmotor is disconnected port2");
  }
  if (P1.installed() == false) {
    Brain.playSound(siren);
    Brain.Screen.print("P1 disconnected port4");
  }
  if (P2.installed() == false) {
    Brain.playSound(siren);
    Brain.Screen.print("P1 disconnected port10");
  }
}

bool c1(int velocity, float distance) {
  if (velocity > 0) {
    if (LeftMotor.position(degrees) < distance) {
      return true;
    } else {
      return false;
    }
  } else {
    if (LeftMotor.position(degrees) > distance * -1) {
      return true;
    } else {
      return false;
    }
  }
}

bool c2(float timeout) {
  float timerduration;
  if (timeout == 0) {
    return true;
  } else {
    now = Brain.Timer.value();
    timerduration = now - start;
    if (timerduration > timeout) {
      return false;
    } else {
      return true;
    }
  }
}

bool c3(bool stalldetection) {
  if (stalldetection) {
    if (LeftMotor.current(amp) > 0.87 && LeftMotor.velocity(percent) == 0) {
      // checks whether its on a wall or stuck (same logic as the is
      // roller stuck function)
      Brain.playSound(tada);
      return false; // stuck or on wall
    } else {
      return true; // not stuck or on wall
    }
  } else {
    return true; // not stuck or on wall
  }
}

void steering(float steering, float velocity) {
  LeftMotor.spin(forward);
  RightMotor.spin(forward);
  if (steering < 0) {
    LeftMotor.setVelocity(velocity, percent);
    RightMotor.setVelocity((steering / 50 + 1) * velocity, percent);
  } else if (steering > 0) {
    LeftMotor.setVelocity((steering / 50 + 1) * velocity, percent);
    RightMotor.setVelocity(velocity, percent);
  }
}

void movechassis(int velocity, float output) {
  LeftMotor.setVelocity(velocity + output, percent);
  RightMotor.setVelocity(velocity - output, percent);
  LeftMotor.spin(forward);
  RightMotor.spin(forward);
  wait(20, msec);
}

void clockwise() {
  LeftMotor.spin(forward);
  RightMotor.spin(reverse);
}

void counterclockwise() {
  LeftMotor.spin(reverse);
  RightMotor.spin(forward);

  // Also control PTO motors if in drive mode
}

void Pid(float distance, float heading, int velocity, float kp, float timeout,
         bool stalldetection) {
  start = Brain.Timer.value();
  float error;
  float output;
  LeftMotor.setPosition(0, degrees);
  RightMotor.setPosition(0, degrees);
  LeftMotor.setStopping(brake);
  RightMotor.setStopping(brake);
  // printf("kp is: %.2f\n", kp);
  while (c1(velocity, distance) && c2(timeout) && c3(stalldetection)) {
    error = heading - BrainInertial.rotation(degrees);
    // printf("Inertial %f\n", BrainInertial.rotation(degrees));
    output = kp * error;
    movechassis(velocity, output);

    // printf("Error: %f\n", error);
    wait(20, msec);
  }

  LeftMotor.setStopping(coast);
  RightMotor.setStopping(coast);
  LeftMotor.stop();
  RightMotor.stop();
  wait(20, msec);
}

void Preciseturn(float heading, int velocity, float momentum, float timeout,
                 bool stalldetection) {
  timerstart = Brain.Timer.value();
  if (heading > BrainInertial.rotation()) {
    // this means that is is turning clockwise
    while (heading - momentum > BrainInertial.rotation(degrees) &&
           c2(timeout) && c3(stalldetection)) {
      LeftMotor.setVelocity(velocity, percent);
      RightMotor.setVelocity(velocity, percent);

      clockwise();
      wait(20, msec);
    }
  } else {
    while (heading + momentum < BrainInertial.rotation() && c2(timeout) &&
           c3(stalldetection)) {
      LeftMotor.setVelocity(velocity, percent);
      RightMotor.setVelocity(velocity, percent);

      // Also set velocity for PTO motors if in drive mode

      counterclockwise();
      wait(20, msec);
    }
  }
  LeftMotor.stop();
  RightMotor.stop();

  // Also stop PTO motors if in drive mode
  wait(0.1, seconds); // for momentum
}

void part1() {
  frontclawleftclose();
  frontclawrightclose();
  Brain.resetTimer();
  wait(0.5, seconds);
  stackpins();
  Pid(350, -16, velocity, kp, 0, false);
  stackpins();
  Pid(50, -16, velocity, kp, 0, false);
  frontclawleftclose();
  frontclawrightclose();
  stackpinsontostandoff();
  Pid(325, 0, velocity, kp, 0, false);
  Preciseturn(-78, 35, 10, 0, false);
  Pid(350, -86, velocity, kp, 2, false);
  stackpinsontostandoff();
  Pid(350, -90, -velocity, kp, 2, false);
}

void part2() {
  Pid(400, -45, velocity, kp, 0, false);
  frontclawrightclose();
  Preciseturn(-85, 30, 10, 0, false);
  wait(0.2, seconds);
  Pid(290, -85, velocity, kp, 0, false);
  frontclawleftclose();
  stackpins();
  Pid(165, -90, velocity, kp, 0, false);
  Pid(215, -110, velocity, kp, 0, false);
  wait(0.3, seconds);
  Preciseturn(-90, 30, 10, 0, false);
  Pid(35, -90, velocity, kp, 0, false);
  Pid(25, -90, -velocity, kp, 0, false);
  stackpins();
  // Pid(40, -110, -velocity, kp, 1, false);
  // Pid(40, -110, velocity, kp, 1, false);
  frontclawleftclose();
  frontclawrightclose();
  Pid(150, -110, -velocity, kp, 0, false);
  Pid(500, -180, -velocity, kp, 0, false);
}

void autofunc() {
  part1();
  wait(5, seconds);
  part2();
  // part1();
  // part2();
  printf("timer is %.2f\n", Brain.Timer.value());
}

int main() {
  pumpoff();
  BrainInertial.calibrate();
  while (BrainInertial.isCalibrating()) {
    wait(20, msec);
  }
  inital();
  pumpon();
  eventraisebackarmtonest = event(raisebackarmtonest);
  Controller.ButtonR3.pressed(part1);
  Controller.ButtonL3.pressed(part2);
  Controller.ButtonFUp.pressed(autofunc);
  touchled5.pressed(autofunc);
  Brain.playSound(tada);
  eventfrontclawgodownforbeam = event(frontarmgodownforbeam);
  eventfrontclawgodown = event(movefrontclawdown);
  // Run main drive control loop
  while (true) {
    touchledcolourselection();
    Brain.Screen.setFont(mono15);
    if (Brain.buttonLeft.pressing()) {
      Brain.Screen.clearScreen();
      if (crawlspeed < 95) {
        Brain.playSound(doorClose);
        crawlspeed = crawlspeed + 5;
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("crawlspeed, %d\n", crawlspeed);
        Brain.Screen.setCursor(1, 2);
        wait(0.2, seconds);
      } else {
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("crawlspeed too high");
        Brain.Screen.setCursor(1, 2);
        wait(0.2, seconds);
      }
    } else if (Brain.buttonRight.pressing()) {
      Brain.Screen.clearScreen();
      if (crawlspeed > 5) {
        Brain.playSound(ratchet);
        crawlspeed = crawlspeed - 5;
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("crawlspeed, %d\n", crawlspeed);
        Brain.Screen.setCursor(1, 2);
        wait(0.2, seconds);
      } else {
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("crawlspeed too low");
        wait(0.2, seconds);
      }
    }
    disconnectionfunc();
    // printf("positioning of A is %d\n", Controller.AxisA.position());
    // printf("positioning of B is %d\n", Controller.AxisB.position());
    if (Controller.ButtonRUp.pressing()) {
      touchledstate = false;
      crawlmodestate = true;
      Brain.playSound(siren);
      wait(0.15, seconds);
    } else {
      crawlmodestate = false;
    }

    // if (!Controller.ButtonLUp.pressing()) {
    //   if (isclawsensorsdetecting) {
    //     if (distancesensorleftclaw.objectDistance(mm) < distancedetection) {
    //       if (isfrontclawleftopen) {
    //         Brain.playSound(doorClose);
    //         frontclawleftclose();
    //       }
    //     }
    //     if (distancesensorightclaw.objectDistance(mm) < distancedetection) {
    //       if (isfrontclawrightopen) {
    //         Brain.playSound(doorClose);
    //         frontclawrightclose();
    //       }
    //     }
    //   }
    // }
    wait(20, msec);
  }
}
