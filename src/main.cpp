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
inertial BrainInertial;
controller Controller = controller();
motor LeftMotor = motor(PORT11, false);
motor RightMotor = motor(PORT5, true);
// motor motor2 = motor(PORT2, true);
touchled touchled5 = touchled(PORT9);
motor BackArmMotor1 = motor(PORT12, false);
motor BackArmMotor2 = motor(PORT6, true);
motor_group BackArmMotorGroup = motor_group(BackArmMotor1, BackArmMotor2);
motor FrontArmMotor1 = motor(PORT10, false);
motor FrontArmMotor2 = motor(PORT2, true);
motor_group frontarmmotorgroup = motor_group(FrontArmMotor1, FrontArmMotor2);
pneumatic P1 = pneumatic(PORT3);
// from front view left
pneumatic P2 = pneumatic(PORT4);
pneumatic P3 = pneumatic(PORT11);
float kp = 0.96;
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
void part2();
void lowerfromstandoffgoal();
void splitdrivewithcrawlmode();
event eventfrontclawgodownforbeam = event();
thread drivetrainthread = thread();
event eventfrontclawgoup = event();
event eventraisebackarmtogroundystack = event();
event eventraisebackarmtoministandoffstack = event();
event eventraisebackarmtobigstandoffystack = event();
event eventraisebackarmtonest = event();
event eventbackarmgodownforbeam = event();
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

void killdrivetrain() {
  drivetrainthread.interrupt();
  // LeftMotor.setStopping(hold);
  // RightMotor.setStopping(hold);
  LeftMotor.stop();
  RightMotor.stop();
  LeftMotor.setVelocity(0, percent);
  RightMotor.setVelocity(0, percent);
}

void pumpoff() {
  P1.pumpOff();
  P2.pumpOff();
  P3.pumpOff();
}

void deployguide() {
  P1.extend(cylinder1);
  // P3.extend(cylinder1);
}

void retractguide() {
  P1.retract(cylinder1);
  // P3.retract(cylinder1);
}

void pumpon() {
  P1.pumpOn();
  P2.pumpOn();
  // P3.pumpOn();
}
void extendclawbalancer() { P2.retract(cylinder1); }
void retractclawbalancer() { P2.extend(cylinder1); }

void extendpusher() { P1.extend(cylinder2); }
void retractpusher() { P1.retract(cylinder2); }

void frontclawleftclose() { P1.retract(cylinder2); }

void frontclawleftopen() {
  P1.extend(cylinder2);
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

void spinbackarmup() { BackArmMotorGroup.spin(forward); }

void spinbackarmdown() { BackArmMotorGroup.spin(reverse); }

void backarmstop() { BackArmMotorGroup.stop(); }

void movefrontclawup() {
  isfrontclawup = true;
  frontarmmotorgroup.spin(forward);
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
  spinbackarmup();
  wait(0.45, seconds);
  BackArmMotorGroup.setStopping(hold);
  backarmstop();
}
void backarmgodownforbeam() {
  spinbackarmdown();
  wait(0.45, seconds);
  backarmstop();
  if (!Controller.ButtonFUp.pressing()) {
    spinbackarmup();
    while (BackArmMotorGroup.position(degrees) < 65) {
      wait(20, msec);
    }
    BackArmMotorGroup.setStopping(hold);
    backarmstop();
  }
}

void dumppinsontobeam() {
  retractclawbalancer();
  closefinger();
  eventbackarmgodownforbeam.broadcast();
  movefrontclawup();
  frontarmmotorgroup.setVelocity(100, percent);
  while (frontarmmotorgroup.position(degrees) < 480) { // 625
    wait(20, msec);
  }
  frontarmmotorgroup.setStopping(brake);
  frontarmmotorgroup.setVelocity(100, percent);
  retractclawbalancer();
  while (frontarmmotorgroup.position(degrees) < 735) { // 740
    wait(20, msec);
  }
  wait(0.2, seconds);
  frontclawopen();
  extendclawbalancer();
  frontclawstop();
  frontarmmotorgroup.setVelocity(100, percent);
  movefrontclawdown();
  while (frontarmmotorgroup.position(degrees) > 5) {
    wait(20, msec);
  }
  extendclawbalancer();
  frontarmmotorgroup.setStopping(brake);
  frontclawleftopen();
  isfrontclawup = true;
  frontclawstop();
}

// ==============================================================================
// ARM FUNCTIONS
// ==============================================================================

void raisebackarmtobigstandoffystack() {
  if (BackArmMotorGroup.position(degrees) > 450) {
    spinbackarmdown();
    while (BackArmMotorGroup.position(degrees) < 455) {
      wait(20, msec);
    }
    BackArmMotorGroup.setStopping(hold);
    wait(0.25, seconds);
    backarmstop();
  } else {
    spinbackarmup();

    while (BackArmMotorGroup.position(degrees) < 555) { // 490
      wait(20, msec);
    }
    backarmstop();
  }
}

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
    // deployguide();
  } else {
    spinbackarmup();
    while (BackArmMotor1.position(degrees) < 300) {
      wait(20, msec);
    }
    backarmstop();
    // deployguide();
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
    while (BackArmMotor1.position(degrees) < 445) {
      wait(20, msec);
    }
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
    frontarmmotorgroup.spinToPosition(130, degrees, true);
    frontclawopen();
    frontarmmotorgroup.spinToPosition(0, degrees, true);
    frontclawstop();
    frontarmmotorgroup.setStopping(coast);
    frontarmmotorgroup.stop();
    // wait(0.25, seconds);
    isclawsensorsdetecting = true;
  } else {
    extendclawbalancer();
    isclawsensorsdetecting = false;
    movefrontclawup();
    while (FrontArmMotor1.position(degrees) < 255) {
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
  while (BackArmMotor1.position(degrees) < 145) {
    wait(20, msec);
  }
  backarmstop();
  BackArmMotor1.setVelocity(100, percent);
  BackArmMotor2.setVelocity(100, percent);
}

void grabstartingpin() {
  isclawsensorsdetecting = false;
  startingpingrabstate = !startingpingrabstate;
  if (startingpingrabstate) {
    printf("in func first\n");
    frontarmmotorgroup.setStopping(hold);
    frontarmmotorgroup.stop();
    deployguide();
    frontclawopen();
    frontarmmotorgroup.setVelocity(100, percent);
    frontarmmotorgroup.setStopping(hold);
    retractclawbalancer();
    frontarmmotorgroup.spinToPosition(102.5, degrees, true);
    frontarmmotorgroup.setStopping(coast);
    frontarmmotorgroup.stop();
  } else {
    printf("in func second\n");
    retractguide();
    extendclawbalancer();
    frontarmmotorgroup.setVelocity(100, percent);
    frontclawleftclose();
    frontarmmotorgroup.spinToPosition(325, degrees, false);
    wait(0.22, seconds);
    frontarmmotorgroup.spinToPosition(210, degrees, false);
    isfrontclawup = true;
    if (BackArmMotorGroup.position(degrees) > 300) {
      lowerbackarmfromstandoff();
    }
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

void raisefrontarmtostandoffheight() {
  LeftMotor.setStopping(hold);
  RightMotor.setStopping(hold);
  isfrontclawup = false;
  FrontArmMotor1.spinToPosition(3, degrees, false);
  FrontArmMotor2.spinToPosition(3, degrees, false);
  wait(0.1, seconds);
  frontarmmotorgroup.spinToPosition(350, degrees, false);
  extendclawbalancer();
}

void lowerfrontarmfromstandoffheight() {
  frontarmmotorgroup.setVelocity(100, percent);
  frontclawopen();
  frontarmmotorgroup.spinToPosition(200, degrees, true);
  frontarmmotorgroup.setStopping(brake);
  frontarmmotorgroup.spinToPosition(5, degrees, true);
  frontarmmotorgroup.setVelocity(100, percent);
  LeftMotor.setStopping(brake);
  RightMotor.setStopping(brake);
}

void stack110() {
  BackArmMotorGroup.setStopping(coast);
  BackArmMotorGroup.setVelocity(30, percent);
  spinbackarmdown();
  wait(0.57, seconds);
  BackArmMotorGroup.stop();
  BackArmMotorGroup.setVelocity(100, percent);
  openfinger();
  spinbackarmup();
  if (BackArmMotorGroup.position(degrees) > 380) {
    while (BackArmMotorGroup.position(degrees) < 440) {
      wait(20, msec);
    }
  }
  wait(0.2, seconds);
  backarmstop();
  backarmstatecounter = 0;
}
void stack121() {
  LeftMotor.stop();
  RightMotor.stop();
  BackArmMotorGroup.setVelocity(30, percent);
  spinbackarmdown();
  while (BackArmMotorGroup.position(degrees) > 480) {
    wait(20, msec);
  }
  BackArmMotorGroup.stop();
  BackArmMotorGroup.setVelocity(100, percent);
  openfinger();
  wait(0.1, seconds);
  LeftMotor.setVelocity(100, percent);
  RightMotor.setVelocity(100, percent);
  RightMotor.spin(forward);
  LeftMotor.spin(forward);
  wait(0.5, seconds);
  spinbackarmdown();
  while (BackArmMotorGroup.position(degrees) > 100) {
    wait(20, msec);
  }
  BackArmMotorGroup.setStopping(coast);
  BackArmMotorGroup.setVelocity(100, percent);
  backarmstop();
  backarmstatecounter = 0;
}

void stack91() {
  BackArmMotorGroup.setStopping(coast);
  BackArmMotorGroup.setVelocity(25, percent);
  spinbackarmdown();
  wait(1.2, seconds);
  BackArmMotorGroup.stop();
  BackArmMotorGroup.setVelocity(30, percent);
  openfinger();
  // wait(0.1, seconds);
  // spinbackarmup();
  // if (BackArmMotorGroup.position(degrees) > 100) {
  //   while (BackArmMotorGroup.position(degrees) < 170) {
  //     wait(20, msec);
  //   }
  // }
  // retractguide();
  // wait(0.25, seconds);
  // backarmstop();
  // retractguide();
  LeftMotor.setVelocity(100, percent);
  RightMotor.setVelocity(100, percent);
  RightMotor.spin(forward);
  LeftMotor.spin(forward);
  wait(2, seconds);
  spinbackarmdown();
  LeftMotor.stop();
  RightMotor.stop();
  while (BackArmMotorGroup.position(degrees) > 20) {
    wait(20, msec);
  }
  BackArmMotorGroup.setStopping(coast);
  BackArmMotorGroup.setVelocity(100, percent);
  backarmstop();
  backarmstatecounter = 0;
}

void raisefrontarmtinybit() {
  movefrontclawup();
  wait(0.2, seconds);
  frontarmmotorgroup.setStopping(hold);
  frontarmmotorgroup.stop();
}

void debug() {
  while (!Controller.ButtonRUp.pressing()) {
    wait(20, msec);
  }
}

void part1() {
  Brain.setTimer(0, seconds);
  Pid(625, 0, velocity, kp, 0, false);
  wait(0.2, seconds);
  frontclawleftclose();
  Preciseturn(46, 28, 10, 0, false);
  frontclawleftopen();
  Pid(500, 46, velocity, kp, 0, false);
  isfrontclawup = false;
  frontclawleftclose();
  wait(0.1, seconds);
  stackpins();
  isfrontclawup = true;
  // Pid(40, 44, -velocity, kp, 0, false);
  Preciseturn(93.5, 30, 10, 0, false);
  Pid(215, 93.5, velocity, kp, 0, false);
  Pid(310, 119.5, velocity, kp, 0, false);
  stackpins();
  Pid(45, 105, velocity, kp, 0, false);
  frontclawleftclose();
  Pid(279, 116, -45, kp, 0, false); // 280 no work 210 no work
  Preciseturn(174, 25, 15, 0, false);
  wait(0.2, seconds);
  Preciseturn(175, 25, 15, 0, false);
  BackArmMotorGroup.spin(reverse);
  wait(0.15, seconds);
  BackArmMotorGroup.stop();
  Pid(450, 181, -65, kp, 1.1, false);
  closefinger();
  Pid(150, 180, 35, kp, 1.1, false);
  wait(0.25, seconds);
  dumppinsontobeam();
  Preciseturn(208.5, 30, 14.5, 0, false);
  // debug();
  wait(0.15, seconds);
  Pid(250, 210.5, velocity, kp, 0, false);
  frontclawleftclose();
  // isfrontclawup = false;
  // stackpins();
  // Preciseturn(265, 55, 10, 0, false);
  // Pid(255, 266, velocity, kp, 0, false);
  // isfrontclawup = true;
  // stackpins();
  // Pid(50, 275, velocity, kp, 0, false);
  // frontclawleftclose();
  // Preciseturn(240, 40, 10, 0, false);
  // Pid(605, 250, -velocity, kp, 0, false);
  Pid(220, 218, -velocity, kp, 0, false);
  // wait(0.25, seconds);
  raisefrontarmtostandoffheight();
  Preciseturn(188, 35, 10, 0, false);
  wait(0.5, seconds);
  Pid(575, 189, velocity, kp, 1.35, false);
  LeftMotor.setStopping(hold);
  RightMotor.setStopping(hold);
  RightMotor.stop();
  LeftMotor.stop();
  frontarmmotorgroup.setVelocity(76, percent);
  frontarmmotorgroup.spinToPosition(200, degrees, false);
  while (frontarmmotorgroup.position(degrees) > 330) {
    wait(20, msec);
  }
  frontclawopen();
  frontarmmotorgroup.setVelocity(100, percent);
  LeftMotor.setStopping(brake);
  RightMotor.setStopping(brake);
  Pid(160, 180, -velocity, kp, 0, false);
  frontarmmotorgroup.setTimeout(1.5, seconds);
  frontarmmotorgroup.spinToPosition(6.7, degrees, false);
  frontarmmotorgroup.setStopping(brake);
  Preciseturn(30, 40, 10, 0, false);
  wait(0.2, seconds);
  raisebackarmtoministandoffstack();
  // Pid(7000, -5, -40, kp, 2.5, false);
  LeftMotor.setVelocity(50, percent);
  RightMotor.setVelocity(50, percent);
  LeftMotor.spin(reverse);
  RightMotor.spin(reverse);
  wait(1.58, seconds);
  // deployguide();
  LeftMotor.setVelocity(20, percent);
  RightMotor.setVelocity(20, percent);
  RightMotor.spin(forward);
  LeftMotor.spin(forward);
  wait(0.65, seconds);
  // LeftMotor.setVelocity(25, percent);
  // RightMotor.setVelocity(25, percent);
  // LeftMotor.spin(reverse);
  // RightMotor.spin(reverse);
  // wait(0.75, seconds);
  LeftMotor.stop();
  RightMotor.stop();
  stack110();
  // retractguide();
  // LeftMotor.stop();
  // RightMotor.stop();
  // grabstartingpin();
  frontclawleftclose();
  Pid(700, 10, velocity, kp, 2, false);
  spinbackarmdown();
  Pid(100, 0, -velocity, kp, 0, false);
  frontclawleftopen();
  Preciseturn(100, 30, 10, 0, false);
  backarmstop();
  Pid(665, 107.5, velocity, kp, 0, false);
  frontclawleftclose();
  Pid(102, 100, -velocity, kp, 0, false);
  Preciseturn(78.5, 30, 10, 0, false);
  frontclawleftopen();
  Pid(650, 80, velocity, kp, 2, false);
  Brain.playSound(siren);
  frontclawleftclose();
  Pid(240, 90, -velocity, kp, 2, false);
  Preciseturn(180, 30, 10, 0, false);
  isfrontclawup = false;
  stackpins();
  Pid(230, 201, velocity, kp, 2, false);
  wait(0.15, seconds);
  Pid(240, 150, velocity, kp, 2, false);
  isfrontclawup = true;
  stackpins();
  Pid(50, 160, velocity, kp, 0, false);
  frontclawleftclose();
  Pid(143, 164, -velocity, kp, 0, false);
  Preciseturn(270, 70, 10, 0, false);
  Pid(100, 235, velocity, kp, 0, false);
  Preciseturn(265, 30, 10, 0, false);
  BackArmMotorGroup.setStopping(hold);
  BackArmMotorGroup.stop();
  BackArmMotorGroup.spin(reverse);
  wait(0.25, seconds);
  BackArmMotorGroup.setStopping(hold);
  BackArmMotorGroup.stop();
  LeftMotor.setVelocity(100, percent);
  RightMotor.setVelocity(100, percent);
  RightMotor.spin(reverse);
  LeftMotor.spin(reverse);
  wait(1.55, seconds);
  LeftMotor.stop();
  RightMotor.stop();
  closefinger();
  wait(0.3, seconds);
  Pid(40, 280, velocity, kp, 2, false);
  dumppinsontobeam();
  Pid(162, 280, velocity, kp, 2, false);
  Preciseturn(353, 30, 10, 0, false);
  wait(0.75, seconds);
  Preciseturn(359, 30, 10, 0, false);
  raisebackarmtogroundystack();
  deployguide();
  Pid(160, 359, -velocity, kp, 0, false);
  printf("time is %.2f\n", Brain.Timer.value());
  stack91();
  // frontclawleftclose();
  // isfrontclawup = false;
  // stackpins();
  // Preciseturn(186, 10, 10, 0, false);
  // Pid(175, 180, velocity, kp, 1.5, false);
  // isfrontclawup = true;
  // stackpins();
  // wait(7, seconds);
  // part2();

  debug();

  openfinger();
  LeftMotor.setVelocity(60, percent);
  RightMotor.setVelocity(60, percent);
  RightMotor.spin(forward);
  LeftMotor.spin(forward);
  wait(0.25, seconds);
  LeftMotor.stop();
  RightMotor.stop();
  BackArmMotorGroup.spinToPosition(0, degrees, true);
  LeftMotor.setVelocity(100, percent);
  RightMotor.setVelocity(100, percent);
  RightMotor.spin(reverse);
  LeftMotor.spin(reverse);
  wait(0.75, seconds);
  LeftMotor.setStopping(coast);
  RightMotor.setStopping(coast);
  LeftMotor.stop();
  RightMotor.stop();
  wait(0.2, seconds);
  closefinger();
  wait(0.4, seconds);
  Pid(50, 180, velocity, kp, 2, false); // 0.95
  openfinger();
  wait(0.4, seconds);
  BackArmMotorGroup.spinToPosition(0, degrees, false);
  Pid(30, 180, -velocity, kp, 2, false);
  closefinger();
  wait(0.6, seconds);
  BackArmMotorGroup.spinToPosition(77, degrees, false);
  wait(0.2, seconds);
  // Pid(600, 180, velocity, kp, 0.4, false);
  // Pid(150, 180, velocity, kp, 0, false);
  // Preciseturn(190.75, 30, 10, 0, false);
  frontclawleftclose();
  frontarmmotorgroup.setStopping(hold);
  frontarmmotorgroup.spinToPosition(92, degrees, false);
  retractclawbalancer();
  Pid(800, 190, velocity, kp, 2, false); // 0.95
  frontclawleftopen();
  // debug();
  // Pid(800, 186, velocity, kp, 0.9, false);
  frontarmmotorgroup.spin(reverse);
  wait(0.5, seconds);
  frontarmmotorgroup.stop();
  // wait(0.2, seconds);
  // Pid(200, 186, velocity, kp, 2.5, false);
  Pid(160, 165, -50, kp, 0, false);
  backarmstatecounter = 2;
  // Preciseturn(130, 35, 10, 0, false);
  // wait(0.2, seconds);
  Preciseturn(129, 35, 10, 0, false);
  Pid(145, 129, 40, kp, 0, false);
  Preciseturn(122, 35, 10, 0, false);
  Pid(205, 121.5, 40, kp, 0, false);
  frontclawleftclose();
  isfrontclawup = false;
  stackpins();
  // wait(0.2, seconds);
  Pid(100, 140, -40, kp, 0, false);
  Pid(250, 147, 40, kp, 0, false); // 135
  Preciseturn(125, 30, 10, 0, false);
  Pid(275, 124, 40, kp, 0, false);
  isfrontclawup = true;
  stackpins();
  Pid(50, 125, 40, kp, 0, false);
  frontclawleftclose();
  Pid(234, 145, -velocity, kp, 0, false); // 245
  raisebackarmtoministandoffstack();
  Preciseturn(88, 40, 15, 0, false);
  wait(0.2, seconds);
  Preciseturn(88, 40, 15, 0, false);
  Pid(550, 88, -velocity, kp, 2, false);
  stack110();
  Pid(365, 88, velocity, kp, 2.7, false);
  Preciseturn(280, 35, 10, 0, false);
  Pid(255, 245, velocity, kp, 2.35, false);
  Pid(600, 270, -velocity, kp, 2.2, false);
  closefinger();
  // Pid(100, 270, velocity, kp, 0, false);
  // Pid(100, 285, -velocity, kp, 1, false);
  dumppinsontobeam();
  openfinger();
  Pid(300, 270, velocity, kp, 1.5, false);
  Preciseturn(195, 30, 10, 0, false);
  Pid(600, 195, -velocity, kp, 1.5, false);
}

void part2() {
  BrainInertial.setHeading(0, degrees);
  BrainInertial.setRotation(0, degrees);
  Pid(540, 0, velocity, kp, 0, false);
  Preciseturn(-90, 30, 15, 0, false);
  Pid(400, -90, velocity, kp, 0.9, false);
  Pid(125, -90, -velocity, kp, 0, false);
  Pid(400, -90, velocity, kp, 1.5, false);
  isfrontclawup = false;
  frontclawleftclose();
  wait(0.1, seconds);
  stackpins();
  Pid(150, -90, -velocity, kp, 0, false);
  wait(0.15, seconds);
  Pid(400, -90, velocity, kp, 0.9, false);
  wait(0.15, seconds);
  Pid(150, -90, -velocity, kp, 0, false);
  wait(0.15, seconds);
  Pid(400, -90, velocity, kp, 1.5, false);
  isfrontclawup = true;
  stackpins();
  isfrontclawup = false;
  frontclawleftclose();
  Pid(150, -90, -velocity, kp, 0, false);
  Pid(400, -90, velocity, kp, 0.9, false);
  Pid(150, -90, -velocity, kp, 0, false);
  Pid(400, -90, velocity, kp, 1.5, false);
  LeftMotor.setVelocity(50, percent);
  RightMotor.setVelocity(50, percent);
  LeftMotor.spin(forward);
  RightMotor.spin(forward);
  wait(0.15, seconds);
  isfrontclawup = true;
  stackpins();
  LeftMotor.spin(reverse);
  RightMotor.spin(reverse);
  // frontclawrightclose();
  // Preciseturn(-85, 30, 10, 0, false);
  // wait(0.2, seconds);
  // Pid(290, -85, velocity, kp, 0, false);
  // frontclawleftclose();
  // stackpins();
  // Pid(165, -90, velocity, kp, 0, false);
  // Pid(215, -110, velocity, kp, 0, false);
  // wait(0.3, seconds);
  // Preciseturn(-90, 30, 10, 0, false);
  // Pid(35, -90, velocity, kp, 0, false);
  // Pid(25, -90, -velocity, kp, 0, false);
  // stackpins();
  // // Pid(40, -110, -velocity, kp, 1, false);
  // // Pid(40, -110, velocity, kp, 1, false);
  // frontclawleftclose();
  // frontclawrightclose();
  // Pid(300, -110, -velocity, kp, 0, false);
  // Pid(600, -180, -velocity, kp, 0, false);
}

void autofunc() {
  // Pid(1500, 0, velocity, kp, 0, false);
  part1();
  // wait(5, seconds);
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
  // eventraisebackarmtonest = event(raisebackarmtonest);
  Controller.ButtonR3.pressed(part1);
  Controller.ButtonL3.pressed(part2);
  Controller.ButtonFUp.pressed(autofunc);
  Controller.ButtonEUp.pressed(dumppinsontobeam);
  touchled5.pressed(autofunc);
  Brain.playSound(tada);
  eventfrontclawgoup = event(stackpins);
  eventfrontclawgodownforbeam = event(frontarmgodownforbeam);
  eventbackarmgodownforbeam = event(backarmgodownforbeam);
  // eventraisebackarmtonest = event(raisebackarmtonest);
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
    // printf("positioning of backarm is %.2f\n",
    // BackArmMotor1.position(degrees)); printf("positioning of B is %d\n",
    // Controller.AxisB.position());
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
