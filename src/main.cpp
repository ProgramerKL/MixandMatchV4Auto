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
motor LeftMotor = motor(PORT8, false);
motor RightMotor = motor(PORT2, true);
motor motor2 = motor(PORT2, true);
motor motor7 = motor(PORT7, true);
motor motor1 = motor(PORT1, false);
motor motor8 = motor(PORT8, false);
motor motor6 = motor(PORT6, false);
motor motor12 = motor(PORT12, true);
distance distancesensorightclaw = distance(PORT11);
distance distancesensorleftclaw = distance(PORT5);
pneumatic P1 = pneumatic(PORT4);
// from front view left
pneumatic P2 = pneumatic(PORT10);
pneumatic P3 = pneumatic(PORT3);
distance distancesensorfourbar = distance(PORT6);
float Rmotorspeed, Lmotorspeed, Rightstick, Leftstick, deadband = 10;
bool crawlmodestate;
bool isfrontclawrightopen;
bool isfrontclawleftopen;
bool isfingeropen;
bool isbackarmup;
bool isclawsensorsdetecting;
bool ispusherextended;
int distancedetection = 75;
int backarmstatecounter;
bool isstandoffgoalstacking;
bool isfrontclawup;
bool startingpingrabstate;
int crawlspeed = 35;
void lowerfromstandoffgoal();
void splitdrivewithcrawlmode();
event eventfrontclawgodown = event();
thread drivetrainthread = thread();
event eventraisebackarmtogroundystack = event();
event eventraisebackarmtoministandoffstack = event();
event eventraisebackarmtobigstandoffystack = event();
event eventraisebackarmtonest = event();
thread raisebackarmthread = thread();
// ==============================================================================
// HELPER FUNCTIONS
// ==============================================================================

void deployguide() { P3.extend(cylinder1); }

void retractguide() { P3.retract(cylinder1); }

void retractfrontguide() { P3.retract(cylinder2); }
void extendfrontguide() { P3.extend(cylinder2); }

void pumpon() {
  P1.pumpOn();
  P2.pumpOn();
}

void extendpusher() {
  P1.extend(cylinder2);
  ispusherextended = true;
}
void retractpusher() {
  P1.retract(cylinder2);
  ispusherextended = false;
}

void frontclawleftclose() {
  P1.retract(cylinder1);
  isfrontclawleftopen = false;
}

void frontclawleftopen() {
  P1.extend(cylinder1);
  isfrontclawleftopen = true;
}

void frontclawrightclose() {
  P2.retract(cylinder2);
  isfrontclawrightopen = false;
}
void frontclawrightopen() {
  P2.extend(cylinder2);
  isfrontclawrightopen = true;
}

void frontclawopen() {
  frontclawleftopen();
  frontclawrightopen();
}

void closefinger() {
  P2.extend(cylinder1);
  isfingeropen = false;
}

void openfinger() {
  P2.retract(cylinder1);
  isfingeropen = true;
}

void spinbackarmup() {
  motor1.setStopping(hold);
  motor7.setStopping(hold);
  motor1.spin(forward);
  motor7.spin(forward);
}

void spinbackarmdown() {
  motor1.setStopping(coast);
  motor7.setStopping(coast);
  motor1.spin(reverse);
  motor7.spin(reverse);
}

void backarmstop() {
  motor1.stop();
  motor7.stop();
}

void movefrontclawup() {
  isfrontclawup = true;
  motor6.spin(forward);
  motor12.spin(forward);
}

void movefrontclawdown() {
  isfrontclawup = false;
  motor6.spin(reverse);
  motor12.spin(reverse);
}

void frontclawstop() {
  motor6.stop();
  motor12.stop();
}

void fingercontrol() {
  if (isfingeropen) {
    spinbackarmdown();
    wait(100, msec);
    closefinger();
    wait(0.4, seconds);
    // eventraisebackarmtonest.broadcast();
    // wait(0.2, seconds);
    // spinbackarmup();
    // while (motor1.position(degrees) < 139) {
    //   wait(20, msec);
    // }
    // motor1.setStopping(hold);
    // motor7.setStopping(hold);
    backarmstop();
    motor1.setStopping(coast);
    motor7.setStopping(coast);
  } else {
    motor1.setStopping(coast);
    motor7.setStopping(coast);
    spinbackarmdown();
    wait(0.45, seconds);
    backarmstop();
    openfinger();
    if (motor1.position(degrees) > 250) {
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
    while (motor1.position(degrees) > 20) {
      wait(20, msec);
    }
    motor1.setStopping(coast);
    motor7.setStopping(coast);
    backarmstop();
    backarmstatecounter = 0;
    retractguide();
  }
}

void dumppinsontobeam() {
  closefinger();
  motor6.setStopping(hold);
  motor12.setStopping(hold);
  movefrontclawup();
  while (motor6.position(degrees) < 520) { // 625
    wait(20, msec);
  }
  motor6.setStopping(coast);
  motor12.setStopping(coast);
  motor6.setVelocity(0, percent);
  motor12.setVelocity(0, percent);
  while (motor6.position(degrees) < 720) {
    wait(20, msec);
  }
  frontclawopen();
  frontclawstop();
  isclawsensorsdetecting = false;
  motor12.setVelocity(100, percent);
  motor6.setVelocity(100, percent);
  wait(0.2, seconds);
  eventfrontclawgodown.broadcast();
  wait(0.2, seconds);
  // spinbackarmdown();
  // wait(0.2, seconds);
  motor1.setStopping(hold);
  motor7.setStopping(hold);
  backarmstop();
  wait(1, seconds);
  motor6.setStopping(coast);
  motor12.setStopping(coast);
  frontclawstop();
  eventraisebackarmtonest.broadcast();
  isclawsensorsdetecting = true;
}

// ==============================================================================
// ARM FUNCTIONS
// ==============================================================================

void lowerbackarmfromstandoff() {
  spinbackarmdown();
  while (motor1.position(degrees) > 1080) {
    wait(20, msec);
  }
  openfinger();
  while (motor1.position(degrees) > 10) {
    wait(20, msec);
  }
  backarmstop();
}

void raisebackarmtogroundystack() {
  if (motor1.position(degrees) > 740) {
    spinbackarmdown();
    while (motor1.position(degrees) > 740) {
      wait(20, msec);
    }
    motor1.setStopping(hold);
    motor7.setStopping(hold);
    backarmstop();
  } else {
    spinbackarmup();
    while (motor1.position(degrees) < 740) {
      wait(20, msec);
    }
    backarmstop();
  }
}

void raisebackarmtoministandoffstack() {
  if (motor1.position(degrees) > 1100) {
    spinbackarmdown();
    while (motor1.position(degrees) > 1100) {
      wait(20, msec);
    }
    motor1.setStopping(hold);
    motor7.setStopping(hold);
    backarmstop();
  } else {
    spinbackarmup();
    while (motor1.position(degrees) < 1100) {
      wait(20, msec);
    }
    backarmstop();
  }
}

void raisebackarmtobigstandoffystack() {
  if (motor1.position(degrees) > 1240) {
    spinbackarmdown();
    while (motor1.position(degrees) > 1240) {
      wait(20, msec);
    }
    motor1.setStopping(hold);
    motor7.setStopping(hold);
    backarmstop();
  } else {
    spinbackarmup();
    while (motor1.position(degrees) < 1240) {
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
    deployguide();
  } else if (backarmstatecounter % 3 == 2) { // raise to 110 height
    raisebackarmthread = thread(raisebackarmtoministandoffstack);
    closefinger();
  } else if (backarmstatecounter % 3 == 0) { // raise to 121 height
    raisebackarmthread = thread(raisebackarmtobigstandoffystack);
  }
}

// ==============================================================================
// DRIVER FUNCTIONS
// ==============================================================================

void splitdrivewithcrawlmode() {
  while (true) {
    if (!isstandoffgoalstacking) {
      LeftMotor.setStopping(brake);
      RightMotor.setStopping(brake);
    } else {
      LeftMotor.setStopping(hold);
      RightMotor.setStopping(hold);
    }
    int absA = fabs(Controller.AxisA.position());
    int absC = fabs(Controller.AxisC.position());
    float A_position = Controller.AxisA.position();
    float C_position = Controller.AxisC.position();
    if (crawlmodestate) {
      LeftMotor.setStopping(hold);
      RightMotor.setStopping(hold);
      if (A_position > deadband) {
        A_position = crawlspeed;
      } else if (A_position < deadband * (-1)) {
        A_position = crawlspeed * (-1);
      } else {
        A_position = 0;
      }
    }

    if (crawlmodestate) {
      if (C_position > deadband) {
        C_position = crawlspeed;
      } else if (C_position < deadband * (-1)) {
        C_position = crawlspeed * (-1);
      } else {
        C_position = 0;
      }
    }
    float LeftSpeed = A_position + C_position;
    float RightSpeed = A_position - C_position;

    if ((absA + absC) > deadband) {
      LeftMotor.setVelocity(LeftSpeed, percent);
      RightMotor.setVelocity(RightSpeed, percent);
    } else {
      LeftMotor.setVelocity(0, percent);
      RightMotor.setVelocity(0, percent);
    }
    LeftMotor.spin(forward);
    RightMotor.spin(forward);
    wait(20, msec);
  }
}

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
  motor1.setStopping(hold);
  motor7.setStopping(hold);
  LeftMotor.setMaxTorque(100, percent);
  RightMotor.setMaxTorque(100, percent);
  motor2.setStopping(hold);
  motor1.setVelocity(100, percent);
  motor7.setVelocity(100, percent);
  motor1.setMaxTorque(100, percent);
  motor7.setMaxTorque(100, percent);
  motor2.setVelocity(100, percent);
  motor2.setMaxTorque(100, percent);
  motor6.setVelocity(100, percent);
  motor12.setVelocity(100, percent);
  motor12.setMaxTorque(100, percent);
  motor6.setMaxTorque(100, percent);
  spinbackarmdown();
  wait(0.2, seconds);
  backarmstop();
  motor1.setPosition(0, degrees);
  motor7.setPosition(0, degrees);
  retractpusher();
}

void stackpins() {
  motor6.setStopping(hold);
  motor12.setStopping(hold);
  extendfrontguide();
  if (isfrontclawup) {
    movefrontclawdown();
    while (motor6.position(degrees) > 5) {
      wait(20, msec);
    }
    frontclawopen();
    frontclawstop();
    wait(1, seconds);
    isclawsensorsdetecting = true;
  } else {
    isclawsensorsdetecting = false;
    movefrontclawup();
    while (motor6.position(degrees) < 175) {
      wait(20, msec);
    }
    frontclawstop();
  }
}

void raisebackarmtonest() {
  spinbackarmup();
  motor1.setVelocity(50, percent);
  motor7.setVelocity(50, percent);
  // while (motor1.position(degrees) < 145) {
  //   wait(20, msec);
  // }
  backarmstop();
  motor1.setVelocity(100, percent);
  motor7.setVelocity(100, percent);
}

void grabstartingpin() {
  isclawsensorsdetecting = false;
  startingpingrabstate = !startingpingrabstate;
  if (startingpingrabstate) {
    frontclawopen();
    motor12.setStopping(hold);
    motor6.setStopping(hold);
    movefrontclawup();
    while (motor6.position(degrees) < 100) {
      wait(20, msec);
    }
    frontclawstop();
  } else {
    // extendpusher();
    // wait(0.5, seconds);
    // retractpusher();
    frontclawleftclose();
    frontclawrightclose();
    extendpusher();
    wait(0.25, seconds);
    retractpusher();
    movefrontclawup();
    while (motor6.position(degrees) < 110) {
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
    retractpusher();
    // movefrontclawdown();
    // while (motor6.position(degrees) > 175) {
    //   wait(20, msec);
    // }
    // isfrontclawup = true;
    // frontclawstop();
  }
  wait(0.5, seconds);
  isclawsensorsdetecting = true;
}

void stackpinsontostandoff() {
  isstandoffgoalstacking = !isstandoffgoalstacking;
  isclawsensorsdetecting = false;
  if (isstandoffgoalstacking) {
    LeftMotor.setStopping(hold);
    RightMotor.setStopping(hold);
    movefrontclawup();
    while (motor6.position(degrees) < 290) {
      wait(20, msec);
    }
    motor12.setStopping(hold);
    motor6.setStopping(hold);
    frontclawstop();
  } else {
    extendpusher();
    movefrontclawdown();
    while (motor6.position(degrees) > 190) {
      wait(20, msec);
    }
    frontclawopen();
    while (motor6.position(degrees) > 5) {
      wait(20, msec);
    }
    motor12.setStopping(coast);
    motor6.setStopping(coast);
    frontclawstop();
    retractpusher();
    LeftMotor.setStopping(coast);
    RightMotor.setStopping(coast);
  }
  wait(0.5, seconds);
  isclawsensorsdetecting = true;
}

void stackpinsincornergoal() {
  movefrontclawup();
  while (motor6.position(degrees) < 115) {
    wait(20, msec);
  }
  motor12.setStopping(hold);
  motor6.setStopping(hold);
  frontclawstop();
  isclawsensorsdetecting = false;
  retractfrontguide();
}

void pushercontrol() {
  if (ispusherextended) {
    retractpusher();
  } else {
    extendpusher();
  }
}

void buttonlogic() {
  if (Controller.ButtonLUp.pressing()) {
    frontclawopen();
  }
  if (Controller.ButtonRDown.pressing()) {
    fingercontrol();
  }
  if (Controller.ButtonFUp.pressing()) {
    backarmcontrol();
  }
  if (Controller.ButtonR3.pressing()) {
    pushercontrol();
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
  if (Controller.ButtonL3.pressing()) {
    stackpinsincornergoal();
  }
  if (Controller.ButtonFDown.pressing()) {
    grabstartingpin();
  }
}

void disconnectionfunc() {
  if (motor1.installed() == false) {
    Brain.playSound(siren);
    Brain.Screen.print("motor1 is disconnected port1");
  }
  if (motor2.installed() == false) {
    Brain.playSound(siren);
    Brain.Screen.print("motor2 is disconnected port2");
  }
  if (motor6.installed() == false) {
    Brain.playSound(siren);
    Brain.Screen.print("motor6 is disconnected port6");
  }
  if (motor7.installed() == false) {
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
  if (motor12.installed() == false) {
    Brain.playSound(siren);
    Brain.Screen.print("motor12 is disconnected port12");
  }
  if (distancesensorleftclaw.installed() == false) {
    Brain.playSound(siren);
    Brain.Screen.print("leftclawdistancesensor disconnected port4");
  }
  if (distancesensorightclaw.installed() == false) {
    Brain.playSound(siren);
    Brain.Screen.print("rightclawdistancesensor disconnected port10");
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

int main() {
  inital();
  pumpon();
  extendfrontguide();
  drivetrainthread = thread(splitdrivewithcrawlmode);
  eventraisebackarmtonest = event(raisebackarmtonest);
  Controller.ButtonLUp.pressed(buttonlogic);
  Controller.ButtonRDown.pressed(buttonlogic);
  Controller.ButtonFUp.pressed(buttonlogic);
  Controller.ButtonR3.pressed(buttonlogic);
  Controller.ButtonEUp.pressed(buttonlogic);
  Controller.ButtonEDown.pressed(buttonlogic);
  Controller.ButtonLDown.pressed(buttonlogic);
  Controller.ButtonL3.pressed(buttonlogic);
  Controller.ButtonFDown.pressed(buttonlogic);
  Brain.playSound(tada);
  eventfrontclawgodown = event(movefrontclawdown);
  // Run main drive control loop
  while (true) {
    disconnectionfunc();
    // printf("positioning of A is %d\n", Controller.AxisA.position());
    // printf("positioning of B is %d\n", Controller.AxisB.position());
    // printf("\033[2J\n");
    // printf("\n");
    if (Controller.ButtonRUp.pressing()) {
      crawlmodestate = true;
      Brain.playSound(siren);
    } else {
      crawlmodestate = false;
    }

    if (!Controller.ButtonLUp.pressing()) {
      if (isclawsensorsdetecting) {
        if (distancesensorleftclaw.objectDistance(mm) < distancedetection) {
          if (isfrontclawleftopen) {
            Brain.playSound(doorClose);
            frontclawleftclose();
          }
        }
        if (distancesensorightclaw.objectDistance(mm) < distancedetection) {
          if (isfrontclawrightopen) {
            Brain.playSound(doorClose);
            frontclawrightclose();
          }
        }
      }
    }

    // printf("left distance sensor is %.2f\n", isrightclawfilled());
    wait(20, msec);
  }
}
