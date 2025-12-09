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
motor motor9 = motor(PORT9, true);
motor motor8 = motor(PORT8, false);
distance leftclawdistancesensor = distance(PORT4);
distance rightclawdistancesensor = distance(PORT10);
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
bool frontClawState;
bool fourbarstate;
bool fingeropenstate;
bool rightclawstateopen;
bool leftclawstateopen;
bool fourbarupstate;
float Rmotorspeed, Lmotorspeed, Rightstick, Leftstick, deadband = 10;
bool frontguidestatedeployed;
bool backguidestatedeployed;
bool isClawDetecting;
bool isClawStacking;
bool crawlmodestate;
int crawlspeed = 30;
void lowerfromstandoffgoal();
thread drivetrainthread = thread();
// ==============================================================================
// HELPER FUNCTIONS
// ==============================================================================

// void disengagemotor2() { motor2.spin(reverse); }

// void disengagemotor9() { motor9.spin(reverse); }

void fourbargoup() {
  motor2.spin(forward);
  motor8.spin(reverse);
  motor9.spin(forward);
  motor8.setStopping(hold);
}

void fourbargodown() { motor8.spin(forward); }
void movefrontarmup() {
  motor2.spin(reverse);
  Motor3.spin(forward);
  // motor9.spin(reverse);
}
void movefrontarmdown() { Motor3.spin(reverse); }

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
  P2.retract(cylinder1);
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

void leftClawOpen() {
  p1c1extend();
  leftclawstateopen = true;
}
void leftClawClose() {
  p1c1retract();
  leftclawstateopen = false;
}
void rightClawOpen() {
  p2c2extend();
  rightclawstateopen = true;
}
void rightClawclose() {
  p2c2retract();
  rightclawstateopen = false;
}
void fingerOpen() {
  p2c1retract();
  fingeropenstate = true;
}
void fingerClose() {
  p2c1extend();
  fingeropenstate = false;
}
void frontguidedeploy() {
  p1c2extend();
  frontguidestatedeployed = true;
}
void frontguideretract() {
  p1c2retract();
  frontguidestatedeployed = false;
}
void guidedeploy() {
  p3c2extend();
  backguidestatedeployed = true;
}
void guideretract() {
  p3c2retract();
  backguidestatedeployed = false;
}

bool isleftclawfilled() {
  if (leftclawdistancesensor.objectDistance(mm) < 15) {
    return true;
  } else {
    return false;
  }
}

bool isrightclawfilled() {
  if (rightclawdistancesensor.objectDistance(mm) < 15) {
    return true;
  } else {
    return false;
  }
}

// ==============================================================================
// DRIVER FUNCTIONS
// ==============================================================================
void splitdrive() {
  LeftMotor.setStopping(brake);
  RightMotor.setStopping(brake);
  while (true) {
    int absA = fabs(Controller.AxisA.position());
    int absC = fabs(Controller.AxisC.position());
    float A_position = Controller.AxisA.position();
    float C_position = Controller.AxisC.position();

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
  }
  wait(20, msec);
}

void splitdrivewithcrawlmode() {
  LeftMotor.setStopping(brake);
  RightMotor.setStopping(brake);
  while (true) {
    int absA = fabs(Controller.AxisA.position());
    int absC = fabs(Controller.AxisC.position());
    float A_position = Controller.AxisA.position();
    float C_position = Controller.AxisC.position();
    if (crawlmodestate) {
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
  }
  wait(20, msec);
}

// ==============================================================================
// MAIN PROGRAM
// ==============================================================================
void initital() {
  pumpson();
  isClawDetecting = true;
  Motor3.setStopping(coast);
  motor2.setStopping(coast);
  motor9.setStopping(coast);
  motor8.setStopping(coast);
  LeftMotor.setMaxTorque(100, percent);
  RightMotor.setMaxTorque(100, percent);
  motor2.setMaxTorque(100, percent);
  Motor3.setMaxTorque(100, percent);
  motor8.setMaxTorque(100, percent);
  motor9.setMaxTorque(100, percent);
  motor2.setVelocity(100, percent);
  Motor3.setVelocity(100, percent);
  motor8.setVelocity(100, percent);
  motor9.setVelocity(100, percent);
  leftClawOpen();
  rightClawOpen();
  lowerfromstandoffgoal();
  fingerOpen();
}
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

void throwpinintobeam() {
  motor8.setStopping(coast);
  movefrontarmup();
  wait(1.2, seconds);
  motor2.stop();
  Motor3.stop();
  motor9.stop();
}

void swingarmbackdown() {
  // motor2.spin(forward); // to disengage the ptomotor
  // motor9.spin(reverse); // to disengage the ptomotor
  // wait(0.2, seconds);
  motor8.setStopping(coast);
  movefrontarmdown();
  wait(1.75, seconds);
  motor9.stop();
  Motor3.stop();
}

void stackonstandoffgoal() {
  fourbargoup();
  while (distancesensorfourbar.objectDistance(mm) < 270) {
    wait(20, msec);
  }
  motor8.stop();
  motor2.stop();
  motor9.stop();
  fourbarupstate = true;
}

void lowerfromstandoffgoal() {
  fourbargodown();
  Brain.playSound(siren);
  while (distancesensorfourbar.objectDistance(mm) > 25) {
    wait(20, msec);
  }
  motor8.setStopping(coast);
  motor8.stop();
  Brain.playSound(siren);
  fourbarupstate = false;
}

void buttonlogic() {
  if (Controller.ButtonEUp.pressing()) {
    frontClawState = !frontClawState;
    if (frontClawState) {
      swingarmbackdown();
    } else {
      throwpinintobeam();
    }
  } else if (Controller.ButtonFUp.pressing()) {
    fourbarstate = !fourbarstate;
    if (fourbarstate) {
      stackonstandoffgoal();
    } else {
      lowerfromstandoffgoal();
    }
  } else if (Controller.ButtonRDown.pressing()) {
    if (!fingeropenstate) {
      fingerOpen();
      fourbargodown();
      wait(0.75, seconds);
      motor2.setStopping(coast);
      motor8.setStopping(coast);
      motor9.setStopping(coast);
      motor2.stop();
      motor8.stop();
      motor9.stop();
    } else {
      fingerClose();
      fourbargoup();
      wait(0.5, seconds);
      motor8.setStopping(hold);
      motor2.stop();
      motor8.stop();
      motor9.stop();
    }
  } else if (Controller.ButtonLUp.pressing()) {
    // if (leftclawstateopen) {
    //   leftClawClose();
    // } else {
    //   leftClawOpen();
    // }
    // if (rightclawstateopen) {
    //   rightClawclose();
    // } else {
    //   rightClawOpen();
    // }
    leftClawOpen();
    rightClawOpen();
    while (true) {
      if (Controller.ButtonLUp.pressing()) {
        isClawDetecting = false;
      } else {
        isClawDetecting = true;
        break;
      }
      wait(20, msec);
    }
  } else if (Controller.ButtonLDown.pressing()) {
    isClawDetecting = false; // this allows the claw to release and not have the
    // detection track it
    isClawStacking = !isClawStacking;
    if (isClawStacking) {
      movefrontarmup();
      wait(1, seconds);
      Motor3.setStopping(hold);
      motor2.stop();
      Motor3.stop();
      motor9.stop();
    } else {
      movefrontarmdown();
      wait(1, seconds);
      leftClawOpen();
      rightClawOpen();
      wait(1.5, seconds);
      Motor3.stop();
    }
  }
}

void frontguidedeployandretract() {
  if (frontguidestatedeployed) {
    frontguideretract();
  } else {
    frontguidedeploy();
  }
}

void backguidedeployandretract() {
  if (backguidestatedeployed) {
    guideretract();
  } else {
    guidedeploy();
  }
}

void fingeropenandclose() {
  if (fingeropenstate) {
    fingerClose();
  } else {
    fingerOpen();
  }
}

int main() {
  initital();
  Brain.playSound(tada);
  drivetrainthread = thread(splitdrivewithcrawlmode);
  Controller.ButtonEUp.pressed(buttonlogic);
  Controller.ButtonFUp.pressed(buttonlogic);
  Controller.ButtonRDown.pressed(buttonlogic);
  Controller.ButtonLUp.pressed(buttonlogic);
  Controller.ButtonLDown.pressed(buttonlogic);
  Controller.ButtonL3.pressed(backguidedeployandretract);
  Controller.ButtonR3.pressed(fingeropenandclose);
  Controller.ButtonFDown.pressed(frontguidedeployandretract);
  LeftMotor.stop();
  RightMotor.stop();
  // Run main drive control loop
  while (true) {
    disconnectionfunc();
    // printf("positioning of A is %d\n", Controller.AxisA.position());
    // printf("positioning of B is %d\n", Controller.AxisB.position());
    // printf("\033[2J\n");
    // printf("\n");
    if (Controller.ButtonRUp.pressing()) {
      crawlmodestate = true;
    } else {
      crawlmodestate = false;
    }
    if (isClawDetecting) {
      if (isleftclawfilled()) {
        leftClawClose();
      }
      if (isrightclawfilled()) {
        rightClawclose();
      }
    }
    // printf("left distance sensor is %.2f\n", isrightclawfilled());
    wait(20, msec);
  }
}
