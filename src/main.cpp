/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Alex Cutforth                                             */
/*    Created:      Thu March 13 2020                                         */
/*    Description:  Code for 4067X in the 2020-2021 Vex Change-Up Challenge */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"

using namespace vex;

competition Competition;
controller Controller1;


motor lDrive1(PORT1, ratio18_1);
motor lDrive2(PORT2,  ratio18_1);
motor rDrive1(PORT19, ratio18_1, true);
motor rDrive2(PORT20,  ratio18_1, true);
motor_group lDrive(lDrive1, lDrive2);
motor_group rDrive(rDrive1, rDrive2);
drivetrain mainDrive(lDrive, rDrive, 319.19, 295, 370, mm, 1);


const int MOTOR_ACCEL_LIMIT = 4;

const float USER_DRIVE_SPEED = 100;
const float AUTON_DRIVE_SPEED = 60;
const float AUTON_ROTATE_SPEED = 60;

const int AutonWaitTimeCycle = 100;
const int CYCLE_SPEED_NORMAL = 10;

int CURRENT_AUTON = 0; //0=Red Small Corner, 1=Red Big Corner, 2=Blue Small Corner, 3=Blue Big Corner


int s_lastL = 0;
int s_lastR = 0;

void setSideSpeeds(int lSpeed, int rSpeed)
{
 if ((lSpeed - s_lastL) > MOTOR_ACCEL_LIMIT)
  {
    lSpeed = s_lastL + MOTOR_ACCEL_LIMIT;
  }
  if ((lSpeed - s_lastL) < -MOTOR_ACCEL_LIMIT)
  {
    lSpeed = s_lastL - MOTOR_ACCEL_LIMIT;
  }
  if ((rSpeed - s_lastR) > MOTOR_ACCEL_LIMIT)
  {
    rSpeed = s_lastR + MOTOR_ACCEL_LIMIT;
  }
  if ((rSpeed - s_lastR) < -MOTOR_ACCEL_LIMIT)
  {
    rSpeed = s_lastR - MOTOR_ACCEL_LIMIT;
  }
  s_lastL = lSpeed;
  s_lastR = rSpeed;

  if (lSpeed == 0)
  {
    lDrive.stop(brakeType::brake);
  }
  else 
  {
    lDrive.spin(directionType::fwd, lSpeed, velocityUnits::pct);
  }
  if (rSpeed == 0)
  {
    rDrive.stop(brakeType::brake);
  }
  else 
  {
    rDrive.spin(directionType::fwd, rSpeed, velocityUnits::pct);
  }
}
void forwardCm(int dist, bool waitAtEnd, int extraSpeed, bool async)
{
  mainDrive.driveFor(dist, distanceUnits::cm, AUTON_DRIVE_SPEED + extraSpeed, velocityUnits::pct, !async);
  if(!async)
    setSideSpeeds(0, 0);
  if (waitAtEnd)
    task::sleep(AutonWaitTimeCycle);
}

void rotate(int degrees, bool waitAtEnd, int extraSpeed, bool async)
{
  mainDrive.turnFor(degrees / 1.75, rotationUnits::deg, AUTON_ROTATE_SPEED + extraSpeed, velocityUnits::pct, !async);
  if(!async)
    setSideSpeeds(0, 0);
  if (waitAtEnd)
    task::sleep(AutonWaitTimeCycle);
}
void RedAuton1() 
{
  //forwardCm(20, true, 0, false);
  rotate(180, false, 0, false);
}
void RedAuton2()
{

}
void BlueAuton1()
{

}
void BlueAuton2()
{
  
}

void autonomous()
{
    switch(CURRENT_AUTON)
    {
      case 0:
      RedAuton1();
      break;
      case 1:
      RedAuton2();
      break;
      case 2:
      BlueAuton1();
      break; 
      case 3:
      BlueAuton2();
      break;
    }
}
void usercontrol() 
{
  while (1) 
  {
    setSideSpeeds(Controller1.Axis3.position() * USER_DRIVE_SPEED / 100, Controller1.Axis2.position() * USER_DRIVE_SPEED / 100);
    task::sleep(CYCLE_SPEED_NORMAL);
  }
}
void ScrollAutonUp()
{
  if (CURRENT_AUTON < 3)
    CURRENT_AUTON++;
  Controller1.Screen.clearScreen();
  Controller1.Screen.print("Current Auton is", CURRENT_AUTON);
}
void ScrollAutonDown()
{
  if (CURRENT_AUTON > 0)
    CURRENT_AUTON--;
  Controller1.Screen.clearScreen();
  Controller1.Screen.print("Current Auton is", CURRENT_AUTON);
}
void pre_auton()
{
  Controller1.ButtonRight.pressed(ScrollAutonUp);
  Controller1.ButtonLeft.pressed(ScrollAutonDown);
}
int main() 
{
  vexcodeInit();
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
  pre_auton();

  while (true) {
    wait(100, msec);
  }
}
