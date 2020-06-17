/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Alex Cutforth                                             */
/*    Created:      Thu March 13 2020                                         */
/*    Description:  Code for 4067X in the 2020-2021 Vex Change-Up Challenge   */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include "Vision.h"

int s_lastL = 0;
int s_lastR = 0;

using namespace std;
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

motor lRoller(PORT10, ratio18_1);
motor rRoller(PORT9, ratio18_1, true);

motor outakeMotorL(PORT11, ratio18_1);
motor outakeMotorR(PORT12, ratio18_1, true);

const int MOTOR_ACCEL_LIMIT = 4;

const float USER_DRIVE_SPEED = 100;
const float AUTON_DRIVE_SPEED = 60;
const float AUTON_ROTATE_SPEED = 60;

const float ROLLER_SPEED = 100;
const float OUTAKE_SPEED = 100;

const int AutonWaitTimeCycle = 100;
const int CYCLE_SPEED_NORMAL = 10;

int CURRENT_AUTON = 0; //0=Red Small Corner, 1=Red Big Corner, 2=Blue Small Corner, 3=Blue Big Corner
bool autonConfirmed;


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
void RedAuton1()
{

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


void drivercontrol() 
{
  while (1) 
  {
    if(IntakeVision.takeSnapshot(BlueBall))
    {
        lRoller.spin(fwd, 100, pct);
    }
    else if(IntakeVision.takeSnapshot(RedBall))
    {
      lRoller.spin(fwd, -100, pct);
    }
    else
    {
      lRoller.stop(brake);
    }
    //Set Motor Speeds
    setSideSpeeds(Controller1.Axis3.position() * USER_DRIVE_SPEED / 100, Controller1.Axis2.position() * USER_DRIVE_SPEED / 100);
    
    //Set Roller speeds
    lRoller.spin(fwd, Controller1.ButtonR2.pressing() ? ROLLER_SPEED : Controller1.ButtonR1.pressing() ? -ROLLER_SPEED : 0, pct);
    rRoller.spin(fwd, Controller1.ButtonR2.pressing() ? ROLLER_SPEED : Controller1.ButtonR1.pressing() ? -ROLLER_SPEED : 0, pct);
    
    //Set Outake Motor speeds
    outakeMotorR.spin(fwd, Controller1.ButtonL2.pressing() ? -OUTAKE_SPEED : Controller1.ButtonL1.pressing() ? OUTAKE_SPEED : 0, pct);
    outakeMotorL.spin(fwd, Controller1.ButtonL2.pressing() ? -OUTAKE_SPEED : Controller1.ButtonL1.pressing() ? OUTAKE_SPEED : 0, pct);
    
    task::sleep(CYCLE_SPEED_NORMAL);
  }
}
void autonomous()
{

}
void UpdateSelectionScreen()
{
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(0,0);  
  Controller1.Screen.print("Auton: ");
  if(autonConfirmed)
    Controller1.Screen.print("[");
  Controller1.Screen.print(CURRENT_AUTON);
  if(autonConfirmed)
    Controller1.Screen.print("]");
}
void AutonUp()
{
  if(autonConfirmed)
    return;
  if(CURRENT_AUTON < 3)
    CURRENT_AUTON++;
  UpdateSelectionScreen();
}
void AutonDown()
{
  if(autonConfirmed)
    return;
  if(CURRENT_AUTON > 0)
    CURRENT_AUTON--;
  UpdateSelectionScreen();
}
void ConfirmAuton()
{
  autonConfirmed = true;
  UpdateSelectionScreen();
}
void UnConfirmAuton()
{
  autonConfirmed = false;
  UpdateSelectionScreen();
}
void pre_auton()
{
  Controller1.ButtonUp.pressed( AutonUp );
  Controller1.ButtonDown.pressed( AutonDown );
  Controller1.ButtonA.pressed( ConfirmAuton );
  Controller1.ButtonB.pressed( UnConfirmAuton );
}
int main() 
{
  vexcodeInit();
  Competition.autonomous(autonomous);
  Competition.drivercontrol(drivercontrol);
  pre_auton();

  while (true) {
    wait(100, msec);
  }
}
