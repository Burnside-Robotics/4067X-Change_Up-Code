/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Alex Cutforth                                             */
/*    Created:      Thu March 13 2020                                         */
/*    Description:  Code for 4067X in the 2020-2021 Vex Change-Up Challenge   */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// ballBumper           bumper        H
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "Vision.h"

int s_lastL = 0;
int s_lastR = 0;

using namespace std;
using namespace vex;

competition Competition;
controller Controller1;

motor lDrive1(PORT1, ratio18_1);
motor lDrive2(PORT2, ratio18_1);
motor rDrive1(PORT3, ratio18_1, true);
motor rDrive2(PORT4, ratio18_1, true);
motor_group lDrive(lDrive1, lDrive2);
motor_group rDrive(rDrive1, rDrive2);
inertial intertialSensor = inertial(PORT19);
optical opticalSensor = optical(PORT5);

drivetrain mainDrive(lDrive, rDrive, 319.19, 295, 370, mm, 1);

motor lRoller(PORT10, ratio18_1, true);
motor rRoller(PORT9, ratio18_1);
motor_group intakeRollers(lRoller, rRoller);

motor lowerOutakeMotor(PORT11, ratio6_1);
motor upperOutakeMotor(PORT20, ratio6_1, true);

//bumper ballBumper(Brain.ThreeWirePort.H);

const float MOTOR_ACCEL_LIMIT = 5;

const float USER_DRIVE_SPEED = 100;
const float AUTON_DRIVE_SPEED = 45;
const float AUTON_ROTATE_SPEED = 100;

const float ROLLER_SPEED = 100;
const float OUTAKE_SPEED = 100;

const int CYCLE_SPEED_NORMAL = 10;

int CURRENT_AUTON = 0; // 0=Red Right, 1=Red Left, 2=Blue Right, 3=Blue Left
bool autonConfirmed = false;

bool isBlueTeam;
bool antiTipEnabled = true;
bool autoHopperEnabled = true;

float wheelCircumfrence = 31.9185813596f;
float turningDiameter = 33;

void setSideSpeeds(int lSpeed, int rSpeed)
{
    if ((lSpeed - s_lastL) > MOTOR_ACCEL_LIMIT)
        lSpeed = s_lastL + MOTOR_ACCEL_LIMIT;
    if ((lSpeed - s_lastL) < -MOTOR_ACCEL_LIMIT)
        lSpeed = s_lastL - MOTOR_ACCEL_LIMIT;
    if ((rSpeed - s_lastR) > MOTOR_ACCEL_LIMIT)
        rSpeed = s_lastR + MOTOR_ACCEL_LIMIT;
    if ((rSpeed - s_lastR) < -MOTOR_ACCEL_LIMIT)
        rSpeed = s_lastR - MOTOR_ACCEL_LIMIT;

    s_lastL = lSpeed;
    s_lastR = rSpeed;

    if (lSpeed == 0)
        lDrive.stop(brakeType::brake);
    else
        lDrive.spin(directionType::fwd, lSpeed, velocityUnits::pct);
    if (rSpeed == 0)
        rDrive.stop(brakeType::brake);
    else
        rDrive.spin(directionType::fwd, rSpeed, velocityUnits::pct);
}
void WaitForBumper(int amount, int maxTimeMsec)
{
    int doneAmount = 0;
    int doneTime = 0;
    while (doneAmount < amount)
    {
        if (ballBumper.pressing())
        {
            doneAmount++;
            while (ballBumper.pressing())
            {
                wait(30, timeUnits::msec);
                doneTime += 30;
                if (doneTime >= maxTimeMsec)
                    break;
            }
        }
        wait(100, timeUnits::msec);
        doneTime += 100;
        if (doneTime >= maxTimeMsec)
            break;
    }
}

float clip(float n, float lower, float upper) 
{
  return std::max(lower, std::min(n, upper));
}
void DriveDistance(float distance)
{
    lDrive1.resetPosition();
    rDrive1.resetPosition();

    float rotationGoal = (360 * distance) / wheelCircumfrence;

    float maxSpeed = 0;
    const float Kp = 0.25;
    const float Ki = 0;
    const float Kd = 0;
    const float Rp = 1;
    const float deadZone = 1;

    float distPid = 0;
    float distError = 0;
    float rotError = 0;
    float integral = 0;

    float derivative = 0;

    float lastError = 0;

    float lMotorSpeed = 0;
    float rMotorSpeed = 0;

    float doneTime = 0;
    while (true)
    {
        distError = rotationGoal - (lDrive1.rotation(deg) + rDrive1.rotation(deg)) / 2;
        rotError = lDrive1.rotation(deg) - rDrive1.rotation(deg);

        integral += distError;

        if (distError > 200)
        {
            integral = 0;
        }
        if (distError > 200)
        {
            integral = 0;
        }

        derivative = distError - lastError;

        lastError = distError;

        distPid = clip(Kp * distError + Ki * integral + Kd * integral, -100, 100);
        lMotorSpeed = distPid - rotError * Rp;
        rMotorSpeed = distPid + rotError * Rp;

        if (doneTime < 500) 
        {
          maxSpeed = doneTime / 5;
        }
        else
        {
            if (lMotorSpeed < deadZone && lMotorSpeed > -deadZone && rMotorSpeed < deadZone && rMotorSpeed > -deadZone)
            {
                rDrive.spin(fwd, 0, pct);
                lDrive.spin(fwd, 0, pct);
                break;
            }
        }
        
        lMotorSpeed = clip(lMotorSpeed, -maxSpeed, maxSpeed);
        rMotorSpeed = clip(rMotorSpeed, -maxSpeed, maxSpeed);

        lDrive.spin(fwd, lMotorSpeed, pct);
        rDrive.spin(fwd, rMotorSpeed, pct);

        wait(15, msec);
        doneTime += 15;
    }
}
void RotateDegrees(int degrees)
{
    //intertialSensor.resetHeading();

    lDrive1.resetPosition();
    rDrive1.resetPosition();

    float maxSpeed = 100;

    const float Kp = 0.4;
    const float Ki = 0.05;
    const float Kd = 0;
    const float deadZone = 1;

    float error = 0;
    float integral = 0;

    float derivative = 0;

    float lastError = 0;

    float motorSpeed = 0;

    float doneTime = 0;
    
    float turningRatio = turningDiameter / (wheelCircumfrence / 3.1415926535);
    float target = turningRatio * degrees;
    while (true)
    {
        error = target - (lDrive1.rotation(deg) - rDrive1.rotation(deg)) / 2;
        integral += error;
        if (error > 40 || error < -40)
        {
            integral = 0;
            derivative = 0;
        }
        else
        {
            if (motorSpeed < deadZone && motorSpeed > -deadZone)
            {
                rDrive.spin(fwd, 0, pct);
                lDrive.spin(fwd, 0, pct);
                break;
            }
        }
        derivative = error - lastError;

        lastError = error;

        motorSpeed = Kp * error + Ki * integral + Kd * derivative;

        if (doneTime < 200)
        {
            if (degrees < 0)
                maxSpeed = -(doneTime / 2);
            else
                maxSpeed = doneTime / 2;
        }

        if (degrees > 0)
        {
            motorSpeed = maxSpeed < motorSpeed ? maxSpeed : motorSpeed;
        }
        else
        {
            motorSpeed = maxSpeed > motorSpeed ? maxSpeed : motorSpeed;
        }
        lDrive.spin(fwd, motorSpeed, pct);
        rDrive.spin(fwd, -motorSpeed, pct);
        if (int(doneTime) % 60 == 0)
        {
            Brain.Screen.clearScreen();
            Brain.Screen.setCursor(4, 4);
            Brain.Screen.print(target);
            Brain.Screen.setCursor(6, 6);
            Brain.Screen.print(error);
        }
        wait(15, msec);
        doneTime += 15;
    }
}
void ToggleScorer(bool on, bool rev)
{
    lowerOutakeMotor.spin(fwd, on ? OUTAKE_SPEED : 0, pct);
    upperOutakeMotor.spin(fwd, on ? rev ? OUTAKE_SPEED : -OUTAKE_SPEED : 0, pct);
}
void ToggleBackOutake(bool on)
{
    lowerOutakeMotor.spin(fwd, on ? OUTAKE_SPEED : 0, pct);
    upperOutakeMotor.spin(fwd, on ? OUTAKE_SPEED : 0, pct);
}
void ToggleIntakeRoller(bool on, bool rev)
{
    intakeRollers.spin(fwd, on ? rev ? ROLLER_SPEED : -ROLLER_SPEED : 0, pct);
}
void RightCorner()
{
    upperOutakeMotor.spinFor(fwd, 90, deg, 100, velocityUnits::pct);
    DriveDistance(-47);
    RotateDegrees(-56);
    ToggleIntakeRoller(true, false);
    DriveDistance(59);
    ToggleScorer(true, false);
    WaitForBumper(3, 4000);
    ToggleIntakeRoller(false, false);
    WaitForBumper(1, 1000);
    ToggleScorer(false, false);
    DriveDistance(-50);
}
void RightCornerAndCenter()
{
    RightCorner();
    ToggleIntakeRoller(true, false);
    ToggleScorer(true, false);
    RotateDegrees(145);
    DriveDistance(106);
    ToggleScorer(false, false);
    RotateDegrees(-97);
    DriveDistance(25);
    ToggleScorer(true, false);
    wait(1.5, sec);
    ToggleIntakeRoller(false, false);
    DriveDistance(-18.5);
    ToggleScorer(false, false);
}
void LeftCorner()
{
     upperOutakeMotor.spinFor(fwd, 90, deg, 100, velocityUnits::pct);
    DriveDistance(-47);
    RotateDegrees(56);
    ToggleIntakeRoller(true, false);
    DriveDistance(59);
    ToggleScorer(true, false);
    WaitForBumper(3, 4000);
    ToggleIntakeRoller(false, false);
    WaitForBumper(1, 1000);
    ToggleScorer(false, false);
    DriveDistance(-50);
}
void LeftCornerAndCenter()
{
    LeftCorner();
    ToggleIntakeRoller(true, false);
    ToggleScorer(true, false);
    RotateDegrees(-145);
    DriveDistance(106);
    ToggleScorer(false, false);
    RotateDegrees(97);
    DriveDistance(25);
    ToggleScorer(true, false);
    wait(1.5, sec);
    ToggleIntakeRoller(false, false);
    DriveDistance(-18.5);
    ToggleScorer(false, false);
}
bool DrawButton(int xPos, int yPos, int width, int height, color Color)
{
    Brain.Screen.drawRectangle(xPos, yPos, width, height, Color);
    int pressX = Brain.Screen.xPosition();
    int pressY = Brain.Screen.yPosition();

    if (Brain.Screen.pressing())
    {
        if (pressX > xPos && pressX < xPos + width && pressY > yPos &&
            pressY < yPos + height)
        {
            return true;
        }
    }
    return false;
}
void UpdateSelectionScreen()
{
    Controller1.Screen.clearScreen();
    if (!Competition.isEnabled())
    {
        Controller1.Screen.setCursor(0, 0);
        Controller1.Screen.print("Auton(^v) Is ");
        Controller1.Screen.setCursor(2, 0);
        switch (CURRENT_AUTON)
        {
        case 0:
            Controller1.Screen.print("Right Corner");
            Controller1.Screen.setCursor(4, 0);
            Controller1.Screen.print(" And Center");
            break;
        case 1:
            Controller1.Screen.print("Right Corner");
            break;
        case 2:
            Controller1.Screen.print("Left Corner");
            Controller1.Screen.setCursor(4, 0);
            Controller1.Screen.print(" And Center");
            break;
        case 3:
            Controller1.Screen.print("Left Corner");
            break;
        }
    }
    else if (Competition.isDriverControl())
    {
        Controller1.Screen.setCursor(0, 0);
        Controller1.Screen.print("Anti Tip(Y) ");
        Controller1.Screen.print(antiTipEnabled ? "Enabled" : "Disabled");
        Controller1.Screen.setCursor(2, 0);
        Controller1.Screen.print("Hopper(B) ");
        Controller1.Screen.print(autoHopperEnabled ? "Enabled" : "Disabled");
        Controller1.Screen.setCursor(5, 0);
        Controller1.Screen.print("Team(<>) Is ");
        Controller1.Screen.print(isBlueTeam ? "Blue" : "Red");
    }
}
void HoneOnBall()
{
    FrontVision.takeSnapshot(isBlueTeam ? BlueBall : RedBall);
    vex::vision::object obj = FrontVision.largestObject;
    intakeRollers.spin(fwd, -100, pct);
    if (obj.height > 30 && obj.width > 70)
    {
        if (obj.centerX > 158)
        {
            setSideSpeeds(100, 100 + (158 - obj.centerX) * 0.5);
        }
        else if (obj.centerX < 157)
        {
            setSideSpeeds(100 - (157 - obj.centerX) * 0.5, 100);
        }
        else
        {
            setSideSpeeds(100, 100);
        }
    }
    else
    {
        setSideSpeeds(0, 0);
    }
}
void drivercontrol()
{
    UpdateSelectionScreen();
    while (1)
    {
        //Hopper Management
        if (autoHopperEnabled)
        {
            if (opticalSensor.isNearObject())
            {
                if (!isBlueTeam)
                {
                    if (opticalSensor.color() == color().blue)
                    {
                        lowerOutakeMotor.spin(fwd, 100, pct);
                        upperOutakeMotor.spin(fwd, 100, pct);
                        wait(300, msec);
                    }
                }
                else
                {
                    if (opticalSensor.color() == color().red)
                    {
                        lowerOutakeMotor.spin(fwd, 100, pct);
                        upperOutakeMotor.spin(fwd, 100, pct);
                        wait(300, msec);
                    }
                }
            }
        }
        //AntiTip Management
        if (intertialSensor.pitch() > 15 && antiTipEnabled)
        {
            mainDrive.setDriveVelocity(70, pct);
            mainDrive.driveFor(directionType::fwd, -30, distanceUnits::cm);
            continue;
        }

        // Set Roller speeds
        lRoller.spin(fwd,
                     Controller1.ButtonR2.pressing()
                         ? -ROLLER_SPEED
                     : Controller1.ButtonR1.pressing() ? ROLLER_SPEED
                                                       : 0,
                     pct);
        rRoller.spin(fwd,
                     Controller1.ButtonR2.pressing()
                         ? -ROLLER_SPEED
                     : Controller1.ButtonR1.pressing() ? ROLLER_SPEED
                                                       : 0,
                     pct);

        // Set Outake Motor speeds
        lowerOutakeMotor.spin(fwd,
                              Controller1.ButtonL2.pressing()
                                  ? OUTAKE_SPEED / 3
                              : Controller1.ButtonL1.pressing() ? -OUTAKE_SPEED / 3
                                                                : 0,
                              pct);
        //Manual Back Outake
        if (!Controller1.ButtonA.pressing())
            upperOutakeMotor.spin(fwd,
                                  Controller1.ButtonL2.pressing()
                                      ? -OUTAKE_SPEED
                                  : Controller1.ButtonL1.pressing() ? OUTAKE_SPEED
                                                                    : 0,
                                  pct);
        else
            upperOutakeMotor.spin(fwd, OUTAKE_SPEED, pct);

        if (Controller1.ButtonDown.pressing())
            HoneOnBall();
        else
        {
            // Set Motor Speeds
            setSideSpeeds(Controller1.Axis3.position() * USER_DRIVE_SPEED / 100,
                          Controller1.Axis2.position() * USER_DRIVE_SPEED / 100);
        }
        task::sleep(CYCLE_SPEED_NORMAL);
    }
}
void autonomous()
{
    switch (CURRENT_AUTON)
    {
    case 0:
        RightCornerAndCenter();
        break;
    case 1:
        RightCorner();
        break;
    case 2:
        LeftCornerAndCenter();
        break;
    case 3:
        LeftCorner();
        break;
    }
}

void ToggleAutoHopper()
{
    if (Competition.isAutonomous())
        return;
    autoHopperEnabled = !autoHopperEnabled;
    UpdateSelectionScreen();
}
void ToggleAntiTip()
{
    if (Competition.isAutonomous())
        return;
    antiTipEnabled = !antiTipEnabled;
    UpdateSelectionScreen();
}
void ToggleTeam()
{
    isBlueTeam = !isBlueTeam;
    UpdateSelectionScreen();
}
void SwitchAuton()
{
    CURRENT_AUTON++;
    if (CURRENT_AUTON == 4)
        CURRENT_AUTON = 0;
    UpdateSelectionScreen();
}
void pre_auton()
{
    Controller1.ButtonY.pressed(ToggleAntiTip);
    Controller1.ButtonB.pressed(ToggleAutoHopper);
    Controller1.ButtonLeft.pressed(ToggleTeam);
    Controller1.ButtonRight.pressed(ToggleTeam);
    Controller1.ButtonUp.pressed(SwitchAuton);
    UpdateSelectionScreen();
}
void Initialise()
{
    mainDrive.setDriveVelocity(AUTON_DRIVE_SPEED, pct);
    mainDrive.setTurnVelocity(AUTON_ROTATE_SPEED, pct);
    intertialSensor.calibrate();
}
int main()
{
    vexcodeInit();
    Initialise();
    Competition.autonomous(autonomous);
    Competition.drivercontrol(drivercontrol);
    pre_auton();
    UpdateSelectionScreen();
    while (true)
    {
        task::sleep(CYCLE_SPEED_NORMAL);
    }
}