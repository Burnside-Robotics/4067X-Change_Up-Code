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
  inertial intertialSensor = inertial(PORT16);
  optical opticalSensor = optical(PORT8);

  drivetrain mainDrive(lDrive, rDrive, 319.19, 295, 370, mm, 1);

  motor lRoller(PORT10, ratio18_1, true);
  motor rRoller(PORT9, ratio18_1);
  motor_group intakeRollers(lRoller, rRoller);

  motor lowerOutakeMotor(PORT11, ratio6_1, true);
  motor upperOutakeMotor(PORT20, ratio6_1);

  //bumper ballBumper(Brain.ThreeWirePort.H);
  
  const int MOTOR_ACCEL_LIMIT = 100;

  const float USER_DRIVE_SPEED = 100;
  const float AUTON_DRIVE_SPEED = 45;
  const float AUTON_ROTATE_SPEED = 100;

  const float ROLLER_SPEED = 100;
  const float OUTAKE_SPEED = 100;

  const int AutonWaitTimeCycle = 100;
  const int CYCLE_SPEED_NORMAL = 10;

  int CURRENT_AUTON = 0; // 0=Red Right, 1=Red Left, 2=Blue Right, 3=Blue Left
  bool autonConfirmed = false;

  bool isBlueTeam;
  bool antiTipEnabled = true;
  bool autoHopperEnabled = true;

  float wheelCircumfrence = 31.9185813596f;
  float turningDiameter = 30;

  void setSideSpeeds(int lSpeed, int rSpeed) {
    if ((lSpeed - s_lastL) > MOTOR_ACCEL_LIMIT) {
      lSpeed = s_lastL + MOTOR_ACCEL_LIMIT;
    }
    if ((lSpeed - s_lastL) < -MOTOR_ACCEL_LIMIT) {
      lSpeed = s_lastL - MOTOR_ACCEL_LIMIT;
    }
    if ((rSpeed - s_lastR) > MOTOR_ACCEL_LIMIT) {
      rSpeed = s_lastR + MOTOR_ACCEL_LIMIT;
    }
    if ((rSpeed - s_lastR) < -MOTOR_ACCEL_LIMIT) {
      rSpeed = s_lastR - MOTOR_ACCEL_LIMIT;
    }
    s_lastL = lSpeed;
    s_lastR = rSpeed;

    if (lSpeed == 0) {
      lDrive.stop(brakeType::brake);
    } else {
      lDrive.spin(directionType::fwd, lSpeed, velocityUnits::pct);
    }
    if (rSpeed == 0) {
      rDrive.stop(brakeType::brake);
    } else {
      rDrive.spin(directionType::fwd, rSpeed, velocityUnits::pct);
    }
  }
  void WaitForBumper(int amount, int maxTimeMsec)
  {
    int doneAmount = 0;
    int doneTime = 0;
    while(doneAmount < amount)
    {
      if(ballBumper.pressing())
      {
        doneAmount++;
        while(ballBumper.pressing())
        {
          wait(30, timeUnits::msec);
          doneTime += 30;
          if(doneTime >= maxTimeMsec)
            break;
        }
      }
      wait(100, timeUnits::msec);
      doneTime += 100;
        if(doneTime >= maxTimeMsec)
            break;
    }
  }
  void DriveDistance(float distance){
    lDrive1.resetPosition();
    rDrive1.resetPosition();

    float rotationGoal = (360 * distance) / wheelCircumfrence;

    float maxSpeed = 0;
    const float Kp = 0.25;
    const float Ki = 0;
    const float Kd = 0;
    const float deadZone = 1;
    
    float lError = 0;
    float rError = 0;
    float lIntegral = 0;
    float rIntegral = 0;

    float lDerivative = 0;
    float rDerivative = 0;

    float lLastError = 0;
    float rLastError = 0;

    float lMotorSpeed = 0;
    float rMotorSpeed = 0;

    float doneTime = 0;
    while(true){
      lError = rotationGoal - lDrive1.rotation(deg);
      rError = rotationGoal - rDrive1.rotation(deg);

      lIntegral += lError;
      rIntegral += rError;
      if(lError > 200 || lError < -200)
      {
        lIntegral = 0;
      }
      if(rError > 200 || rError < -200)
      {
        rIntegral = 0;
      }

      lDerivative = lError - lLastError;
      rDerivative = rError - rLastError;

      lLastError = lError;
      rLastError = rError;

      lMotorSpeed = Kp * lError + Ki * lIntegral + Kd * lDerivative;
      rMotorSpeed = Kp * rError + Ki * rIntegral + Kd * rDerivative;
      if(doneTime < 500)
      {
        if(distance < 0)
          maxSpeed = -(doneTime / 5);
        else
          maxSpeed = doneTime / 5;
      }
      else
      {
        
        if(lMotorSpeed < deadZone && lMotorSpeed > -deadZone && rMotorSpeed < deadZone && rMotorSpeed > -deadZone)
        {
          rDrive.spin(fwd, 0, pct);
          lDrive.spin(fwd, 0, pct);
          break;
        }
        
      }
      if(distance > 0)
      {
        lMotorSpeed = maxSpeed < lMotorSpeed ? maxSpeed : lMotorSpeed;
        rMotorSpeed = maxSpeed < rMotorSpeed ? maxSpeed : rMotorSpeed;
      }
      else
      {
        lMotorSpeed = maxSpeed > lMotorSpeed ? maxSpeed : lMotorSpeed;
        rMotorSpeed = maxSpeed > rMotorSpeed ? maxSpeed : rMotorSpeed;
      }


      lDrive.spin(fwd, lMotorSpeed, pct);
      rDrive.spin(fwd, rMotorSpeed, pct);

      wait(15, msec);
      doneTime += 15;
    }
  }
  void RotateDegrees(int degrees)
  {
    intertialSensor.resetHeading();

    float maxSpeed = 80;

    const float Kp = 1;
    const float Ki = 0;
    const float Kd = 0;
    const float deadZone = 1;
    
    float error = 0;
    float integral = 0;

    float derivative = 0;

    float lastError = 0;

    float motorSpeed = 0;

    float doneTime = 0;

    while(true)
    {
      float head = intertialSensor.heading();
      if(head > 180)
        head -= 360;
      error = degrees - head; 
      integral += error;
      if(error > 40 || error < -40)
      {
        integral = 0;
        derivative = 0;
      }
      else
      {
        
        if(motorSpeed < deadZone && motorSpeed > -deadZone)
        {
          rDrive.spin(fwd, 0, pct);
          lDrive.spin(fwd, 0, pct);
          break;
        }
        
      }
      derivative = error - lastError;

      lastError = error;

      motorSpeed = Kp * error + Ki * integral + Kd * derivative;
      
      if(doneTime < 200)
      {
        if(degrees < 0)
          maxSpeed = -(doneTime / 2);
        else
          maxSpeed = doneTime / 2;
      }
      
       if(degrees > 0)
      {
        motorSpeed = maxSpeed < motorSpeed ? maxSpeed : motorSpeed;
      }
      else
      {
        motorSpeed = maxSpeed > motorSpeed ? maxSpeed : motorSpeed;
      }
      lDrive.spin(fwd, motorSpeed, pct);
      rDrive.spin(fwd, -motorSpeed, pct);
      if(int(doneTime) % 60 == 0)
      {
        Brain.Screen.clearScreen();
        Brain.Screen.setCursor(4, 4);
        Brain.Screen.print(head);
        Brain.Screen.setCursor(6, 6);
        Brain.Screen.print(error);


      }
      wait(15, msec);
      doneTime += 15;
    }
  }
  void ToggleScorer(bool on, bool rev) {
    lowerOutakeMotor.spin(fwd, on ? -OUTAKE_SPEED : 0, pct);
    upperOutakeMotor.spin(fwd, on ? rev ? OUTAKE_SPEED : -OUTAKE_SPEED : 0, pct);
  }
  void ToggleBackOutake(bool on) {
    lowerOutakeMotor.spin(fwd, on ? -OUTAKE_SPEED : 0, pct);
    upperOutakeMotor.spin(fwd, on ? OUTAKE_SPEED : 0, pct);
  }
  void ToggleIntakeRoller(bool on, bool rev) {
    intakeRollers.spin(fwd, on ? rev ? ROLLER_SPEED : -ROLLER_SPEED : 0, pct);
  }
  void RightCorner() {
    upperOutakeMotor.spinFor(fwd, 90, deg, 100, velocityUnits::pct);
    //mainDrive.driveFor(fwd, -41, distanceUnits::cm);
    DriveDistance(-47);
    //mainDrive.turnFor(-23, deg);
    RotateDegrees(-46);
    ToggleIntakeRoller(true, false);
    //mainDrive.driveFor(fwd, 50, distanceUnits::cm);
    DriveDistance(59);
    ToggleScorer(true, false);
    WaitForBumper(3, 4000);
    ToggleIntakeRoller(false, false);
    WaitForBumper(1, 1000);
    ToggleScorer(false, false);
    //mainDrive.driveFor(fwd, -50, distanceUnits::cm);
    DriveDistance(-50);
  }
  void RightCornerAndCenter() {
    RightCorner();
    
    ToggleIntakeRoller(true, false);
    ToggleScorer(true, false);
    //mainDrive.turnFor(64, deg);
    RotateDegrees(135);
    //mainDrive.driveFor(fwd, 100, distanceUnits::cm);
    DriveDistance(106);
    ToggleScorer(false, false);
    //mainDrive.turnFor(-43, deg);
    RotateDegrees(-90);
    //mainDrive.driveFor(fwd, 25, distanceUnits::cm);
    DriveDistance(16);
    ToggleScorer(true, false);
    wait(1.5, sec);
    ToggleIntakeRoller(false, false);
    //mainDrive.driveFor(fwd, -18.5, distanceUnits::cm);
    DriveDistance(-18.5);
    ToggleScorer(false, false);
    
  }
  void LeftCorner()
  {
    upperOutakeMotor.spinFor(fwd, 90, deg, 100, velocityUnits::pct);
    mainDrive.driveFor(fwd, -41, distanceUnits::cm);
    mainDrive.turnFor(24, deg);
    ToggleIntakeRoller(true, false);
    mainDrive.driveFor(fwd, 50, distanceUnits::cm);
    ToggleScorer(true, false);
    WaitForBumper(3, 4000);
    ToggleIntakeRoller(false, false);
    WaitForBumper(1, 2000);
    ToggleScorer(false, false);
    mainDrive.driveFor(fwd, -50, distanceUnits::cm);
  }
  void LeftCornerAndCenter()
  {
    LeftCorner();
    
    ToggleIntakeRoller(true, false);
    ToggleScorer(true, false);
    mainDrive.turnFor(-67.5, deg);
    mainDrive.driveFor(fwd, 100, distanceUnits::cm);
    ToggleScorer(false, false);
    mainDrive.turnFor(-2.5, deg);
    mainDrive.driveFor(fwd, 25, distanceUnits::cm);
    ToggleScorer(true, false);
    wait(1.5, sec);
    ToggleIntakeRoller(false, false);
    mainDrive.driveFor(fwd, -18.5, distanceUnits::cm);
    ToggleScorer(false, false);
  }
  bool DrawButton(int xPos, int yPos, int width, int height, color Color) {
    Brain.Screen.drawRectangle(xPos, yPos, width, height, Color);
    int pressX = Brain.Screen.xPosition();
    int pressY = Brain.Screen.yPosition();

    if (Brain.Screen.pressing()) {
      if (pressX > xPos && pressX < xPos + width && pressY > yPos &&
          pressY < yPos + height) {
        return true;
      }
    }
    return false;
  }
  void UpdateSelectionScreen()
  {
    Controller1.Screen.clearScreen();
    if (!Competition.isEnabled()) {
      Controller1.Screen.setCursor(0, 0);
      Controller1.Screen.print("Auton(^v) Is ");
      Controller1.Screen.setCursor(2, 0);
      switch (CURRENT_AUTON) {
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
    else if (Competition.isDriverControl()) {
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
      if(obj.height > 30 && obj.width > 70)
      {
          if(obj.centerX > 158)
          {
            setSideSpeeds(100, 100 + (158 - obj.centerX) * 0.5);
          }
          else if(obj.centerX < 157)
          {
            setSideSpeeds(100 - (157 - obj.centerX) * 0.5, 100);
          }
          else {
            setSideSpeeds(100, 100);
          }
      }
      else{
      setSideSpeeds(0, 0);

      
      }
  }
  void drivercontrol() {

    wait(2, sec);
    RotateDegrees(90);
    RotateDegrees(-90);
    Brain.Screen.clearScreen();
    UpdateSelectionScreen();  
    
    while (1) {
      //Hopper Management
      if (autoHopperEnabled) {
        if (opticalSensor.isNearObject()) {
          if(!isBlueTeam)
          {
            if(opticalSensor.color() == color().blue)
            {
              lowerOutakeMotor.spin(fwd, -100, pct);
              upperOutakeMotor.spin(fwd, 100, pct);
              wait(400, msec);
            }
          }
          else
          {
            if(opticalSensor.color() == color().red)
            {
              lowerOutakeMotor.spin(fwd, -100, pct);
              upperOutakeMotor.spin(fwd, 100, pct);
              wait(400, msec);
            }
          }
        }
      }
      //AntiTip Management
      if (intertialSensor.pitch() > 15 && antiTipEnabled) {
        mainDrive.setDriveVelocity(70, pct);
        mainDrive.driveFor(directionType::fwd, -30, distanceUnits::cm);
        continue;
      }
     

      // Set Roller speeds
      lRoller.spin(fwd,
                  Controller1.ButtonR2.pressing()
                      ? -ROLLER_SPEED : Controller1.ButtonR1.pressing() ? ROLLER_SPEED
                      : 0,
                  pct);
      rRoller.spin(fwd,
                  Controller1.ButtonR2.pressing()
                      ? -ROLLER_SPEED : Controller1.ButtonR1.pressing() ? ROLLER_SPEED
                      : 0,
                  pct);

      // Set Outake Motor speeds
      lowerOutakeMotor.spin(fwd,
                        Controller1.ButtonL2.pressing()
                            ? -OUTAKE_SPEED / 3
                            : Controller1.ButtonL1.pressing() ? OUTAKE_SPEED / 3 : 0,
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
     
       if(Controller1.ButtonDown.pressing())
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
  void autonomous() {
    mainDrive.setDriveVelocity(AUTON_DRIVE_SPEED, pct);
    mainDrive.setTurnVelocity(AUTON_ROTATE_SPEED, pct);

    wait(3, sec);
    //DriveDistance(60.96);
    //RotateDegrees(-44);
  RightCornerAndCenter();
    return;
    switch (CURRENT_AUTON) {
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

  void ToggleAutoHopper() {
    if (Competition.isAutonomous())
      return;
    autoHopperEnabled = !autoHopperEnabled;
    UpdateSelectionScreen();
  }
  void ToggleAntiTip() {
    if (Competition.isAutonomous())
      return;
    antiTipEnabled = !antiTipEnabled;
    UpdateSelectionScreen();
  }
  void ToggleTeam() {
    isBlueTeam = !isBlueTeam;
    UpdateSelectionScreen();
  }
  void SwitchAuton() {
    CURRENT_AUTON++;
    if (CURRENT_AUTON == 4)
      CURRENT_AUTON = 0;
    UpdateSelectionScreen();
  }
  void RecalibrateIntertial() { intertialSensor.calibrate(); }
  void pre_auton() {
    Controller1.ButtonY.pressed(ToggleAntiTip);
    Controller1.ButtonB.pressed(ToggleAutoHopper);
    Controller1.ButtonLeft.pressed(ToggleTeam);
    Controller1.ButtonRight.pressed(ToggleTeam);
    Controller1.ButtonUp.pressed(SwitchAuton);
    Controller1.ButtonDown.pressed(RecalibrateIntertial);
    UpdateSelectionScreen();
  }
  void Initialise()
  {
    intertialSensor.calibrate();
  }
  int main() {
    vexcodeInit();
    Initialise();
    
    Competition.autonomous(autonomous);
    Competition.drivercontrol(drivercontrol);
    pre_auton();
    UpdateSelectionScreen();
    while (true) {
      task::sleep(CYCLE_SPEED_NORMAL);
    }
  }