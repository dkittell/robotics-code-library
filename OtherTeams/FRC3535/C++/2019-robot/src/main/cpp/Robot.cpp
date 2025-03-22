/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma region Includes
#include "Robot.h"
#include <cameraserver/CameraServer.h>
#include <ctre/Phoenix.h>
#include <frc/DoubleSolenoid.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/Joystick.h>
#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Solenoid.h>
#include <frc/SolenoidBase.h>
#include <frc/TimedRobot.h>
#include <frc/WPILib.h>
#include <iostream>
#include <string>
#include <wpi/raw_ostream.h>
using namespace frc;
#pragma endregion Includes

#pragma region Custom Max Speed
Preferences *prefs;
double js1lMax;
double js1rMax;
double js2lMax;
double js2rMax;
#pragma endregion Custom Max Speed

int autonCounter = 0;
int autonState = 0;

#pragma region Timer Definition
frc::Timer MyTimer; // Note no 'p' preceeding - it's a real object - not a pointer. So there's no need to 'new' it either...
#pragma endregion Timer Definition

#pragma region Device Definition - Camera
cs::UsbCamera camera1;
cs::UsbCamera camera2;
cs::UsbCamera camera3;
#pragma endregion Device Definition - Camera

#pragma region Device Definition - Pneumatics
// If you need to test without the compressor set this variable to 1
int nTestWithoutCompressor = 0;

int nLiftSolenoidRL = 0;
int nLiftSolenoidRR = 1;
int nLiftSolenoidFL = 2;
int nLiftSolenoidFR = 3;
int nFrontHatchSolenoid = 4;
int nIntakeSolenoid0 = 5;
int nIntakeSolenoid1 = 6;

int nRearHatchSolenoid0 = 0; //
int nRearHatchSolenoid1 = 1; //
int nRearHatchSolenoid2 = 2; //
int nRearHatchSolenoid3 = 3; //

int nPCM1 = 0;
int nPCM2 = 1;

// Compressor Definitions - Start
frc::Compressor *compressor;

frc::Solenoid s_LiftSolenoid0{nPCM1, nLiftSolenoidRL};
frc::Solenoid s_LiftSolenoid1{nPCM1, nLiftSolenoidRR};
frc::Solenoid s_LiftSolenoid2{nPCM1, nLiftSolenoidFL};
frc::Solenoid s_LiftSolenoid3{nPCM1, nLiftSolenoidFR};

// Solenoid corresponds to a single solenoid.
frc::Solenoid s_FrontHatchPickup{nPCM1, nFrontHatchSolenoid}; // Front Hatch
// DoubleSolenoid corresponds to a double solenoid.
frc::DoubleSolenoid ds_BallIntakeExtend{nPCM1, nIntakeSolenoid0, nIntakeSolenoid1}; // Ball Intake Extend/Collapse

// 1 to extend, 3 to bring in
frc::DoubleSolenoid ds_RearHatchExtend{nPCM2, nRearHatchSolenoid1, nRearHatchSolenoid3}; // Start & Back
// 0 to release, 2 to grab
frc::DoubleSolenoid ds_RearHatchGrab{nPCM2, nRearHatchSolenoid0, nRearHatchSolenoid2}; // LT & RT

// Compressor Definitions - Stop
#pragma endregion Device Definition - Pneumatics

#pragma region Device Definition - Limit Switches
// Ball Intake - Ball Is In (True/False)
// int nBallIntakeLimitSwitch = 0;

// Ball Intake - Pulled In (True/False)
// Front Hatch can only be used when retracted = true
// int nBallIntakeLimitSwitchRetracted = 1;

// Arm Up/Down (True/False)
// Arm at lowest point (True/False)
int nArmLimitSwitchLow = 0;
// Arm at highest point (True/False)
int nArmLimitSwitchHigh = 1;

// Platform Arm Out (True/False)
// int nPlatformOutLimitSwitch = 4;
// Platform Arm Retracted (True/False)
// int nPlatformRetractedLimitSwitch = 5;

// Ball Intake - Ball Is In (True/False)
// frc::DigitalInput *ballintakelimitswitch;

// Ball Intake - Pulled In (True/False)
// Front Hatch can only be used when retracted = true
// frc::DigitalInput *ballintakelimitswitchRetracted;

// Arm Up/Down (True/False)
// Arm at lowest point (True/False)
frc::DigitalInput *armlimitswitchLow;
// Arm at highest point (True/False)
frc::DigitalInput *armlimitswitchHigh;

// Platform Arm Out (True/False)
// frc::DigitalInput *platformlimitswitch;
// Platform Arm Retracted (True/False)
// frc::DigitalInput *platformlimitswitchHigh;
#pragma endregion Device Definition - Limit Switches

#pragma region Device Definition - Encoder
// int nPWP;
// int nQP;
// int nLevel1Hatch = 1300;
// int nLevel2Hatch = 2600;
// int nLevel3Hatch = 3900;

// int nLevel1Cargo = 1400;
// int nLevel2Cargo = 2800;
// int nLevel3Cargo = 5600;

// int EncoderValue;
// int currentQP;
int currentPWP;
#pragma endregion Device Definition - Encoder

#pragma region Controller Definition
#pragma region Dead Band Variables JS1
double js1lSpeed = 0;  // Speed controls up & down
double js1rSpeed = 0;  // Speed controls up & down
double js1lRotate = 0; // Rotate controls left & right
double js1rRotate = 0; // Rotate controls left & right
double js1LTSpeed = 0;
double js1RTSpeed = 0;

// JS1 Left Deadband
double js1lDeadband_1 = 0.15;
double js1lDeadband_2 = 0.90;

// JS1 Right Deadband
double js1rDeadband_1 = 0.15;
double js1rDeadband_2 = 0.60;
#pragma endregion Dead Band Variables JS1

#pragma region Dead Band Variables JS2
double js2lSpeed = 0;  // Speed controls up & down
double js2rSpeed = 0;  // Speed controls up & down
double js2lRotate = 0; // Rotate controls left & right
double js2rRotate = 0; // Rotate controls left & right
double js2LTSpeed = 0;
double js2RTSpeed = 0;

// JS2 Left Deadband
double js2lDeadband_1 = 0.15;
double js2lDeadband_2 = 0.90;

// JS2 Right Deadband
double js2rDeadband_1 = 0.15;
double js2rDeadband_2 = 0.50;
#pragma endregion Dead Band Variables JS2

#pragma region Controller Button Variables
#pragma region Logitech Gamepad F310 Controller Layout - D Switch
// Logitech Gamepad F310 Controller Layout - D Switch
int btnD_X = 1;
int btnD_A = 2;
int btnD_B = 3;
int btnD_Y = 4;
int btnD_LB = 5;
int btnD_RB = 6;
int btnD_LT = 7;
int btnD_RT = 8;
int btnD_Back = 9;
int btnD_Start = 10;
int btnD_LToggle = 11;
int btnD_RToggle = 12;
int axisD_lUpDown = 1;
int axisD_lLeftRight = 0;
int axisD_rUpDown = 3;
int axisD_rLeftRight = 2;
#pragma endregion Logitech Gamepad F310 Controller Layout - D Switch

#pragma region Logitech Gamepad F310 Controller Layout - X Switch
// Logitech Gamepad F310 Controller Layout - X Switch
int btnX_X = 3;
int btnX_A = 1;
int btnX_B = 2;
int btnX_Y = 4;
int btnX_LB = 5;
int btnX_RB = 6;
int btnX_Back = 7;
int btnX_Start = 8;
int btnX_LToggle = 9;
int btnX_RToggle = 10;
int axisX_lUpDown = 1;
int axisX_lLeftRight = 0;
int axisX_rUpDown = 5;
int axisX_rLeftRight = 4;
#pragma endregion Logitech Gamepad F310 Controller Layout - X Switch

#pragma region Logitech Attack 3 J - UJ18
int btnA_Trigger = 1;
int btnA_2 = 2;
int btnA_3 = 3;
int btnA_4 = 4;
int btnA_5 = 5;
int btnA_6 = 6;
int btnA_7 = 7;
int btnA_8 = 8;
int btnA_9 = 9;
int btnA_10 = 10;
int btnA_11 = 11;
int axisX_UpDown = 0;
int axisX_LeftRight = 1;
#pragma endregion Logitech Attack 3 J - UJ18
#pragma endregion Controller Button Variables

#pragma region Controller Joystick Definitions
frc::Joystick js1{0}; // Driver's Controller
frc::Joystick js2{1}; // Operator's Controller
#pragma endregion Controller Joystick Definitions
#pragma endregion Controller Definition

#pragma region Device Definition - Drive Motors
// frc::Victor pwmPlatformExtend{1};
// VictorSPX PlatformDriveMotor{12}; // Drive

#pragma region Device Definition - Arcade - 2 Drive Motors
// ctre::phoenix::motorcontrol::can::WPI_TalonSRX m_leftMotor{9};
// ctre::phoenix::motorcontrol::can::WPI_TalonSRX m_rightMotor{7};
// frc::DifferentialDrive m_robotDrive{m_leftMotor, m_rightMotor};
#pragma endregion Device Definition - Arcade - 2 Drive Motors

#pragma region Device Definition - Arcade - 4 Drive Motors
// TalonSRX Definitions - Start
// TalonSRX m_rearRight{7};  // Right Rear Drive Motor
// TalonSRX m_frontRight{8};  // Right Front Drive Motor
// TalonSRX m_rearLeft{9};  // Left Rear Drive Motor
// TalonSRX m_frontLeft{10}; // Left Front Drive Motor

// ctre::phoenix::motorcontrol::can::WPI_TalonSRX m_frontLeft{10}; // Left Front Drive Motor
// ctre::phoenix::motorcontrol::can::WPI_TalonSRX m_rearLeft{9};   // Left Rear Drive Motor
// // frc::Talon m_frontLeft{10}; // Left Front Drive Motor
// // frc::Talon m_rearLeft{9};   // Left Rear Drive Motor
// frc::SpeedControllerGroup m_left{m_frontLeft, m_rearLeft};

// ctre::phoenix::motorcontrol::can::WPI_TalonSRX m_frontRight{8}; // Right Front Drive Motor
// ctre::phoenix::motorcontrol::can::WPI_TalonSRX m_rearRight{7};  // Right Rear Drive Motor
// // frc::Talon m_frontRight{8}; // Right Front Drive Motor
// // frc::Talon m_rearRight{7}; // Right Rear Drive Motor
// frc::SpeedControllerGroup m_right{m_frontRight, m_rearRight};

// frc::DifferentialDrive m_drive{m_left, m_right};
#pragma endregion Device Definition - Arcade - 4 Drive Motors

#pragma region Device Definition - CTRE Arcade - 4 Drive Motors
WPI_TalonSRX *m_frontRight = new WPI_TalonSRX(8); // Right Front Drive Motor
WPI_TalonSRX *m_rearRight = new WPI_TalonSRX(7);  // Right Rear Drive Motor
WPI_TalonSRX *m_frontLeft = new WPI_TalonSRX(10); // Left Front Drive Motor
WPI_TalonSRX *m_rearLeft = new WPI_TalonSRX(9);   // Left Rear Drive Motor

DifferentialDrive *_diffDrive = new DifferentialDrive(*m_frontLeft, *m_frontRight);

Faults _faults_L;
Faults _faults_R;
#pragma endregion Device Definition - CTRE Arcade - 4 Drive Motors

#pragma endregion Device Definition - Drive Motors

#pragma region Device Definition - Arm
TalonSRX ArmRMotor{16}; // Right Arm Motor
TalonSRX ArmLMotor{15}; // Left Arm Motor
#pragma endregion Device Definition - Arm

#pragma region Device Definition - Ball Intake
VictorSPX BallIntakeMotor{11}; // Ball Intake Motor
#pragma endregion Device Definition - Ball Intake

#pragma region Functions

#pragma region CTRE Arcade Drive
void CTREArcadeDrive(double lSpeed, double rSpeed)
{
    std::stringstream work;

    /* drive robot */
    _diffDrive->ArcadeDrive(lSpeed, rSpeed, false);

    /* -------- [2] Make sure Gamepad Forward is positive for FORWARD, and GZ is positive for RIGHT */
    work << " GF:" << lSpeed << " GT:" << rSpeed;

    /* get sensor values */
    double leftPos = m_frontLeft->GetSelectedSensorPosition(0);
    double rghtPos = m_frontRight->GetSelectedSensorPosition(0);
    double leftVelUnitsPer100ms = m_frontLeft->GetSelectedSensorVelocity(0);
    double rghtVelUnitsPer100ms = m_frontRight->GetSelectedSensorVelocity(0);

    work << " L:" << leftPos << " R:" << rghtPos;
    work << " L:" << leftVelUnitsPer100ms << " R:" << rghtVelUnitsPer100ms;

    /* get sensor values */
    double leftPosRear = m_rearLeft->GetSelectedSensorPosition(0);
    double rghtPosRear = m_rearRight->GetSelectedSensorPosition(0);
    double leftVelUnitsPer100msRear = m_rearLeft->GetSelectedSensorVelocity(0);
    double rghtVelUnitsPer100msRear = m_rearRight->GetSelectedSensorVelocity(0);

    work << " L:" << leftPosRear << " R:" << rghtPosRear;
    work << " L:" << leftVelUnitsPer100msRear << " R:" << rghtVelUnitsPer100msRear;

    /* drive motor at least 25%, Talons will auto-detect if sensor is out of phase */
    m_frontLeft->GetFaults(_faults_L);
    m_frontRight->GetFaults(_faults_R);

    if (_faults_L.SensorOutOfPhase)
    {
        work << " L sensor is out of phase";
    }
    if (_faults_R.SensorOutOfPhase)
    {
        work << " R sensor is out of phase";
    }

    /* print to console */
    std::cout << work.str() << std::endl;
}
#pragma endregion CTRE Arcade Drive

#pragma region True Auton
void auton1()
{
    switch (autonState)
    {
    case 0:
        // Drive Forward for one second
        CTREArcadeDrive(.25, .25);
        if (autonCounter >= 50)
        {
            CTREArcadeDrive(0, 0);
            autonState = 10;
        }
        break;
    case 10:

        // Turn 90 degrees
        CTREArcadeDrive(0, .25);
        if (autonCounter >= 100)
        {
            CTREArcadeDrive(0, 0);
            autonState = 20;
        }
        break;
    case 20:
        // Drive Forward for one second
        CTREArcadeDrive(.25, .25);
        if (autonCounter >= 150)
        {
            CTREArcadeDrive(0, 0);
            autonState = 30;
        }
        break;
    case 30:
        s_FrontHatchPickup.Set(true);
        // backup robot
        CTREArcadeDrive(-.25, -.25);
        if (autonCounter >= 200)
        {
            CTREArcadeDrive(0, 0);
            autonState = 40;
        }
        break;
    case 40:
        // Stop
        break;
    }
    autonCounter++;
}
void auton2()
{
    switch (autonState)
    {
    case 0:
        // Drive Forward for one second
        CTREArcadeDrive(.25, .25);
        if (autonCounter >= 50)
        {
            CTREArcadeDrive(0, 0);
            autonState = 30;
        }
        break;

    case 30:
        s_FrontHatchPickup.Set(true);
        // backup robot
        CTREArcadeDrive(-.25, -.25);
        if (autonCounter >= 200)
        {
            CTREArcadeDrive(0, 0);
            autonState = 40;
        }
        break;
    case 40:
        // Stop
        break;
    }
    autonCounter++;
}
void auton3()
{
    switch (autonState)
    {
    case 0:
        // Drive Forward for one second
        CTREArcadeDrive(.25, .25);
        if (autonCounter >= 50)
        {
            CTREArcadeDrive(0, 0);
            autonState = 10;
        }
        break;
    case 10:

        // Turn 90 degrees
        CTREArcadeDrive(.25, 0);
        if (autonCounter >= 100)
        {
            CTREArcadeDrive(0, 0);
            autonState = 20;
        }
        break;
    case 20:
        // Drive Forward for one second
        CTREArcadeDrive(.25, .25);
        if (autonCounter >= 150)
        {
            CTREArcadeDrive(0, 0);
            autonState = 30;
        }
        break;
    case 30:
        s_FrontHatchPickup.Set(true);
        // backup robot
        CTREArcadeDrive(-.25, -.25);
        if (autonCounter >= 200)
        {
            CTREArcadeDrive(0, 0);
            autonState = 40;
        }
        break;
    case 40:
        // Stop
        break;
    }
    autonCounter++;
}
#pragma endregion True Auton

#pragma region Tank Drive
// void Drive(double lSpeed, double rSpeed, double rotation)
// {
//   m_rearLeft.SetInverted(1);
//   m_frontLeft.SetInverted(1);
//   m_rearRight.SetInverted(1);

//   // Right Drive Motors - Start
//   // Values from -1 to 1 are accepted.
//   m_rearRight.Set(ControlMode::PercentOutput, rSpeed + rotation);
//   m_frontRight.Set(ControlMode::PercentOutput, rSpeed + rotation);
//   // Right Drive Motors - Stop

//   // Left Drive Motors - Start
//   // Values from -1 to 1 are accepted.
//   m_rearLeft.Set(ControlMode::PercentOutput, lSpeed + rotation);
//   m_frontLeft.Set(ControlMode::PercentOutput, lSpeed + rotation);
//   // Left Drive Motors - Stop

// }
#pragma endregion Tank Drive

#pragma region Arm Limit Switch
void ArmLimitSwitch(double speed)
{
    ArmRMotor.SetInverted(1);

    if (armlimitswitchLow->Get() == 1 && speed < 0) // If the limit switch is not tripped and the speed is negative let the motors go
        //  if (speed < 0) // If the limit switch is not tripped and the speed is negative let the motors go
    {
        currentPWP = ArmLMotor.GetSelectedSensorPosition(0);
        std::cout << "Axis currentPWP " << currentPWP << std::endl;
        frc::SmartDashboard::PutNumber("Current Encoder Value", ArmLMotor.GetSelectedSensorPosition(0));

        // Values from -1 to 1 are accepted.
        ArmLMotor.Set(ControlMode::PercentOutput, speed);
        ArmRMotor.Set(ControlMode::PercentOutput, speed);
    }
    else if (armlimitswitchLow->Get() == 0)
    {
        ArmLMotor.GetSensorCollection().SetQuadraturePosition(0, 0);
        ArmLMotor.GetSensorCollection().SetPulseWidthPosition(0, 0);
    }

    if (armlimitswitchHigh->Get() == 1 && speed > 0) // If the limit switch is not tripped and the speed is positive let the motors go
        // if (speed > 0) // If the limit switch is not tripped and the speed is positive let the motors go
    {
        currentPWP = ArmLMotor.GetSelectedSensorPosition(0);
        std::cout << "Axis currentPWP " << currentPWP << std::endl;
        frc::SmartDashboard::PutNumber("Current Encoder Value", ArmLMotor.GetSelectedSensorPosition(0));

        // Values from -1 to 1 are accepted.
        ArmLMotor.Set(ControlMode::PercentOutput, speed);
        ArmRMotor.Set(ControlMode::PercentOutput, speed);
    }
}
#pragma endregion Arm Limit Switch

#pragma region Arm Encoders
// void Arm(double speed, double stopvalue)
// {

//     ArmRMotor.SetInverted(1);

//     std::cout << "Check Positions After Setting" << std::endl;
//     // int currentPWP = ArmLMotor.GetSensorCollection().GetPulseWidthPosition();
//     // int currentQP = ArmLMotor.GetSensorCollection().GetQuadraturePosition();
//     int currentPWP = ArmLMotor.GetSelectedSensorPosition(0);

//     // std::cout << "currentPWP " << currentPWP << " currentQP " << currentQP << std::endl;
//     std::cout << "currentPWP " << currentPWP << " currentPWP " << currentPWP << std::endl;
//     // nPWP = currentPWP;
//     // nQP = currentQP;
//     // std::cout << "nPWP " << nPWP << " nQP " << nQP << std::endl;

//     // if (currentQP < stopvalue && armlimitswitchLow == false && armlimitswitchHigh == false)
//     if ((currentPWP < stopvalue) && (armlimitswitchHigh == false))
//     {
//         // Returns Encoder Values
//         // EncoderValue = ArmLMotor.GetSensorCollection().GetQuadraturePosition();
//         // int EncoderValue = ArmLMotor.GetSensorCollection().GetPulseWidthPosition();
//         // Sets Encoder Values
//         // ArmLMotor.GetSensorCollection().SetQuadraturePosition(0, 0);
//         // ArmLMotor.GetSensorCollection().SetPulseWidthPosition(0, 0);

//         // Values from -1 to 1 are accepted.
//         ArmLMotor.Set(ControlMode::PercentOutput, speed);
//         ArmRMotor.Set(ControlMode::PercentOutput, speed);

//         // std::cout << "Encoder Value: " << EncoderValue << std::endl;
//         // std::cout << "Speed Value: " << speed << std::endl;
//         // Values from -1 to 1 are accepted.
//         // ArmLMotor.Set(ControlMode::PercentOutput, 0);
//         // std::cout << "Encoder Value: " << EncoderValue << std::endl;
//         // std::cout << "Speed Value: " << speed << std::endl;
//     }
//     else
//     {
//         // Values from -1 to 1 are accepted.
//         ArmLMotor.Set(ControlMode::PercentOutput, 0.01);
//         ArmRMotor.Set(ControlMode::PercentOutput, 0.01);
//         // ArmLMotor.SetNeutralMode(1);
//     }
// }
#pragma endregion Arm Encoders

#pragma region Ball Intake
void BallIntake(double speed)
{
    // If limit switch is not tripped then keep running the motor to bring the ball in
    // if (speed > 0)
    // {
    //     // Values from -1 to 1 are accepted.
    //     BallIntakeMotor.Set(ControlMode::PercentOutput, speed);
    // }

    // // If limit switch is tripped then allow the ball to shoot out but no longer come in
    // if (speed < 0)
    // {
    // Values from -1 to 1 are accepted.
    BallIntakeMotor.Set(ControlMode::PercentOutput, speed);
    // }
}
#pragma endregion Ball Intake

#pragma region Platform Extend
// void PlatformExtend(double Speed)
// {
//     // Values from -1 to 1 are accepted.
//     pwmPlatformExtend.Set(Speed);
// }

// void PlatformDrive(double Speed)
// {
//     // Values from -1 to 1 are accepted.
//     PlatformDriveMotor.Set(ControlMode::PercentOutput, Speed);
// }
#pragma endregion Platform Extend
#pragma endregion Functions

#pragma region Smart Dashboard
float voltage;
#pragma endregion Smart Dashboard

// When Robot Is Turned On
#pragma region Robot Init
void Robot::RobotInit()
{
#pragma region Robot Init - Inner
    m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
    m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
    frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

#pragma region Camera
    // frc::CameraServer::GetInstance()->StartAutomaticCapture(0);
    // frc::CameraServer::GetInstance()->StartAutomaticCapture(1);
    // frc::CameraServer::GetInstance()->StartAutomaticCapture(2);

    camera1 = frc::CameraServer::GetInstance()->StartAutomaticCapture(0);
    camera2 = frc::CameraServer::GetInstance()->StartAutomaticCapture(1);
    camera3 = frc::CameraServer::GetInstance()->StartAutomaticCapture(2);

    frc::SmartDashboard::PutNumber("Camera 1 Brightness", camera1.GetBrightness());
    frc::SmartDashboard::PutNumber("Camera 2 Brightness", camera2.GetBrightness());
    frc::SmartDashboard::PutNumber("Camera 3 Brightness", camera3.GetBrightness());

    // camera1.SetBrightness(50);
    // camera2.SetBrightness(30);
    // camera3.SetBrightness(30);

    camera1.SetFPS(15);
    camera2.SetFPS(15);
    camera3.SetFPS(15);

    camera1.SetResolution(320, 240);
    camera2.SetResolution(320, 240);
    camera3.SetResolution(320, 240);
#pragma endregion Camera

#pragma region Arm
    ArmLMotor.GetSensorCollection().SetQuadraturePosition(0, 0);
    ArmLMotor.GetSensorCollection().SetPulseWidthPosition(0, 0);

    armlimitswitchLow = new frc::DigitalInput(0);
    armlimitswitchHigh = new frc::DigitalInput(1);

    // Reset to 0 When All the way down
    ArmLMotor.SetSelectedSensorPosition(0);
#pragma endregion Arm

#pragma region Pneumatics
    //id = wherever the solenoid is connected to the PCM
    compressor = new frc::Compressor(nPCM1);
    // compressor = new frc::Compressor(nPCM2);

    compressor->ClearAllPCMStickyFaults();
    compressor->SetClosedLoopControl(true);
    s_FrontHatchPickup.Set(true);
#pragma endregion Pneumatics

#pragma region CTRE Arcade Drive Motors
    /* factory default values */
    // m_frontRight->ConfigFactoryDefault();
    // m_rearRight->ConfigFactoryDefault();
    // m_frontLeft->ConfigFactoryDefault();
    // m_rearLeft->ConfigFactoryDefault();

    /* set up followers */
    m_rearRight->Follow(*m_frontRight);
    m_rearLeft->Follow(*m_frontLeft);

    /* [3] flip values so robot moves forward when stick-forward/LEDs-green */
    m_frontRight->SetInverted(false);
    m_rearRight->SetInverted(true);
    m_frontLeft->SetInverted(true);
    m_rearLeft->SetInverted(true);

    /* [4] adjust sensor phase so sensor moves
    	 * positive when Talon LEDs are green */
    m_frontRight->SetSensorPhase(true);
    m_frontLeft->SetSensorPhase(true);

    /*
    	* WPI drivetrain classes defaultly assume left and right are opposite. call
    	* this so we can apply + to both sides when moving forward. DO NOT CHANGE
    */
    _diffDrive->SetRightSideInverted(false);
#pragma endregion CTRE Arcade Drive Motors
#pragma endregion Robot Init - Inner
}
#pragma endregion Robot Init

// Every 20ms regardless of mode.
#pragma region Robot Periodic
void Robot::RobotPeriodic()
{
    frc::SmartDashboard::PutBoolean("Limit Switch Low", armlimitswitchLow->Get());
    frc::SmartDashboard::PutBoolean("Limit Switch High", armlimitswitchHigh->Get());
#pragma region Controller Values
#pragma region Get JS1 User Input
    js1lSpeed = js1.GetRawAxis(1); // Speed controls up & down
    // js1LTSpeed = js1.GetRawAxis(2); // L Trigger Axis (Controller X)
    // js1RTSpeed = js1.GetRawAxis(3); // R Trigger Axis (Controller X)
    // js1rSpeed = js1.GetRawAxis(4);  // Speed controls left & right (Controller X)
    js1rSpeed = -js1.GetRawAxis(2); // Speed controls left & right (Controller D)
    js1lRotate = js1.GetRawAxis(0); // Rotate controls left & right
    js1rRotate = js1.GetRawAxis(2); // Rotate controls left & right

    frc::SmartDashboard::PutNumber("JS1 Left Speed", js1lSpeed);
    frc::SmartDashboard::PutNumber("JS1 Right Speed", js1rSpeed);
#pragma endregion Get JS1 User Input

#pragma region Get JS2 User Input
    js2lSpeed = js2.GetRawAxis(1); // Speed controls up & down
    // js2LTSpeed = js2.GetRawAxis(2); // L Trigger Axis (Controller X)
    // js2RTSpeed = js2.GetRawAxis(3); // R Trigger Axis (Controller X)
    // js2rSpeed = js2.GetRawAxis(4);  // Speed controls left & right (Controller X)
    js2rSpeed = -js2.GetRawAxis(3); // Speed controls Up & Down (Controller D)
    js2lRotate = js2.GetRawAxis(0); // Rotate controls left & right
    js2rRotate = js2.GetRawAxis(2); // Rotate controls left & right

    frc::SmartDashboard::PutNumber("JS2 Left Speed", js2lSpeed);
    frc::SmartDashboard::PutNumber("JS2 Right Speed", js2rSpeed);
#pragma endregion Get JS2 User Input
#pragma endregion Controller Values
}
#pragma endregion Robot Periodic

// When Auton Begins
#pragma region Autonomous Init
void Robot::AutonomousInit()
{
    m_autoSelected = m_chooser.GetSelected();
    // m_autoSelected = SmartDashboard::GetString("Auto Selector",
    // kAutoNameDefault);
    std::cout << "Auto selected: " << m_autoSelected << std::endl;

    if (m_autoSelected == kAutoNameCustom)
    {
        // Custom Auto goes here
    }
    else
    {
#pragma region Autonomous Init - Inner
#pragma region Dead Band Variables JS1
        js1lSpeed = 0;  // Speed controls up & down
        js1rSpeed = 0;  // Speed controls up & down
        js1lRotate = 0; // Rotate controls left & right
        js1rRotate = 0; // Rotate controls left & right
        js1LTSpeed = 0;
        js1RTSpeed = 0;
#pragma endregion Dead Band Variables JS1

#pragma region Dead Band Variables JS2
        js2lSpeed = 0;  // Speed controls up & down
        js2rSpeed = 0;  // Speed controls up & down
        js2lRotate = 0; // Rotate controls left & right
        js2rRotate = 0; // Rotate controls left & right
        js2LTSpeed = 0;
        js2RTSpeed = 0;
#pragma endregion Dead Band Variables JS2

        s_FrontHatchPickup.Set(true);

        if (nTestWithoutCompressor == 1)
        {
            // FOR TESTING - START WITHOUT COMPRESSOR - START
            compressor->SetClosedLoopControl(false);
            // FOR TESTING - START WITHOUT COMPRESSOR - STOP
        }
        else
        {
            compressor->SetClosedLoopControl(true);
        }

#pragma region Set Motor Values To Zero
        // Values from -1 to 1 are accepted.
        ArmRMotor.Set(ControlMode::PercentOutput, 0);
        ArmLMotor.Set(ControlMode::PercentOutput, 0);
        BallIntakeMotor.Set(ControlMode::PercentOutput, 0);
// PlatformDriveMotor.Set(ControlMode::PercentOutput, 0);
// pwmPlatformExtend.Set(0);
#pragma endregion Set Motor Values To Zero

#pragma region Set Encoder Values To Zero
// ArmRMotor.GetSensorCollection().SetQuadraturePosition(0, 0);
// ArmRMotor.GetSensorCollection().SetPulseWidthPosition(0, 0);
// ArmLMotor.GetSensorCollection().SetQuadraturePosition(0, 0);
// ArmLMotor.GetSensorCollection().SetPulseWidthPosition(0, 0);
#pragma endregion Set Encoder Values To Zero

        // m_frontRight.Set(ControlMode::PercentOutput, 0); // Right Front Drive Motor
        // m_rearRight.Set(ControlMode::PercentOutput, 0);  // Right Rear Drive Motor
        // m_frontLeft.Set(ControlMode::PercentOutput, 0);  // Left Front Drive Motor
        // m_rearLeft.Set(ControlMode::PercentOutput, 0);   // Left Rear Drive Motor

#pragma region CTRE Arcade Drive
        m_frontRight->Set(ControlMode::PercentOutput, 0); // Right Front Drive Motor
        m_rearRight->Set(ControlMode::PercentOutput, 0);  // Right Rear Drive Motor
        m_frontLeft->Set(ControlMode::PercentOutput, 0);  // Left Front Drive Motor
        m_rearLeft->Set(ControlMode::PercentOutput, 0);   // Left Rear Drive Motor
#pragma endregion CTRE Arcade Drive

#pragma endregion Autonomous Init - Inner
    }
}
#pragma endregion Autonomous Init

// Every 20ms in auton
#pragma region Autonomous Periodic
void Robot::AutonomousPeriodic()
{

#pragma region Timer Definition
    // Now let's use the timer (it's an object so we use '.')
    MyTimer.Reset();
    MyTimer.Start();
    double timenow = MyTimer.Get();
    std::cout << "Timer: " << timenow << std::endl;
#pragma endregion Timer Definition

    if (m_autoSelected == kAutoNameCustom)
    {
        // Custom Auto goes here
    }
    else
    {
#pragma region Autonomous Periodic - Inner

#pragma region Custom Max Speed
        prefs = Preferences::GetInstance();
        js1lMax = prefs->GetDouble("js1lMax", 1.0);
        js1rMax = prefs->GetDouble("js1rMax", 1.0);
        js2lMax = prefs->GetDouble("js2lMax", 1.0);
        js2rMax = prefs->GetDouble("js2rMax", 1.0);

        if (js1lDeadband_2 != js1lMax)
        {
            js1lDeadband_2 = js1lMax;
            std::cout << "js1lDeadband_2: " << js1lDeadband_2 << std::endl;
        }

        if (js1rDeadband_2 != js1rMax)
        {
            js1rDeadband_2 = js1rMax;
            std::cout << "js1rDeadband_2: " << js1rDeadband_2 << std::endl;
        }

        if (js2lDeadband_2 != js2lMax)
        {
            js2lDeadband_2 = js2lMax;
            std::cout << "js2lDeadband_2: " << js2lDeadband_2 << std::endl;
        }

        if (js2rDeadband_2 != js2rMax)
        {
            js2rDeadband_2 = js2rMax;
            std::cout << "js2rDeadband_2: " << js2rDeadband_2 << std::endl;
        }

//  std::cout << "js1lMax: " << js1lMax << std::endl;
//  std::cout << "js1rMax: " << js1rMax << std::endl;
//  std::cout << "js2lMax: " << js2lMax << std::endl;
//  std::cout << "js2rMax: " << js2rMax << std::endl;
#pragma endregion Custom Max Speed

#pragma region Smart Dashboard / Logs
        currentPWP = ArmLMotor.GetSelectedSensorPosition(0);
        std::cout << "Axis currentPWP " << currentPWP << std::endl;
        frc::SmartDashboard::PutNumber("Current Encoder Value", ArmLMotor.GetSelectedSensorPosition(0));
        DriverStation &ds = DriverStation::GetInstance();
        voltage = ds.GetBatteryVoltage();
        frc::SmartDashboard::PutNumber("Battery Voltage", voltage);
        bool bBatteryGood = true;
        if (voltage < 9.8)
        {
            bBatteryGood = false;
        }
        frc::SmartDashboard::PutBoolean("Battery Life", bBatteryGood);
        int station;
        station = ds.GetLocation();
        frc::SmartDashboard::PutNumber("Position", station);
        if (ds.GetAlliance() == 0.0)
        {
            frc::SmartDashboard::PutString("Alliance Color", "Red");
        }
        else
        {
            frc::SmartDashboard::PutString("Alliance Color", "Blue");
        }
        frc::SmartDashboard::PutBoolean("Limit Switch Low", armlimitswitchLow->Get());
        frc::SmartDashboard::PutBoolean("Limit Switch High", armlimitswitchHigh->Get());
        currentPWP = ArmLMotor.GetSelectedSensorPosition(0);
        std::cout << "Axis currentPWP " << currentPWP << std::endl;
        frc::SmartDashboard::PutNumber("Current Encoder Value", ArmLMotor.GetSelectedSensorPosition(0));
        // Compressor Stats
        bool enabled = compressor->Enabled();
        bool pressureSwitch = compressor->GetPressureSwitchValue();
        double CompressorCurrent = compressor->GetCompressorCurrent();
        std::cout << "Compressor Current: " << CompressorCurrent << std::endl;
        std::cout << "Pressure Switch: " << pressureSwitch << std::endl;
        frc::SmartDashboard::PutNumber("Compressor Current", CompressorCurrent);
        frc::SmartDashboard::PutNumber("Pressure Switch", pressureSwitch);
#pragma endregion Smart Dashboard / Logs

#pragma region Controller Values
#pragma region Get JS1 User Input
        js1lSpeed = js1.GetRawAxis(1); // Speed controls up & down
        // js1LTSpeed = js1.GetRawAxis(2); // L Trigger Axis (Controller X)
        // js1RTSpeed = js1.GetRawAxis(3); // R Trigger Axis (Controller X)
        // js1rSpeed = js1.GetRawAxis(4);  // Speed controls left & right (Controller X)
        js1rSpeed = -js1.GetRawAxis(2); // Speed controls left & right (Controller D)
        js1lRotate = js1.GetRawAxis(0); // Rotate controls left & right
        js1rRotate = js1.GetRawAxis(2); // Rotate controls left & right

        frc::SmartDashboard::PutNumber("JS1 Left Speed", js1lSpeed);
        frc::SmartDashboard::PutNumber("JS1 Right Speed", js1rSpeed);

#pragma region Dead Bands
        if ((js1lSpeed > js1lDeadband_1))
        {
            js1lSpeed = ((js1lSpeed - js1lDeadband_1) * js1lDeadband_2);
        }
        else if ((js1lSpeed < -js1lDeadband_1))
        {
            js1lSpeed = ((js1lSpeed + js1lDeadband_1) * js1lDeadband_2);
        }
        else
        {
            js1lSpeed = 0; // If between boundaries. Do nothing.
        }
        if ((js1rSpeed > js1rDeadband_1))
        {
            js1rSpeed = ((js1rSpeed - js1rDeadband_1) * js1rDeadband_2);
        }
        else if ((js1rSpeed < -js1rDeadband_1))
        {
            js1rSpeed = ((js1rSpeed + js1rDeadband_1) * js1rDeadband_2);
        }
        else
        {
            js1rSpeed = 0; // If between boundaries. Do nothing.
        }
#pragma endregion Dead Bands

#pragma region Lift Robot - Raise
        // js1.GetRawButtonPressed // Don't have to hold down the button
        if (js1.GetRawButton(btnD_Back)) // Raise The Robot Front
        {
            s_LiftSolenoid1.Set(true); // Front - Raise Robot
        }
        else
        {
            s_LiftSolenoid1.Set(false);
        }
        if (js1.GetRawButton(btnD_Start)) // Raise The Robot Rear
        {
            s_LiftSolenoid0.Set(true); // Rear - Raise Robot
        }
        else
        {
            s_LiftSolenoid0.Set(false);
        }
#pragma endregion Lift Robot - Raise

#pragma region Arcade Drive
        // m_drive.ArcadeDrive(-js1lSpeed, js1rSpeed);
        if (js1.GetRawButton(btnD_X)) // Slow forward/reverse
        {
            CTREArcadeDrive(js1lSpeed / 2, js1rSpeed);
        }
        else
        {
            CTREArcadeDrive(js1lSpeed, js1rSpeed);
        }
#pragma endregion Arcade Drive

#pragma region Hatch Pickup - Front
        if (js1.GetRawButtonPressed(btnD_RB))
        {
            // s_FrontHatchPickup.Set(true);

            if (s_FrontHatchPickup.Get())
            {
                s_FrontHatchPickup.Set(false);
            }
            else
            {
                s_FrontHatchPickup.Set(true);
            }
            // }
            // if (js1.GetRawButtonPressed(btnD_LB))
            // {
            //     s_FrontHatchPickup.Set(false);
        }
#pragma endregion Hatch Pickup - Front

#pragma endregion Get JS1 User Input

#pragma region Get JS2 User Input
        js2lSpeed = js2.GetRawAxis(1); // Speed controls up & down
        // js2LTSpeed = js2.GetRawAxis(2); // L Trigger Axis (Controller X)
        // js2RTSpeed = js2.GetRawAxis(3); // R Trigger Axis (Controller X)
        // js2rSpeed = js2.GetRawAxis(4);  // Speed controls left & right (Controller X)
        js2rSpeed = -js2.GetRawAxis(3); // Speed controls Up & Down (Controller D)
        js2lRotate = js2.GetRawAxis(0); // Rotate controls left & right
        js2rRotate = js2.GetRawAxis(2); // Rotate controls left & right

        frc::SmartDashboard::PutNumber("JS2 Left Speed", js2lSpeed);
        frc::SmartDashboard::PutNumber("JS2 Right Speed", js2rSpeed);
#pragma region Dead Bands
        if ((js2lSpeed > js2lDeadband_1))
        {
            js2lSpeed = ((js2lSpeed - js2lDeadband_1) * js2lDeadband_2);
        }
        else if ((js2lSpeed < -js2lDeadband_1))
        {
            js2lSpeed = ((js2lSpeed + js2lDeadband_1) * js2lDeadband_2);
        }
        else
        {
            js2lSpeed = 0; // If between boundaries. Do nothing.
        }
        if ((js2rSpeed > js2rDeadband_1))
        {
            js2rSpeed = ((js2rSpeed - js2rDeadband_1) * js2rDeadband_2);
        }
        else if ((js2rSpeed < -js2rDeadband_1))
        {
            js2rSpeed = ((js2rSpeed + js2rDeadband_1) * js2rDeadband_2);
        }
        else
        {
            js2rSpeed = 0; // If between boundaries. Do nothing.
        }
#pragma endregion Dead Bands

#pragma region Lift Robot - Lower
        if (js2.GetRawButton(btnD_Start)) // Lower Robot Rear
        {
            s_LiftSolenoid2.Set(true); // Rear Lower Robot
        }
        else
        {
            s_LiftSolenoid2.Set(false);
        }
        if (js2.GetRawButton(btnD_Back)) // Lower Robot Front
        {
            s_LiftSolenoid3.Set(true); // Front Lower Robot
        }
        else
        {
            s_LiftSolenoid3.Set(false);
        }
#pragma endregion Lift Robot - Lower

#pragma region Rear Hatch Extend Button Controls
        if (js2.GetRawButtonPressed(btnD_A)) // Bring In
        {
            ds_RearHatchExtend.Set(frc::DoubleSolenoid::kForward);
            std::cout << "Double Solenoid: " << ds_RearHatchExtend.Get() << std::endl;
        }
        else if (js2.GetRawButtonPressed(btnD_Y)) // Extend Out
        {
            ds_RearHatchExtend.Set(frc::DoubleSolenoid::kReverse);
            std::cout << "Double Solenoid: " << ds_RearHatchExtend.Get() << std::endl;
        }
        else
        {
            ds_RearHatchExtend.Set(frc::DoubleSolenoid::kOff);
            std::cout << "Double Solenoid: " << ds_RearHatchExtend.Get() << std::endl;
        }
#pragma endregion Rear Hatch Extend Button Controls
#pragma region Rear Hatch Grab Button Controls
        if (js2.GetRawButtonPressed(btnD_B)) // Release
        {
            ds_RearHatchGrab.Set(frc::DoubleSolenoid::kForward);
            std::cout << "Double Solenoid: " << ds_RearHatchGrab.Get() << std::endl;
        }
        else if (js2.GetRawButtonPressed(btnD_X)) // Pick Up
        {
            ds_RearHatchGrab.Set(frc::DoubleSolenoid::kReverse);
            std::cout << "Double Solenoid: " << ds_RearHatchGrab.Get() << std::endl;
        }
        else
        {
            ds_RearHatchGrab.Set(frc::DoubleSolenoid::kOff);
            std::cout << "Double Solenoid: " << ds_RearHatchGrab.Get() << std::endl;
        }
#pragma endregion Rear Hatch Grab Button Controls

#pragma region Arm - Manual - Analog
#pragma region Arm - Manual - Analog - Left
// std::cout << "js2lSpeed: " << js2lSpeed << std::endl;
// if (js2lSpeed > 0)
// {
//     std::cout << "Arm Up" << std::endl;
//     ArmLimitSwitch(js2lSpeed);
// }
// else if (js2lSpeed < 0)
// {
//     std::cout << "Arm Down" << std::endl;
//     ArmLimitSwitch(-js2lSpeed);
// }
// else
// {
//     ArmLMotor.Set(ControlMode::PercentOutput, 0);
//     ArmRMotor.Set(ControlMode::PercentOutput, 0);
// }
#pragma endregion Arm - Manual - Analog - Left
#pragma region Arm - Manual - Analog - Right
        std::cout << "js2rSpeed: " << js2rSpeed << std::endl;
        if (js2rSpeed > 0 || js2rSpeed < 0)
        {
            ArmLimitSwitch(js2rSpeed);
        }
        else
        {
            ArmLMotor.Set(ControlMode::PercentOutput, 0);
            ArmRMotor.Set(ControlMode::PercentOutput, 0);
        }
#pragma endregion Arm - Manual - Analog - Right
#pragma endregion Arm - Manual - Analog

#pragma region Arm - Manual
// // Press & Hold LB to lower arm - Begin
// if (js2.GetRawButton(btnD_LB))
// {
//     // if (!armlimitswitchLow)
//     // {
//     ArmLimitSwitch(-0.3);
//     // }
// }
// // Press & Hold LB to lower arm - End
// // Press & Hold RB to high arm - Begin
// if (js2.GetRawButton(btnD_RB) && !js2.GetRawButton(btnD_X) && !js2.GetRawButton(btnD_Y) && !js2.GetRawButton(btnD_B))
// {
//     // if (!armlimitswitchHigh)
//     // {
//     ArmLimitSwitch(0.3);
//     // }
// }
// // Press & Hold RB to high arm - End
// if (!js2.GetRawButton(btnD_LB) && !js2.GetRawButton(btnD_RB))
// {
//     // Values from -1 to 1 are accepted.
//     ArmLMotor.Set(ControlMode::PercentOutput, 0);
//     ArmRMotor.Set(ControlMode::PercentOutput, 0);
// }
#pragma endregion Arm - Manual

#pragma region Ball Intake Extend - In / Out
        if (js2.GetRawButtonPressed(btnD_LB))
        {
            ds_BallIntakeExtend.Set(frc::DoubleSolenoid::kForward); // Bring In
            std::cout << "Double Solenoid: " << ds_BallIntakeExtend.Get() << std::endl;
        }
        else if (js2.GetRawButtonPressed(btnD_RB))
        {
            ds_BallIntakeExtend.Set(frc::DoubleSolenoid::kReverse); // Extend
            std::cout << "Double Solenoid: " << ds_BallIntakeExtend.Get() << std::endl;
        }
        else
        {
            ds_BallIntakeExtend.Set(frc::DoubleSolenoid::kOff);
            std::cout << "Double Solenoid: " << ds_BallIntakeExtend.Get() << std::endl;
        }
#pragma endregion Ball Intake Extend - In / Out

#pragma endregion Get JS2 User Input

#pragma region Get JS1 &JS2 User Input

#pragma region Ball Intake - Ball In / Out
        if (js2.GetRawButton(btnD_RT) || js1.GetRawButton(btnD_RT))
        {
            BallIntake(-1); // Kick Ball Out
        }
        else if (js2.GetRawButton(btnD_LT) || js1.GetRawButton(btnD_LT))
        {
            BallIntake(1); // Bring Ball In
        }
        else
        {
            BallIntakeMotor.Set(ControlMode::PercentOutput, 0);
        }
#pragma endregion Ball Intake - Ball In / Out
#pragma endregion Get JS1 &JS2 User Input
#pragma endregion Controller Values
#pragma endregion Autonomous Periodic - Inner
    }
}
#pragma endregion Autonomous Periodic

// When Teleop Begins
#pragma region Teleop Init
void Robot::TeleopInit()
{
#pragma region Telop Init - Inner
#pragma region Dead Band Variables JS1
    js1lSpeed = 0;  // Speed controls up & down
    js1rSpeed = 0;  // Speed controls up & down
    js1lRotate = 0; // Rotate controls left & right
    js1rRotate = 0; // Rotate controls left & right
    js1LTSpeed = 0;
    js1RTSpeed = 0;
#pragma endregion Dead Band Variables JS1

#pragma region Dead Band Variables JS2
    js2lSpeed = 0;  // Speed controls up & down
    js2rSpeed = 0;  // Speed controls up & down
    js2lRotate = 0; // Rotate controls left & right
    js2rRotate = 0; // Rotate controls left & right
    js2LTSpeed = 0;
    js2RTSpeed = 0;
#pragma endregion Dead Band Variables JS2

    s_FrontHatchPickup.Set(true);

    if (nTestWithoutCompressor == 1)
    {
        // FOR TESTING - START WITHOUT COMPRESSOR - START
        compressor->SetClosedLoopControl(false);
        // FOR TESTING - START WITHOUT COMPRESSOR - STOP
    }
    else
    {
        compressor->SetClosedLoopControl(true);
    }

#pragma region Set Motor Values To Zero
    // Values from -1 to 1 are accepted.
    ArmRMotor.Set(ControlMode::PercentOutput, 0);
    ArmLMotor.Set(ControlMode::PercentOutput, 0);
    BallIntakeMotor.Set(ControlMode::PercentOutput, 0);
// PlatformDriveMotor.Set(ControlMode::PercentOutput, 0);
// pwmPlatformExtend.Set(0);
#pragma endregion Set Motor Values To Zero

#pragma region Set Encoder Values To Zero
// ArmRMotor.GetSensorCollection().SetQuadraturePosition(0, 0);
// ArmRMotor.GetSensorCollection().SetPulseWidthPosition(0, 0);
// ArmLMotor.GetSensorCollection().SetQuadraturePosition(0, 0);
// ArmLMotor.GetSensorCollection().SetPulseWidthPosition(0, 0);
#pragma endregion Set Encoder Values To Zero

    // m_frontRight.Set(ControlMode::PercentOutput, 0); // Right Front Drive Motor
    // m_rearRight.Set(ControlMode::PercentOutput, 0);  // Right Rear Drive Motor
    // m_frontLeft.Set(ControlMode::PercentOutput, 0);  // Left Front Drive Motor
    // m_rearLeft.Set(ControlMode::PercentOutput, 0);   // Left Rear Drive Motor

#pragma region CTRE Arcade Drive
    m_frontRight->Set(ControlMode::PercentOutput, 0); // Right Front Drive Motor
    m_rearRight->Set(ControlMode::PercentOutput, 0);  // Right Rear Drive Motor
    m_frontLeft->Set(ControlMode::PercentOutput, 0);  // Left Front Drive Motor
    m_rearLeft->Set(ControlMode::PercentOutput, 0);   // Left Rear Drive Motor
#pragma endregion CTRE Arcade Drive
#pragma endregion Telop Init - Inner
}
#pragma endregion Teleop Init

// Every 20ms of Teleop
#pragma region Teleop Periodic
void Robot::TeleopPeriodic()
{
#pragma region Telop Periodic - Inner

#pragma region Timer Definition
    // Now let's use the timer (it's an object so we use '.')
    MyTimer.Reset();
    MyTimer.Start();
    double timenow = MyTimer.Get();
    std::cout << "Timer: " << timenow << std::endl;
#pragma endregion Timer Definition

#pragma region Custom Max Speed
    prefs = Preferences::GetInstance();
    js1lMax = prefs->GetDouble("js1lMax", 1.0);
    js1rMax = prefs->GetDouble("js1rMax", 1.0);
    js2lMax = prefs->GetDouble("js2lMax", 1.0);
    js2rMax = prefs->GetDouble("js2rMax", 1.0);

    if (js1lDeadband_2 != js1lMax)
    {
        js1lDeadband_2 = js1lMax;
        std::cout << "js1lDeadband_2: " << js1lDeadband_2 << std::endl;
    }

    if (js1rDeadband_2 != js1rMax)
    {
        js1rDeadband_2 = js1rMax;
        std::cout << "js1rDeadband_2: " << js1rDeadband_2 << std::endl;
    }

    if (js2lDeadband_2 != js2lMax)
    {
        js2lDeadband_2 = js2lMax;
        std::cout << "js2lDeadband_2: " << js2lDeadband_2 << std::endl;
    }

    if (js2rDeadband_2 != js2rMax)
    {
        js2rDeadband_2 = js2rMax;
        std::cout << "js2rDeadband_2: " << js2rDeadband_2 << std::endl;
    }

//  std::cout << "js1lMax: " << js1lMax << std::endl;
//  std::cout << "js1rMax: " << js1rMax << std::endl;
//  std::cout << "js2lMax: " << js2lMax << std::endl;
//  std::cout << "js2rMax: " << js2rMax << std::endl;
#pragma endregion Custom Max Speed

#pragma region Smart Dashboard / Logs
    currentPWP = ArmLMotor.GetSelectedSensorPosition(0);
    std::cout << "Axis currentPWP " << currentPWP << std::endl;
    frc::SmartDashboard::PutNumber("Current Encoder Value", ArmLMotor.GetSelectedSensorPosition(0));
    DriverStation &ds = DriverStation::GetInstance();
    voltage = ds.GetBatteryVoltage();
    frc::SmartDashboard::PutNumber("Battery Voltage", voltage);
    bool bBatteryGood = true;
    if (voltage < 9.8)
    {
        bBatteryGood = false;
    }
    frc::SmartDashboard::PutBoolean("Battery Life", bBatteryGood);
    int station;
    station = ds.GetLocation();
    frc::SmartDashboard::PutNumber("Position", station);
    if (ds.GetAlliance() == 0.0)
    {
        frc::SmartDashboard::PutString("Alliance Color", "Red");
    }
    else
    {
        frc::SmartDashboard::PutString("Alliance Color", "Blue");
    }
    frc::SmartDashboard::PutBoolean("Limit Switch Low", armlimitswitchLow->Get());
    frc::SmartDashboard::PutBoolean("Limit Switch High", armlimitswitchHigh->Get());
    currentPWP = ArmLMotor.GetSelectedSensorPosition(0);
    std::cout << "Axis currentPWP " << currentPWP << std::endl;
    frc::SmartDashboard::PutNumber("Current Encoder Value", ArmLMotor.GetSelectedSensorPosition(0));
    // Compressor Stats
    bool enabled = compressor->Enabled();
    bool pressureSwitch = compressor->GetPressureSwitchValue();
    double CompressorCurrent = compressor->GetCompressorCurrent();
    std::cout << "Compressor Current: " << CompressorCurrent << std::endl;
    std::cout << "Pressure Switch: " << pressureSwitch << std::endl;
    frc::SmartDashboard::PutNumber("Compressor Current", CompressorCurrent);
    frc::SmartDashboard::PutNumber("Pressure Switch", pressureSwitch);
#pragma endregion Smart Dashboard / Logs

#pragma region Controller Values
#pragma region Get JS1 User Input
    js1lSpeed = js1.GetRawAxis(1); // Speed controls up & down
    // js1LTSpeed = js1.GetRawAxis(2); // L Trigger Axis (Controller X)
    // js1RTSpeed = js1.GetRawAxis(3); // R Trigger Axis (Controller X)
    // js1rSpeed = js1.GetRawAxis(4);  // Speed controls left & right (Controller X)
    js1rSpeed = -js1.GetRawAxis(2); // Speed controls left & right (Controller D)
    js1lRotate = js1.GetRawAxis(0); // Rotate controls left & right
    js1rRotate = js1.GetRawAxis(2); // Rotate controls left & right

    frc::SmartDashboard::PutNumber("JS1 Left Speed", js1lSpeed);
    frc::SmartDashboard::PutNumber("JS1 Right Speed", js1rSpeed);

#pragma region Dead Bands
    if ((js1lSpeed > js1lDeadband_1))
    {
        js1lSpeed = ((js1lSpeed - js1lDeadband_1) * js1lDeadband_2);
    }
    else if ((js1lSpeed < -js1lDeadband_1))
    {
        js1lSpeed = ((js1lSpeed + js1lDeadband_1) * js1lDeadband_2);
    }
    else
    {
        js1lSpeed = 0; // If between boundaries. Do nothing.
    }
    if ((js1rSpeed > js1rDeadband_1))
    {
        js1rSpeed = ((js1rSpeed - js1rDeadband_1) * js1rDeadband_2);
    }
    else if ((js1rSpeed < -js1rDeadband_1))
    {
        js1rSpeed = ((js1rSpeed + js1rDeadband_1) * js1rDeadband_2);
    }
    else
    {
        js1rSpeed = 0; // If between boundaries. Do nothing.
    }
#pragma endregion Dead Bands

#pragma region Lift Robot - Raise
    // js1.GetRawButtonPressed // Don't have to hold down the button
    if (js1.GetRawButton(btnD_Back)) // Raise The Robot Front
    {
        s_LiftSolenoid1.Set(true); // Front - Raise Robot
    }
    else
    {
        s_LiftSolenoid1.Set(false);
    }
    if (js1.GetRawButton(btnD_Start)) // Raise The Robot Rear
    {
        s_LiftSolenoid0.Set(true); // Rear - Raise Robot
    }
    else
    {
        s_LiftSolenoid0.Set(false);
    }
#pragma endregion Lift Robot - Raise

#pragma region Arcade Drive
    // m_drive.ArcadeDrive(-js1lSpeed, js1rSpeed);
    if (js1.GetRawButton(btnD_X)) // Slow forward/reverse
    {
        CTREArcadeDrive(js1lSpeed / 2, js1rSpeed);
    }
    else
    {
        CTREArcadeDrive(js1lSpeed, js1rSpeed);
    }
#pragma endregion Arcade Drive

#pragma region Hatch Pickup - Front
    if (js1.GetRawButtonPressed(btnD_RB))
    {
        s_FrontHatchPickup.Set(true);
    }
    if (js1.GetRawButtonPressed(btnD_LB))
    {
        s_FrontHatchPickup.Set(false);
    }
#pragma endregion Hatch Pickup - Front

#pragma endregion Get JS1 User Input

#pragma region Get JS2 User Input
    js2lSpeed = js2.GetRawAxis(1); // Speed controls up & down
    // js2LTSpeed = js2.GetRawAxis(2); // L Trigger Axis (Controller X)
    // js2RTSpeed = js2.GetRawAxis(3); // R Trigger Axis (Controller X)
    // js2rSpeed = js2.GetRawAxis(4);  // Speed controls left & right (Controller X)
    js2rSpeed = -js2.GetRawAxis(3); // Speed controls Up & Down (Controller D)
    js2lRotate = js2.GetRawAxis(0); // Rotate controls left & right
    js2rRotate = js2.GetRawAxis(2); // Rotate controls left & right

    frc::SmartDashboard::PutNumber("JS2 Left Speed", js2lSpeed);
    frc::SmartDashboard::PutNumber("JS2 Right Speed", js2rSpeed);
#pragma region Dead Bands
    if ((js2lSpeed > js2lDeadband_1))
    {
        js2lSpeed = ((js2lSpeed - js2lDeadband_1) * js2lDeadband_2);
    }
    else if ((js2lSpeed < -js2lDeadband_1))
    {
        js2lSpeed = ((js2lSpeed + js2lDeadband_1) * js2lDeadband_2);
    }
    else
    {
        js2lSpeed = 0; // If between boundaries. Do nothing.
    }
    if ((js2rSpeed > js2rDeadband_1))
    {
        js2rSpeed = ((js2rSpeed - js2rDeadband_1) * js2rDeadband_2);
    }
    else if ((js2rSpeed < -js2rDeadband_1))
    {
        js2rSpeed = ((js2rSpeed + js2rDeadband_1) * js2rDeadband_2);
    }
    else
    {
        js2rSpeed = 0; // If between boundaries. Do nothing.
    }
#pragma endregion Dead Bands

#pragma region Lift Robot - Lower
    if (js2.GetRawButton(btnD_Start)) // Lower Robot Rear
    {
        s_LiftSolenoid2.Set(true); // Rear Lower Robot
    }
    else
    {
        s_LiftSolenoid2.Set(false);
    }
    if (js2.GetRawButton(btnD_Back)) // Lower Robot Front
    {
        s_LiftSolenoid3.Set(true); // Front Lower Robot
    }
    else
    {
        s_LiftSolenoid3.Set(false);
    }
#pragma endregion Lift Robot - Lower

#pragma region Rear Hatch Extend Button Controls
    if (js2.GetRawButtonPressed(btnD_A)) // Bring In
    {
        ds_RearHatchExtend.Set(frc::DoubleSolenoid::kForward);
        std::cout << "Double Solenoid: " << ds_RearHatchExtend.Get() << std::endl;
    }
    else if (js2.GetRawButtonPressed(btnD_Y)) // Extend Out
    {
        ds_RearHatchExtend.Set(frc::DoubleSolenoid::kReverse);
        std::cout << "Double Solenoid: " << ds_RearHatchExtend.Get() << std::endl;
    }
    else
    {
        ds_RearHatchExtend.Set(frc::DoubleSolenoid::kOff);
        std::cout << "Double Solenoid: " << ds_RearHatchExtend.Get() << std::endl;
    }
#pragma endregion Rear Hatch Extend Button Controls
#pragma region Rear Hatch Grab Button Controls
    if (js2.GetRawButtonPressed(btnD_B)) // Release
    {
        ds_RearHatchGrab.Set(frc::DoubleSolenoid::kForward);
        std::cout << "Double Solenoid: " << ds_RearHatchGrab.Get() << std::endl;
    }
    else if (js2.GetRawButtonPressed(btnD_X)) // Pick Up
    {
        ds_RearHatchGrab.Set(frc::DoubleSolenoid::kReverse);
        std::cout << "Double Solenoid: " << ds_RearHatchGrab.Get() << std::endl;
    }
    else
    {
        ds_RearHatchGrab.Set(frc::DoubleSolenoid::kOff);
        std::cout << "Double Solenoid: " << ds_RearHatchGrab.Get() << std::endl;
    }
#pragma endregion Rear Hatch Grab Button Controls

#pragma region Arm - Manual - Analog
#pragma region Arm - Manual - Analog - Left
// std::cout << "js2lSpeed: " << js2lSpeed << std::endl;
// if (js2lSpeed > 0)
// {
//     std::cout << "Arm Up" << std::endl;
//     ArmLimitSwitch(js2lSpeed);
// }
// else if (js2lSpeed < 0)
// {
//     std::cout << "Arm Down" << std::endl;
//     ArmLimitSwitch(-js2lSpeed);
// }
// else
// {
//     ArmLMotor.Set(ControlMode::PercentOutput, 0);
//     ArmRMotor.Set(ControlMode::PercentOutput, 0);
// }
#pragma endregion Arm - Manual - Analog - Left
#pragma region Arm - Manual - Analog - Right
    std::cout << "js2rSpeed: " << js2rSpeed << std::endl;
    if (js2rSpeed > 0 || js2rSpeed < 0)
    {
        ArmLimitSwitch(js2rSpeed);
    }
    else
    {
        ArmLMotor.Set(ControlMode::PercentOutput, 0);
        ArmRMotor.Set(ControlMode::PercentOutput, 0);
    }
#pragma endregion Arm - Manual - Analog - Right
#pragma endregion Arm - Manual - Analog

#pragma region Arm - Manual
// // Press & Hold LB to lower arm - Begin
// if (js2.GetRawButton(btnD_LB))
// {
//     // if (!armlimitswitchLow)
//     // {
//     ArmLimitSwitch(-0.3);
//     // }
// }
// // Press & Hold LB to lower arm - End
// // Press & Hold RB to high arm - Begin
// if (js2.GetRawButton(btnD_RB) && !js2.GetRawButton(btnD_X) && !js2.GetRawButton(btnD_Y) && !js2.GetRawButton(btnD_B))
// {
//     // if (!armlimitswitchHigh)
//     // {
//     ArmLimitSwitch(0.3);
//     // }
// }
// // Press & Hold RB to high arm - End
// if (!js2.GetRawButton(btnD_LB) && !js2.GetRawButton(btnD_RB))
// {
//     // Values from -1 to 1 are accepted.
//     ArmLMotor.Set(ControlMode::PercentOutput, 0);
//     ArmRMotor.Set(ControlMode::PercentOutput, 0);
// }
#pragma endregion Arm - Manual

#pragma region Ball Intake Extend - In / Out
    if (js2.GetRawButtonPressed(btnD_LB))
    {
        ds_BallIntakeExtend.Set(frc::DoubleSolenoid::kForward); // Bring In
        std::cout << "Double Solenoid: " << ds_BallIntakeExtend.Get() << std::endl;
    }
    else if (js2.GetRawButtonPressed(btnD_RB))
    {
        ds_BallIntakeExtend.Set(frc::DoubleSolenoid::kReverse); // Extend
        std::cout << "Double Solenoid: " << ds_BallIntakeExtend.Get() << std::endl;
    }
    else
    {
        ds_BallIntakeExtend.Set(frc::DoubleSolenoid::kOff);
        std::cout << "Double Solenoid: " << ds_BallIntakeExtend.Get() << std::endl;
    }
#pragma endregion Ball Intake Extend - In / Out

#pragma endregion Get JS2 User Input

#pragma region Get JS1 &JS2 User Input

#pragma region Ball Intake - Ball In / Out
    if (js2.GetRawButton(btnD_RT) || js1.GetRawButton(btnD_RT))
    {
        BallIntake(-1); // Kick Ball Out
    }
    else if (js2.GetRawButton(btnD_LT) || js1.GetRawButton(btnD_LT))
    {
        BallIntake(1); // Bring Ball In
    }
    else
    {
        BallIntakeMotor.Set(ControlMode::PercentOutput, 0);
    }
#pragma endregion Ball Intake - Ball In / Out
#pragma endregion Get JS1 &JS2 User Input
#pragma endregion Controller Values
#pragma endregion Telop Periodic - Inner
}
#pragma endregion Teleop Periodic

#pragma region Test Periodic
void Robot::TestPeriodic()
{
}
#pragma endregion Test Periodic

#ifndef RUNNING_FRC_TESTS
int main()
{
    return frc::StartRobot<Robot>();
}
#endif