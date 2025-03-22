/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"
#include <cameraserver/CameraServer.h>
#include <frc/Joystick.h>
#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/TimedRobot.h>
#include <frc/WPILib.h>
#include <iostream>
#include <string>
#include <wpi/raw_ostream.h>


using namespace frc;

#include <CANVenom.h>

#pragma region Custom Max Speed
Preferences *prefs;
double js1lMax;
double js1rMax;
double js2lMax;
double js2rMax;
#pragma endregion Custom Max Speed

#pragma region Controller Definition

#pragma region Controller Button Variables
#pragma region PXN
int btnPXND_X = 3;
int btnPXND_A = 1;
int btnPXND_B = 2;
int btnPXND_Y = 4;
int btnPXND_LB = 5;
int btnPXND_RB = 6;
int btnPXND_L3 = 9;
int btnPXND_R3 = 10;
int btnPXND_Share = 7;
int btnPXND_Options = 8;
#pragma endregion PXN

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

#pragma region Venom
CANVenom *m_frontLeft = new CANVenom(3);  // Left Front Drive Motor
CANVenom *m_frontRight = new CANVenom(1); // Right Front Drive Motor
CANVenom *m_rearLeft = new CANVenom(5);   // Left Rear Drive Motor
CANVenom *m_rearRight = new CANVenom(2);  // Right Rear Drive Motor
// CANVenom *m_shooter = new CANVenom(5);
#pragma endregion Venom

void Robot::RobotInit()
{
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic()
{
  frc::SmartDashboard::PutNumber("Firmware", 0);
  frc::SmartDashboard::PutNumber("Serial Number ", 0);
  frc::SmartDashboard::PutNumber("Temperature ", 0);
  frc::SmartDashboard::PutNumber("Get Name ", 0);
  frc::SmartDashboard::PutNumber("Current Encoder Value", 0);
  frc::SmartDashboard::PutNumber("Current Output Current", 0);
  frc::SmartDashboard::PutNumber("Current Output Voltage", 0);
  frc::SmartDashboard::PutNumber("Current Speed", 0);
}

#pragma region Auton
/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit()
{
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  std::cout << "Auto selected: " << m_autoSelected << std::endl;

  if (m_autoSelected == kAutoNameCustom)
  {
    // Custom Auto goes here
  }
  else
  {
    // Default Auto goes here
  }
}

void Robot::AutonomousPeriodic()
{
  if (m_autoSelected == kAutoNameCustom)
  {
    // Custom Auto goes here
  }
  else
  {
    // Default Auto goes here
  }
}
#pragma endregion Auton

#pragma region Teleop
void Robot::TeleopInit()
{
  m_rearLeft->SetMaxAcceleration(20000);             // Set max acceleration to 20,000 RPM/s
  m_rearLeft->SetMaxJerk(31250);                     //Set max jerk to 31,250 RPM/s^2
  m_rearLeft->SetPID(0.195, 0.010, 0.0, 0.184, 0.0); // Configure PID gains
}

void Robot::TeleopPeriodic()
{

// std::cout << "Get Position " << m_rearLeft->GetPosition() << std::endl;
  // std::cout << "Get Output Current " << m_rearLeft->GetOutputCurrent() << std::endl;
  // std::cout << "Get Output Voltage " << m_rearLeft->GetOutputVoltage() << std::endl;
  std::cout << "Get Speed " << m_rearLeft->GetSpeed() << std::endl;

  if (js1.GetRawButton(btnD_Start))
  {
    std::cout << "Firmware " << m_rearLeft->GetFirmwareVersion() << std::endl;
    std::cout << "Serial Number " << m_rearLeft->GetSerialNumber() << std::endl;
    std::cout << "Temperature " << m_rearLeft->GetTemperature() << " C" << std::endl;
    std::cout << "Get Name " << m_rearLeft->GetName() << std::endl;
  }

  if (js1.GetRawButtonPressed(btnD_X))
  { 
   
    m_rearLeft->Enable();
    m_rearLeft->Set(.4);
    std::cout << "Get Position " << m_rearLeft->GetPosition() << std::endl;
    std::cout << "Get Output Current " << m_rearLeft->GetOutputCurrent() << std::endl;
    std::cout << "Get Output Voltage " << m_rearLeft->GetOutputVoltage() << std::endl;
    std::cout << "Get Speed " << m_rearLeft->GetSpeed() << std::endl;
    // std::cout << "Get Speed " <<  << std::endl;

  }
  if (js1.GetRawButtonReleased(btnD_X))
  {
    m_rearLeft->StopMotor();
  }

  if (js1.GetRawButtonPressed(btnD_A))
  {
    m_rearLeft->Enable();
   // m_rearLeft->Set(-.4);
	 m_rearLeft->SetCommand(CANVenom::ControlMode::kSpeedControl, 1300); // Spin the motor at 3000 RPM.
    std::cout << "Get Position " << m_rearLeft->GetPosition() << std::endl;
    std::cout << "Get Output Current " << m_rearLeft->GetOutputCurrent() << std::endl;
    std::cout << "Get Output Voltage " << m_rearLeft->GetOutputVoltage() << std::endl;
    std::cout << "Get Speed " << m_rearLeft->GetSpeed() << std::endl;
  }
  if (js1.GetRawButtonReleased(btnD_A))
  {
    m_rearLeft->StopMotor();
  }
}
#pragma endregion Teleop

void Robot::TestPeriodic()
{
}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
