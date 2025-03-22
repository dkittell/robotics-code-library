/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <array>

#include <frc/AnalogInput.h>
#include <frc/Joystick.h>
#include <frc/PWMVictorSPX.h>
#include <frc/TimedRobot.h>
#include <frc/controller/PIDController.h>

#include <frc/Joystick.h>
#include <frc/WPILib.h>
#include <iostream>
#include <string>
#include <wpi/raw_ostream.h>

using namespace frc;

#include <CANVenom.h>

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
int axisX_LT = 2;
int axisX_RT = 3;
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

CANVenom *m_shooter = new CANVenom(5);

/**
 * This is a sample program to demonstrate how to use a soft potentiometer and a
 * PID Controller to reach and maintain position setpoints on an elevator
 * mechanism.
 */
class Robot : public frc::TimedRobot
{
public:
  void TeleopPeriodic() override
  {
    std::cout << "Get Postion " << m_shooter->GetPosition() << std::endl;
    std::cout << "Get Output Current " << m_shooter->GetOutputCurrent() << std::endl;
    std::cout << "Get Output Voltage " << m_shooter->GetOutputVoltage() << std::endl;
    std::cout << "Get Speed " << m_shooter->GetSpeed() << std::endl;
    std::cout << "Temperature " << m_shooter->GetTemperature() << " C" << std::endl;

#pragma region PID Loop
    // // When the button is pressed once, the selected elevator setpoint is
    // // incremented.
    m_shooter->SetInverted(1);
    // std::cout << "Axis LT " << js1.GetRawAxis(axisX_LT) << std::endl;
    if (js1.GetRawAxis(axisX_LT) > 0.15)
    {
      bool currentButtonValue = js1.GetRawAxis(axisX_LT);
      if (currentButtonValue && !m_previousButtonValue)
      {
        // Index of the elevator setpoint wraps around
        m_index = (m_index + 1) % (sizeof(kSetPoints) / 8);
      }
      m_previousButtonValue = currentButtonValue;

      m_pidController.SetSetpoint(kSetPoints[m_index]);
      double output =
          m_pidController.Calculate(m_potentiometer.GetAverageVoltage());
      m_shooter->Set(output);
    }

#pragma endregion PID Loop

    if (js1.GetRawButtonPressed(btnX_X))
    {
      m_shooter->Enable();
      m_shooter->Set(.7);
    }
    if (js1.GetRawButtonReleased(btnX_X))
    {
      m_shooter->StopMotor();
    }

    if (js1.GetRawButtonPressed(btnX_A))
    {
      m_shooter->Enable();
      m_shooter->Set(-.7);
    }
    if (js1.GetRawButtonReleased(btnX_A))
    {
      m_shooter->StopMotor();
    }

    if (js1.GetRawButton(btnX_Start))
    {
      std::cout << "Firmware " << m_shooter->GetFirmwareVersion() << std::endl;
      std::cout << "Serial Number " << m_shooter->GetSerialNumber() << std::endl;
      std::cout << "Temperature " << m_shooter->GetTemperature() << " C" << std::endl;
      std::cout << "Get Name " << m_shooter->GetName() << std::endl;
    }
  }

private:
  static constexpr int kPotChannel = 1;

  // Bottom, middle, and top elevator setpoints
  static constexpr std::array<double, 3> kSetPoints = {{1.0, 2.6, 4.3}};

  /* Proportional, integral, and derivative speed constants; motor inverted.
   *
   * DANGER: When tuning PID constants, high/inappropriate values for pGain,
   * iGain, and dGain may cause dangerous, uncontrollable, or undesired
   * behavior!
   *
   * These may need to be positive for a non-inverted motor.
   */
  static constexpr double kP = -5.0;
  static constexpr double kI = -0.02;
  static constexpr double kD = -2.0;

  int m_index = 0;
  bool m_previousButtonValue = false;

  frc::AnalogInput m_potentiometer{kPotChannel};

  frc2::PIDController m_pidController{kP, kI, kD};
};

constexpr std::array<double, 3> Robot::kSetPoints;

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
