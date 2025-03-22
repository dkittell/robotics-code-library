#include "rev/CANSparkMax.h"
#include <frc/Joystick.h>
#include <frc/TimedRobot.h>
#include <frc/drive/DifferentialDrive.h>

class Robot : public frc::TimedRobot
{

  // Define and assign motors
  static const int leftLeadDeviceID = 3, leftFollowDeviceID = 1, rightLeadDeviceID = 4, rightFollowDeviceID = 2;
  rev::CANSparkMax m_leftFollowMotor{leftFollowDeviceID, rev::CANSparkMax::MotorType::kBrushed};
  rev::CANSparkMax m_leftLeadMotor{leftLeadDeviceID, rev::CANSparkMax::MotorType::kBrushed};
  rev::CANSparkMax m_rightFollowMotor{rightFollowDeviceID, rev::CANSparkMax::MotorType::kBrushed};
  rev::CANSparkMax m_rightLeadMotor{rightLeadDeviceID, rev::CANSparkMax::MotorType::kBrushed};

  frc::DifferentialDrive m_robotDrive{m_leftLeadMotor, m_rightLeadMotor};
  frc::Joystick js_Driver{0};

public:
  void RobotInit()
  {
    m_leftFollowMotor.Follow(m_leftLeadMotor);
    m_rightFollowMotor.Follow(m_rightLeadMotor);
  }

  void TeleopPeriodic()
  {
    m_robotDrive.ArcadeDrive(-js_Driver.GetRawAxis(2), -js_Driver.GetRawAxis(1));
    // m_robotDrive.tankDrive(-js_Driver.getRawAxis(3), js_Driver.getRawAxis(1));
  }
};

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
