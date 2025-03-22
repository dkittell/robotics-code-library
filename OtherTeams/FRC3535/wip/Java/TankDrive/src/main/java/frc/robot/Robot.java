/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.SpeedControllerGroup;

/**
 * This is a demo program showing the use of the RobotDrive class, specifically
 * it contains the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  private Joystick m_leftStick;
  private Joystick m_rightStick;

  // region Motors
  private static final int kFrontLeftChannel = 0;
  private static final int kRearLeftChannel = 1;
  private static final int kFrontRightChannel = 2;
  private static final int kRearRightChannel = 3;
  // endregion Motors
  PWMVictorSPX m_frontLeft = new PWMVictorSPX(kFrontLeftChannel);
  PWMVictorSPX m_rearLeft = new PWMVictorSPX(kRearLeftChannel);
  PWMVictorSPX m_frontRight = new PWMVictorSPX(kFrontRightChannel);
  PWMVictorSPX m_rearRight = new PWMVictorSPX(kRearRightChannel);

  @Override
  public void robotInit() {

  }

  @Override
  public void teleopPeriodic() {
    SpeedControllerGroup m_left = new SpeedControllerGroup(m_frontLeft, m_rearLeft);
    SpeedControllerGroup m_right = new SpeedControllerGroup(m_frontRight, m_rearRight);

    DifferentialDrive m_drive = new DifferentialDrive(m_left, m_right);
    m_leftStick = new Joystick(0);
    m_rightStick = new Joystick(1);

    m_drive.tankDrive(m_leftStick.getY(), m_rightStick.getY());
  }

}
