// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 * This is a demo program showing the use of the RobotDrive class, specifically it contains the code
 * necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {

  private DifferentialDrive m_myRobot;
  private Joystick m_leftStick;
  private Joystick m_rightStick;

  public static PWMVictorSPX m_frontLeft = new PWMVictorSPX(0);
  public static PWMVictorSPX m_frontRight = new PWMVictorSPX(2);
  public static PWMVictorSPX m_rearLeft = new PWMVictorSPX(1);
  public static PWMVictorSPX m_rearRight = new PWMVictorSPX(3);

  // 0 and 1 Left Side
  // 2 and 3 Right Side

  public static SpeedControllerGroup m_leftGroup = new SpeedControllerGroup(
    m_frontLeft,
    m_rearLeft
  );
  public static SpeedControllerGroup m_rightGroup = new SpeedControllerGroup(
    m_frontRight,
    m_rearRight
  );
  public static DifferentialDrive m_drive = new DifferentialDrive(
    m_leftGroup,
    m_rightGroup
  );

  @Override
  public void robotInit() {
    m_leftStick = new Joystick(0);
    // m_rightStick = new Joystick(1);
  }

  @Override
  public void teleopPeriodic() {
    // m_drive.tankDrive(-m_leftStick.getRawAxis(1), -m_leftStick.getRawAxis(3)); // Logitech F310
    // m_drive.arcadeDrive(-m_leftStick.getRawAxis(1), m_leftStick.getRawAxis(2)); // Logitech F310
    m_drive.arcadeDrive(-m_leftStick.getRawAxis(1), m_leftStick.getRawAxis(0)); // Logitech Attack 3
  }
}
