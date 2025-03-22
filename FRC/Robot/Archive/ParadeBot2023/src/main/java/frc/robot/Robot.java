// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import com.ctre.phoenix.motorcontrol.can.*;


/**
 * This is a demo program showing the use of the DifferentialDrive class, specifically it contains
 * the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {

  private DifferentialDrive m_myRobot;
  private Joystick m_leftStick;
  private Joystick m_rightStick;

  private final WPI_VictorSPX m_frontLeft = new WPI_VictorSPX(7);
  private final WPI_VictorSPX m_rearLeft = new WPI_VictorSPX(9);
  private final WPI_VictorSPX m_frontRight = new WPI_VictorSPX(11);
  private final WPI_VictorSPX m_rearRight = new WPI_VictorSPX(12);

  MotorControllerGroup m_left = new MotorControllerGroup(
    m_frontLeft,
    m_rearLeft
  );

  MotorControllerGroup m_right = new MotorControllerGroup(
    m_frontRight,
    m_rearRight
  );

  DifferentialDrive m_drive = new DifferentialDrive(m_left, m_right);

  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_frontRight.setInverted(true);

    m_leftStick = new Joystick(0);
    m_rightStick = new Joystick(1);
  }

  @Override
  public void teleopPeriodic() {
    m_drive.tankDrive(-m_leftStick.getY(), -m_rightStick.getY());

  
  }
}
