// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.Timer;

/**
 * This is a demo program showing the use of the DifferentialDrive class. Runs
 * the motors with
 * arcade steering.
 */
public class Robot extends TimedRobot {

  // Make sure you use the REV Hardware Client to set the CAN IDs
  CANSparkMax m_FrontLeftMotor = new CANSparkMax(2, MotorType.kBrushed);
  CANSparkMax m_FrontRightMotor = new CANSparkMax(4, MotorType.kBrushed);
  CANSparkMax m_RearLeftMotor = new CANSparkMax(1, MotorType.kBrushed);
  CANSparkMax m_RearRightMotor = new CANSparkMax(3, MotorType.kBrushed);

  private DifferentialDrive m_robotDrive = new DifferentialDrive(m_FrontLeftMotor::set, m_FrontRightMotor::set);
  private  Joystick js_Driver = new Joystick(0);

  public Robot() {
    SendableRegistry.addChild(m_robotDrive, m_FrontLeftMotor);
    SendableRegistry.addChild(m_robotDrive, m_FrontRightMotor);
  }

  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    // 4 motor drive train setup
    m_FrontRightMotor.setInverted(true);
    m_RearRightMotor.setInverted(true);

    m_RearLeftMotor.follow(m_FrontLeftMotor);
    m_RearRightMotor.follow(m_FrontRightMotor);

    m_robotDrive = new DifferentialDrive(m_FrontLeftMotor::set, m_FrontRightMotor::set);
    js_Driver = new Joystick(0);
  }

  @Override
  public void teleopPeriodic() {
    // Only use one of the two and double check the axis you want to use within the
    // Driver Station application

    // Tank Drive, which controls the left and right side independently
    // Tank Drive - Logitech F310 Controller in D Switch Mode
    // m_robotDrive.tankDrive(js_Driver.getRawAxis(1), js_Driver.getRawAxis(3));
    // Tank Drive - Logitech F310 Controller in X Switch Mode
    // m_robotDrive.tankDrive(js_Driver.getRawAxis(1), js_Driver.getRawAxis(5));
    // Tank Drive - XBox Controller
    // m_robotDrive.tankDrive(js_Driver.getRawAxis(1), js_Driver.getRawAxis(5));

    // Arcade Drive, which controls a forward and turn speed
    // Arcade Drive - Logitech F310 Controller in D Switch Mode
    // m_robotDrive.arcadeDrive(js_Driver.getRawAxis(1), js_Driver.getRawAxis(2));
    // Arcade Drive - Logitech F310 Controller in X Switch Mode
    m_robotDrive.arcadeDrive(js_Driver.getRawAxis(1), js_Driver.getRawAxis(4));
    // Arcade Drive - XBox Controller
    // m_robotDrive.arcadeDrive(js_Driver.getRawAxis(1), js_Driver.getRawAxis(4));
  }
}
