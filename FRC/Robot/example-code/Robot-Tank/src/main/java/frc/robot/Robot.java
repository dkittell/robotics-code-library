// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


/**
 * This is a demo program showing the use of the DifferentialDrive class, specifically it contains
 * the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  private DifferentialDrive m_myRobot;
  private Joystick m_leftStick;
  private Joystick m_rightStick;
  

  // private final MotorController m_leftMotor = new PWMSparkMax(1);
  // private final MotorController m_rightMotor = new PWMSparkMax(2);

  private final CANSparkMax m_FrontLeftMotor = new CANSparkMax(1, MotorType.kBrushed);
  private final CANSparkMax m_FrontRightMotor = new CANSparkMax(2, MotorType.kBrushed);
  private final CANSparkMax m_RearLeftMotor = new CANSparkMax(2, MotorType.kBrushed);
  private final CANSparkMax m_RearRightMotor = new CANSparkMax(2, MotorType.kBrushed);

  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotor.setInverted(true);

    m_myRobot = new DifferentialDrive(m_leftMotor, m_rightMotor);
    m_leftStick = new Joystick(0);
    m_rightStick = new Joystick(1);
  }


  @Override
  public void autonomousPeriodic() {
    m_myRobot.tankDrive(0.6, 0.6);
  }


  @Override
  public void teleopPeriodic() {
    m_myRobot.tankDrive(m_leftStick.getY(), m_rightStick.getY());
  }
}
