// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 * This is a demo program showing the use of the DifferentialDrive class. Runs the motors with
 * arcade steering.
 */
public class Robot extends TimedRobot {

  // private final PWMSparkMax m_leftMotor = new PWMSparkMax(0);
  // private final PWMSparkMax m_rightMotor = new PWMSparkMax(1);

  // CANSparkMax m_frontLeft = new CANSparkMax(4, MotorType.kBrushed);
  // CANSparkMax m_frontRight = new CANSparkMax(3, MotorType.kBrushed);
  // CANSparkMax m_rearLeft = new CANSparkMax(2, MotorType.kBrushed);
  // CANSparkMax m_rearRight = new CANSparkMax(1, MotorType.kBrushed);

  private final WPI_VictorSPX m_frontLeft = new WPI_VictorSPX(7);
  private final WPI_VictorSPX m_rearLeft = new WPI_VictorSPX(9);
  private final WPI_VictorSPX m_frontRight = new WPI_VictorSPX(11);
  private final WPI_VictorSPX m_rearRight = new WPI_VictorSPX(12);

  private final DifferentialDrive m_robotDrive = new DifferentialDrive(
    m_frontLeft,
    m_frontRight
  );
  private final Joystick m_stick = new Joystick(0);

  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_frontRight.setInverted(true);

    m_rearLeft.follow(m_frontLeft);
    m_rearRight.follow(m_frontRight);
  }

  @Override
  public void teleopPeriodic() {
    // Drive with arcade drive.
    // That means that the Y axis drives forward
    // and backward, and the X turns left and right.
    // m_robotDrive.arcadeDrive(m_stick.getRawAxis(1), m_stick.getRawAxis(2)); // Logitech
    m_robotDrive.arcadeDrive(m_stick.getRawAxis(1), m_stick.getRawAxis(4)); // XBox
    // m_frontLeft.set(0.5);

  }
}
