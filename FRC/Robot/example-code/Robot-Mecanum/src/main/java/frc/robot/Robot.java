// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 * This is a demo program showing how to use Mecanum control with the
 * MecanumDrive class.
 */
public class Robot extends TimedRobot {
  private static final int kFrontLeftChannel = 2;
  private static final int kRearLeftChannel = 4;
  private static final int kFrontRightChannel = 1;
  private static final int kRearRightChannel = 3;

  private static final int kJoystickChannel = 0;

  private MecanumDrive m_robotDrive;
  private Joystick m_stick;

  @Override
  public void robotInit() {
    // PWMSparkMax frontLeft = new PWMSparkMax(kFrontLeftChannel);
    // PWMSparkMax rearLeft = new PWMSparkMax(kRearLeftChannel);
    // PWMSparkMax frontRight = new PWMSparkMax(kFrontRightChannel);
    // PWMSparkMax rearRight = new PWMSparkMax(kRearRightChannel);

    CANSparkMax frontLeft = new CANSparkMax(kFrontLeftChannel, MotorType.kBrushed);
    CANSparkMax rearLeft = new CANSparkMax(kRearLeftChannel, MotorType.kBrushed);
    CANSparkMax frontRight = new CANSparkMax(kFrontRightChannel, MotorType.kBrushed);
    CANSparkMax rearRight = new CANSparkMax(kRearRightChannel, MotorType.kBrushed);

    // Invert the right side motors.
    // You may need to change or remove this to match your robot.
    frontLeft.setInverted(true);
    frontRight.setInverted(false);
    rearLeft.setInverted(true);
    rearRight.setInverted(false);

    m_robotDrive = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);

    m_stick = new Joystick(kJoystickChannel);
  }

  @Override
  public void autonomousPeriodic() {

    m_robotDrive.driveCartesian(0.3, 0.0, 0.0, 0.0);

  }

  @Override
  public void teleopPeriodic() {
    // Use the joystick X axis for lateral movement, Y axis for forward
    // movement, and Z axis for rotation.
    m_robotDrive.driveCartesian(-m_stick.getY(), m_stick.getX(), m_stick.getZ(), 0.0);
  }
}
