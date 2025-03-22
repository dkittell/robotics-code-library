// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.MecanumDrive;

/** This is a demo program showing how to use Mecanum control with the MecanumDrive class. */
public class Robot extends TimedRobot {

  private static final int can_id_frontLeft = 3;
  private static final int can_id_rearLeft = 8;
  private static final int can_id_frontRight = 13;
  private static final int can_id_rearRight = 5;

  private static final int kJoystickChannel = 0;

  private MecanumDrive m_robotDrive;
  private Joystick m_stick;

  @Override
  public void robotInit() {
    CANSparkMax frontLeft = new CANSparkMax(
      can_id_frontLeft,
      MotorType.kBrushless
    );
    CANSparkMax rearLeft = new CANSparkMax(
      can_id_rearLeft,
      MotorType.kBrushless
    );
    CANSparkMax frontRight = new CANSparkMax(
      can_id_frontRight,
      MotorType.kBrushless
    );
    CANSparkMax rearRight = new CANSparkMax(
      can_id_rearRight,
      MotorType.kBrushless
    );

    SendableRegistry.addChild(m_robotDrive, frontLeft);
    SendableRegistry.addChild(m_robotDrive, rearLeft);
    SendableRegistry.addChild(m_robotDrive, frontRight);
    SendableRegistry.addChild(m_robotDrive, rearRight);

    // Invert the right side motors.
    // You may need to change or remove this to match your robot.
    frontLeft.setInverted(true);
    frontRight.setInverted(false);
    rearLeft.setInverted(false);
    rearRight.setInverted(true);

    m_robotDrive =
      new MecanumDrive(
        frontLeft::set,
        rearLeft::set,
        frontRight::set,
        rearRight::set
      );

    m_stick = new Joystick(kJoystickChannel);
  }

  @Override
  public void teleopPeriodic() {
    // Use the joystick Y axis for forward movement, X axis for lateral
    // movement, and Z axis for rotation.
    m_robotDrive.driveCartesian(
      -m_stick.getY(),
      m_stick.getX(),
      m_stick.getZ()
    );
  }
}
