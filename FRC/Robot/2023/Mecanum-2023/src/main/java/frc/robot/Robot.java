// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
// import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.MecanumDrive;

/** This is a demo program showing how to use Mecanum control with the MecanumDrive class. */
public class Robot extends TimedRobot {

  // private static final int kFrontLeftChannel = 2;
  // private static final int kRearLeftChannel = 3;
  // private static final int kFrontRightChannel = 1;
  // private static final int kRearRightChannel = 0;

  public static final int can_id_frontLeft = 3;
  public static final int can_id_rearLeft = 1;
  public static final int can_id_frontRight = 4;
  public static final int can_id_rearRight = 2;

  private static final int kJoystickChannel = 0;

  private MecanumDrive m_robotDrive;
  private Joystick m_stick;

  public CANSparkMax m_frontLeft = new CANSparkMax(
    can_id_frontLeft,
    MotorType.kBrushless
  );
  public CANSparkMax m_frontRight = new CANSparkMax(
    can_id_frontRight,
    MotorType.kBrushless
  );
  public CANSparkMax m_rearLeft = new CANSparkMax(
    can_id_rearLeft,
    MotorType.kBrushless
  );
  public CANSparkMax m_rearRight = new CANSparkMax(
    can_id_rearRight,
    MotorType.kBrushless
  );

  @Override
  public void robotInit() {
    // PWMSparkMax frontLeft = new PWMSparkMax(kFrontLeftChannel);
    // PWMSparkMax rearLeft = new PWMSparkMax(kRearLeftChannel);
    // PWMSparkMax frontRight = new PWMSparkMax(kFrontRightChannel);
    // PWMSparkMax rearRight = new PWMSparkMax(kRearRightChannel);

    // Invert the right side motors.
    // You may need to change or remove this to match your robot.
    m_frontRight.setInverted(true);
    m_rearRight.setInverted(true);

    m_robotDrive =
      new MecanumDrive(m_frontLeft, m_rearLeft, m_frontRight, m_rearRight);

    m_stick = new Joystick(kJoystickChannel);
  }

  @Override
  public void teleopPeriodic() {
    // Use the joystick X axis for forward movement, Y axis for lateral
    // movement, and Z axis for rotation.
    m_robotDrive.driveCartesian(
      -m_stick.getY(),
      m_stick.getX(),
      m_stick.getZ()
    );
  }

  @Override
  public void testPeriodic() {
    if (m_stick.getRawButton(1)) {
      m_frontLeft.set(0.45);
    } else {
      m_frontLeft.set(0);
    }
    if (m_stick.getRawButton(2)) {
      m_frontRight.set(0.45);
    } else {
      m_frontRight.set(0);
    }
    if (m_stick.getRawButton(3)) {
      m_rearLeft.set(0.45);
    } else {
      m_rearLeft.set(0);
    }
    if (m_stick.getRawButton(4)) {
      m_rearRight.set(0.45);
    } else {
      m_rearRight.set(0);
    }
  }
}
