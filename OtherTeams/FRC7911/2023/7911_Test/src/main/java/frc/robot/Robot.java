// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.MecanumDrive;

/** This is a demo program showing how to use Mecanum control with the MecanumDrive class. */
public class Robot extends TimedRobot {

  private static final int nFrontLeft = 5;
  private static final int nRearLeft = 7;
  private static final int nFrontRight = 8;
  private static final int nRearRight = 6;

  private final DoubleSolenoid p_Boom = new DoubleSolenoid(
    PneumaticsModuleType.CTREPCM,
    0,
    1
  );

  private final DoubleSolenoid p_Arm = new DoubleSolenoid(
    PneumaticsModuleType.CTREPCM,
    2,
    3
  );

  private final DoubleSolenoid p_Hand = new DoubleSolenoid(
    PneumaticsModuleType.CTREPCM,
    4,
    5
  );

  private MecanumDrive m_robotDrive;
  XboxController js_Driver = new XboxController(0);
  XboxController js_Operator = new XboxController(1);

  @Override
  public void robotInit() {
    WPI_VictorSPX frontLeft = new WPI_VictorSPX(nFrontLeft);
    WPI_VictorSPX rearLeft = new WPI_VictorSPX(nRearLeft);
    WPI_VictorSPX frontRight = new WPI_VictorSPX(nFrontRight);
    WPI_VictorSPX rearRight = new WPI_VictorSPX(nRearRight);

    // Invert the right side motors.
    // You may need to change or remove this to match your robot.
    frontRight.setInverted(true);
    rearRight.setInverted(true);

    m_robotDrive = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);
  }

  @Override
  public void teleopPeriodic() {
    // Use the joystick Y axis for forward movement, X axis for lateral
    // movement, and Z axis for rotation.
    m_robotDrive.driveCartesian(
      -js_Driver.getRawAxis(1),
      -js_Driver.getRawAxis(1),
      -js_Driver.getRawAxis(3)
    );

    if (js_Operator.getRawButton(1)) {
      p_Boom.set(DoubleSolenoid.Value.kForward);
    } else if (js_Operator.getRawButton(2)) {
      p_Boom.set(DoubleSolenoid.Value.kReverse);
    }

    if (js_Operator.getRawButton(3)) {
      p_Arm.set(DoubleSolenoid.Value.kForward);
    } else if (js_Operator.getRawButton(4)) {
      p_Arm.set(DoubleSolenoid.Value.kReverse);
    }

    if (js_Operator.getRawButton(5)) {
      p_Hand.set(DoubleSolenoid.Value.kForward);
    } else if (js_Operator.getRawButton(6)) {
      p_Hand.set(DoubleSolenoid.Value.kReverse);
    }
  }
}
