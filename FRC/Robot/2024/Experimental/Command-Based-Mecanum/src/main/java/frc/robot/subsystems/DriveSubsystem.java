// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {

  private final CANSparkMax m_backLeft = new CANSparkMax(
    Constants.MotorConstants.can_id_backLeft,
    com.revrobotics.CANSparkLowLevel.MotorType.kBrushless
  );
  private final CANSparkMax m_backRight = new CANSparkMax(
    Constants.MotorConstants.can_id_backRight,
    com.revrobotics.CANSparkLowLevel.MotorType.kBrushless
  );
  private final CANSparkMax m_frontLeft = new CANSparkMax(
    Constants.MotorConstants.can_id_frontLeft,
    com.revrobotics.CANSparkLowLevel.MotorType.kBrushless
  );
  private final CANSparkMax m_frontRight = new CANSparkMax(
    Constants.MotorConstants.can_id_frontRight,
    com.revrobotics.CANSparkLowLevel.MotorType.kBrushless
  );
  RelativeEncoder m_backLeftEncoder;
  RelativeEncoder m_backRightEncoder;
  RelativeEncoder m_frontLeftEncoder;
  RelativeEncoder m_frontRightEncoder;

  /** Creates a new ExampleSubsystem. */
  public DriveSubsystem() {
    // Invert the right side motors
    m_frontRight.setInverted(false);
    m_backRight.setInverted(true);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(() -> {
      /* one-time action goes here */
    });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber(
      "Drive encoder value",
      m_frontLeftEncoder.getPosition()
    );
  }

  public void setMotors(
    double backLeftSpeed,
    double backRightSpeed,
    double frontLeftSpeed,
    double frontRightSpeed
  ) {
    m_backLeft.set(frontRightSpeed);
    m_backRight.set(backRightSpeed);
    m_frontLeft.set(frontLeftSpeed);
    m_frontRight.set(backLeftSpeed);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
