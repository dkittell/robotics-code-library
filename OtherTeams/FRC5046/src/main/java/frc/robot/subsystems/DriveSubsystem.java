// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.CANSparkMax;
//import com.revrobotics.RelativeEncoder;
//import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  // The motors on the left side of the drive.
  private final static CANSparkMax m_leftLeader = new CANSparkMax(DriveConstants.kLeftMotor1Port, MotorType.kBrushless);
  private final static CANSparkMax m_leftFollower = new CANSparkMax(DriveConstants.kLeftMotor2Port,
      MotorType.kBrushless);

  // The motors on the right side of the drive.
  private final static CANSparkMax m_rightLeader = new CANSparkMax(DriveConstants.kRightMotor1Port,
      MotorType.kBrushless);
  private final static CANSparkMax m_rightFollower = new CANSparkMax(DriveConstants.kRightMotor2Port,
      MotorType.kBrushless);

  // private final static RelativeEncoder m_rightEncoder =
  // m_rightLeader.getEncoder();
  // private final static RelativeEncoder m_leftEncoder =
  // m_leftLeader.getEncoder();

  // private final static SparkPIDController leftDrivePIDController =
  // m_leftLeader.getPIDController();
  // private final static SparkPIDController rightDrivePIDController =
  // m_rightLeader.getPIDController();

  // The robot's drive
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftLeader::set, m_rightLeader::set);

  private final Encoder m_leftEncoder = new Encoder(
      DriveConstants.kLeftEncoderPorts[0],
      DriveConstants.kLeftEncoderPorts[1],
      DriveConstants.kLeftEncoderReversed);

  // The right-side drive encoder
  private final Encoder m_rightEncoder = new Encoder(
      DriveConstants.kRightEncoderPorts[0],
      DriveConstants.kRightEncoderPorts[1],
      DriveConstants.kRightEncoderReversed);
  Pigeon2 m_gyro = new Pigeon2(0);

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {

    m_leftLeader.restoreFactoryDefaults();
    m_leftFollower.restoreFactoryDefaults();
    // m_leftFollower1.restoreFactoryDefaults();
    m_rightLeader.restoreFactoryDefaults();
    m_rightFollower.restoreFactoryDefaults();
    // m_rightFollower1.restoreFactoryDefaults();
    // rightDrivePIDController.setFeedbackDevice( m_rightEncoder );
    // leftDrivePIDController.setFeedbackDevice(m_leftEncoder);

    SendableRegistry.addChild(m_drive, m_leftLeader);
    SendableRegistry.addChild(m_drive, m_rightLeader);

    m_leftFollower.follow(m_leftLeader);
    m_rightFollower.follow(m_rightLeader);

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightLeader.setInverted(true);
    // m_rightEncoder.setInverted(true);
    m_leftLeader.setIdleMode(DriveConstants.DriveMotorIdleMode);
    m_rightLeader.setIdleMode(DriveConstants.DriveMotorIdleMode);

    // leftDrivePIDController.setP(DriveConstants.LDrivingP);
    // leftDrivePIDController.setI(DriveConstants.LDrivingI);
    // leftDrivePIDController.setD(DriveConstants.LDrivingD);
    // leftDrivePIDController.setFF(DriveConstants.LDrivingFF);
    // leftDrivePIDController.setOutputRange(DriveConstants.LDrivingMinOutput,DriveConstants.LDrivingMaxOutput);

    // rightDrivePIDController.setP(DriveConstants.RDrivingP);
    // rightDrivePIDController.setI(DriveConstants.RDrivingI);
    // rightDrivePIDController.setD(DriveConstants.RDrivingD);
    // rightDrivePIDController.setFF(DriveConstants.RDrivingFF);
    // rightDrivePIDController.setOutputRange(DriveConstants.RDrivingMinOutput,DriveConstants.RDrivingMaxOutput);

    // Sets the distance per pulse for the encoders
    m_leftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    m_rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    // m_leftEncoder.setPositionConversionFactor(DriveConstants.OneInches);

  }

  public void zeroHeading() {
    m_gyro.reset();

  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    // rot = .5;
    m_drive.arcadeDrive(fwd, rot);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public Encoder getLeftEncoder() {
    return m_leftEncoder;
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public Encoder getRightEncoder() {
    return m_rightEncoder;
  }

  /**
   * Gets the average distance of the TWO encoders.
   *
   * @return the average of the TWO encoder readings
   */
  public double getAverageEncoderDistance() {
    return (m_leftEncoder.getDistance() + m_rightEncoder.getDistance()) / 2.0;
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from 180 to 180
   */
  public double getHeading() {
    return Math.IEEEremainder(m_gyro.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more
   * slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    // Publish encoder distances to telemetry.
    builder.addDoubleProperty("leftDistance", m_leftEncoder::getDistance, null);
    builder.addDoubleProperty("rightDistance", m_rightEncoder::getDistance, null);
  }

  // }
  @Override
  public void periodic() {
    // System.out.print("Right:");
    // System.out.print(m_rightEncoder.getPosition());

    // System.out.print("Left:");
    /// System.out.print(m_leftEncoder.getPosition());
    // SmartDashboard.putNumber("REncoder Position", m_leftEncoder.);
    // SmartDashboard.putNumber("LEncoder Position", m_leftEncoder.getPosition());
    // SmartDashboard.putNumber("rEncoder Velocity", m_rightEncoder.getVelocity());
    // SmartDashboard.putNumber("LEncoder velocity",m_leftEncoder.getVelocity());

  }

  public Command Drivecommand() {
    return this.runOnce(() -> m_drive.arcadeDrive(0.5, 0));
  }

  public Command DriveBackcommand() {
    return this.runOnce(() -> m_drive.arcadeDrive(-0.5, 0));
  }

  public Command Stopcommand() {
    return this.runOnce(() -> m_drive.arcadeDrive(0, 0));
  }
}