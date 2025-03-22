// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import com.playingwithfusion.CANVenom;
import com.playingwithfusion.CANVenom.BrakeCoastMode;
import com.playingwithfusion.CANVenom.ControlMode;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import frc.robot.frc3535_Constants;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/** Represents a differential drive style drivetrain. */
public class Drivetrain {
  public static final double kMaxSpeed = 3.0; // meters per second
  public static final double kMaxAngularSpeed = 2 * Math.PI; // one rotation per second

  private static final double kTrackWidth = 0.381 * 2; // meters
  private static final double kWheelRadius = 0.0508; // meters
  private static final int kEncoderResolution = 4096;

  // private final SpeedController m_leftLeader = new PWMVictorSPX(1);
  // private final SpeedController m_leftFollower = new PWMVictorSPX(2);
  // private final SpeedController m_rightLeader = new PWMVictorSPX(3);
  // private final SpeedController m_rightFollower = new PWMVictorSPX(4);

  // private final Encoder m_leftEncoder = new Encoder(0, 1);
  // private final Encoder m_rightEncoder = new Encoder(2, 3);

  // private final SpeedControllerGroup m_leftGroup = new
  // SpeedControllerGroup(m_leftLeader, m_leftFollower);
  // private final SpeedControllerGroup m_rightGroup = new
  // SpeedControllerGroup(m_rightLeader, m_rightFollower);
  // region Venom Motors

  public static com.playingwithfusion.CANVenom m_frontLeft = new CANVenom(frc3535_Constants.m_frontLeft);
  public static com.playingwithfusion.CANVenom m_rearLeft = new CANVenom(frc3535_Constants.m_rearLeft);
  public static com.playingwithfusion.CANVenom m_frontRight = new CANVenom(frc3535_Constants.m_frontRight);
  public static com.playingwithfusion.CANVenom m_rearRight = new CANVenom(frc3535_Constants.m_rearRight);

  public static SpeedControllerGroup m_leftGroup = new SpeedControllerGroup(m_frontLeft, m_rearLeft);
  public static SpeedControllerGroup m_rightGroup = new SpeedControllerGroup(m_frontRight, m_rearRight);
  public static DifferentialDrive m_drive = new DifferentialDrive(m_leftGroup, m_rightGroup);

  // endregion Venom Motors

  public static void motorSafety(boolean msState) {
    m_frontLeft.setSafetyEnabled(msState);
    m_frontRight.setSafetyEnabled(msState);
    m_rearLeft.setSafetyEnabled(msState);
    m_rearRight.setSafetyEnabled(msState);
  }

  public static void resetEncoders() {
    m_frontLeft.setPosition(0);
    m_frontRight.setPosition(0);
    m_rearLeft.setPosition(0);
    m_rearRight.setPosition(0);
    m_frontLeft.resetPosition();
    m_frontRight.resetPosition();
    m_rearLeft.resetPosition();
    m_rearRight.resetPosition();
  }

  public static void enableMotors() {
    m_frontLeft.enable();
    m_frontRight.enable();
    m_rearLeft.enable();
    m_rearRight.enable();
    m_rearLeft.follow(m_frontLeft);
    m_rearRight.follow(m_frontRight);

  }

  public static void venomBrakeMode(int nMode) {
    if (nMode == 1) {
      m_frontLeft.setBrakeCoastMode(BrakeCoastMode.Brake);
      m_frontRight.setBrakeCoastMode(BrakeCoastMode.Brake);
      m_rearLeft.setBrakeCoastMode(BrakeCoastMode.Brake);
      m_rearRight.setBrakeCoastMode(BrakeCoastMode.Brake);
    }
    if (nMode == 2) {
      m_frontLeft.setBrakeCoastMode(BrakeCoastMode.Coast);
      m_frontRight.setBrakeCoastMode(BrakeCoastMode.Coast);
      m_rearLeft.setBrakeCoastMode(BrakeCoastMode.Coast);
      m_rearRight.setBrakeCoastMode(BrakeCoastMode.Coast);
    }
  }

  private final AnalogGyro m_gyro = new AnalogGyro(0);

  private final PIDController m_leftPIDController = new PIDController(1, 0, 0);
  private final PIDController m_rightPIDController = new PIDController(1, 0, 0);

  private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(kTrackWidth);

  private final DifferentialDriveOdometry m_odometry;

  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1, 3);

  /**
   * Constructs a differential drive object. Sets the encoder distance per pulse
   * and resets the gyro.
   */
  public Drivetrain() {
    m_gyro.reset();

    m_rearLeft.follow(m_frontLeft);
    m_rearRight.follow(m_frontRight);

    // Set the distance per pulse for the drive encoders. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    // m_leftEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius /
    // kEncoderResolution);
    // m_rightEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius /
    // kEncoderResolution);

    resetEncoders();

    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());
  }

  /**
   * Sets the desired wheel speeds.
   *
   * @param speeds The desired wheel speeds.
   */
  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    final double leftFeedforward = m_feedforward.calculate(speeds.leftMetersPerSecond);
    final double rightFeedforward = m_feedforward.calculate(speeds.rightMetersPerSecond);

    final double leftOutput = m_leftPIDController.calculate(m_leftEncoder.getRate(), speeds.leftMetersPerSecond);
    final double rightOutput = m_rightPIDController.calculate(m_rightEncoder.getRate(), speeds.rightMetersPerSecond);
    m_leftGroup.setVoltage(leftOutput + leftFeedforward);
    m_rightGroup.setVoltage(rightOutput + rightFeedforward);
  }

  /**
   * Drives the robot with the given linear velocity and angular velocity.
   *
   * @param xSpeed Linear velocity in m/s.
   * @param rot    Angular velocity in rad/s.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double rot) {
    var wheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot));
    setSpeeds(wheelSpeeds);
  }

  /** Updates the field-relative position. */
  public void updateOdometry() {
    m_odometry.update(m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
  }

  /**
   * Resets the field-relative position to a specific location.
   *
   * @param pose The position to reset to.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
  }

  /**
   * Returns the pose of the robot.
   *
   * @return The pose of the robot.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }
}
