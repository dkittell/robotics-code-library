// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.subsystems.RobotGyro;

import com.playingwithfusion.CANVenom;
import com.playingwithfusion.CANVenom.BrakeCoastMode;
import com.playingwithfusion.CANVenom.ControlMode;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;


/** Represents a differential drive style drivetrain. */
public class Drivetrain {

  // region Venom Motors
  public static com.playingwithfusion.CANVenom m_frontLeft = new CANVenom(Constants.m_frontLeft);
  public static com.playingwithfusion.CANVenom m_rearLeft = new CANVenom(Constants.m_rearLeft);
  public static com.playingwithfusion.CANVenom m_frontRight = new CANVenom(Constants.m_frontRight);
  public static com.playingwithfusion.CANVenom m_rearRight = new CANVenom(Constants.m_rearRight);
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


  /**
   * Constructs a differential drive object. Sets the encoder distance per pulse
   * and resets the gyro.
   */
  public Drivetrain() {
    // m_gyro.reset();

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

    // m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());

     // Reset the drivetrain's odometry to the starting pose of the trajectory.
    //  Drivetrain.m_drive.resetOdometry(m_trajectory.getInitialPose());
  }

  
}