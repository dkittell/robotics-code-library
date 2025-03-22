// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.GyroBase;
import edu.wpi.first.wpilibj.AnalogGyro;
// import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;

import frc.robot.subsystems.Drivetrain;

/** Add your docs here. */
public class RobotGyro {
    private static final int kGyroPort = 0;
    public static final double kAngleSetpoint = 0.0;
    public static final double kP = 0.005; // propotional turning constant
    public final static AnalogGyro m_gyro = new AnalogGyro(kGyroPort);
  
    // gyro calibration constant, may need to be adjusted;
    // gyro value of 360 is set to correspond to one full revolution
    private static final double kVoltsPerDegreePerSecond = 0.0128;


}