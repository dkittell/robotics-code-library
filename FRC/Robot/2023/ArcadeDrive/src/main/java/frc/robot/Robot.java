// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Rev Robotics - https://software-metadata.revrobotics.com/REVLib-2023.json
// CTRE - https://maven.ctr-electronics.com/release/com/ctre/phoenix/Phoenix-frc2022-latest.json
// Venom - https://www.playingwithfusion.com/frc/playingwithfusion2023.json

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This is a demo program showing the use of the DifferentialDrive class. Runs the motors with
 * arcade steering.
 */
public class Robot extends TimedRobot {

  ADIS16470_IMU imu = new ADIS16470_IMU();
  AnalogGyro gyro = new AnalogGyro(1);

  String m_autoSelected;

  int kGyroPort = 0;
  AnalogGyro m_gyro = new AnalogGyro(kGyroPort);
  CANSparkMax m_frontLeft = new CANSparkMax(1, MotorType.kBrushless);
  CANSparkMax m_frontRight = new CANSparkMax(3, MotorType.kBrushless);
  CANSparkMax m_rearLeft = new CANSparkMax(2, MotorType.kBrushless);
  CANSparkMax m_rearRight = new CANSparkMax(4, MotorType.kBrushless);

  Joystick m_stick = new Joystick(0);
  SendableChooser<String> m_chooser = new SendableChooser<>();
  String kCustomAuto = "My Auto";
  String kDefaultAuto = "Default";
  double kAngleSetpoint = 0.0;
  double kP = 0.005; // propotional turning constant
  double kPitch = 0.10;
  double kPitchSetpoint = 0.0;
  double kVoltsPerDegreePerSecond = 0.0128;
  double avgDistance = 0;
  Timer rTimer = new Timer();
  MotorControllerGroup m_left = new MotorControllerGroup(
    m_frontLeft,
    m_rearLeft
  );
  MotorControllerGroup m_right = new MotorControllerGroup(
    m_frontRight,
    m_rearRight
  );
  RelativeEncoder m_frontLeftEncoder = m_frontLeft.getEncoder();
  RelativeEncoder m_frontRightEncoder = m_frontRight.getEncoder();
  RelativeEncoder m_rearLeftEncoder = m_rearLeft.getEncoder();
  RelativeEncoder m_rearRightEncoder = m_rearRight.getEncoder();
  DifferentialDrive m_robotDrive = new DifferentialDrive(m_left, m_right);

  @Override
  public void robotInit() {
    m_gyro.setSensitivity(kVoltsPerDegreePerSecond);
    gyro.calibrate();
    gyro.reset();
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_frontRight.setInverted(true);
    m_rearRight.setInverted(true);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("gyro", imu.getYComplementaryAngle());
    SmartDashboard.putNumber("GyroAngle", imu.getAngle());
    SmartDashboard.putNumber("FL Encoder", m_frontLeftEncoder.getPosition());
    SmartDashboard.putNumber("Timer", rTimer.get());
  }

  public void driveStraight(int nYaw, double dDrive) {
    double turningValue = (kAngleSetpoint - imu.getAngle()) * kP;

    // Drive Straight
    // if (imu.getAngle() > nYaw) {
    m_robotDrive.arcadeDrive(dDrive, turningValue);
    // } else if (imu.getAngle() < -nYaw) {
    // m_robotDrive.arcadeDrive(dDrive, -dTurn);
    // }
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    // m_frontLeftEncoder.setPosition(0);
    // m_frontRightEncoder.setPosition(0);
    // m_rearLeftEncoder.setPosition(0);
    // m_rearRightEncoder.setPosition(0);

    m_frontLeft.setIdleMode(IdleMode.kBrake);
    m_frontRight.setIdleMode(IdleMode.kBrake);
    m_rearLeft.setIdleMode(IdleMode.kBrake);
    m_rearRight.setIdleMode(IdleMode.kBrake);
    // Set properties of drive
    // m_robotDrive.setMaxOutput(0.40);
    gyro.reset();
    rTimer.start();
  }

  @Override
  public void autonomousPeriodic() {
    // region Time Based Auton
    // if (rTimer.get() < 8.0) {
    // driveStraight(5, 0.7);
    // }
    // if (rTimer.get() > 8.0) {
    // m_robotDrive.arcadeDrive(0, 0.9);
    // }
    // if (rTimer.get() > 9) {
    // m_robotDrive.arcadeDrive(0, 0);
    // }
    // endregion Time Based Auton

    double turningValue = (kAngleSetpoint - m_gyro.getAngle()) * kP;
    double avgDistance =
      m_frontLeftEncoder.getPosition() + m_frontRightEncoder.getPosition() / 2;
    if (m_frontLeftEncoder.getPosition() < 50) {
      m_robotDrive.arcadeDrive(0.7, -turningValue);
    } else {
      m_robotDrive.arcadeDrive(0, 0);
    }
  }

  @Override
  public void teleopInit() {
    m_frontLeft.setIdleMode(IdleMode.kCoast);
    m_rearLeft.setIdleMode(IdleMode.kCoast);
    m_frontRight.setIdleMode(IdleMode.kCoast);
    m_rearRight.setIdleMode(IdleMode.kCoast);
    m_robotDrive.setDeadband(0.15);
    m_robotDrive.setMaxOutput(0.4);
  }

  @Override
  public void teleopPeriodic() {
    // Drive with arcade drive.
    // That means that the Y axis drives forward
    // and backward, and the X turns left and right.
    // m_robotDrive.arcadeDrive(-m_stick.getRawAxis(2), m_stick.getRawAxis(1));
    // double turningValue = (kAngleSetpoint - gyro.getAngle()) * kP;
    // Invert the direction of the turn if we are going backwards
    if (m_stick.getRawButton(6)) {
      double turningValue = (kAngleSetpoint - imu.getAngle()) * kP;
      turningValue = Math.copySign(turningValue, m_stick.getY());
      m_robotDrive.arcadeDrive(-m_stick.getY(), turningValue);
    } else {
      m_robotDrive.arcadeDrive(m_stick.getRawAxis(1), -m_stick.getRawAxis(2));
    }
    // if (m_stick.getRawButton(1)) {
    //   m_Intake.set(0.4);
    // } else if (m_stick.getRawButton(2)) {
    //   m_Intake.set(-0.4);
    // } else {
    //   m_Intake.set(0);
    // }
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    m_frontLeftEncoder.setPosition(0);
    m_frontRightEncoder.setPosition(0);
    m_rearLeftEncoder.setPosition(0);
    m_rearRightEncoder.setPosition(0);
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
