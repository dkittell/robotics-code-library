/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

// region imports
import frc.robot.Variables;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// region Gyro
import edu.wpi.first.wpilibj.AnalogGyro;
// endregion Gyro

// region csv reader
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.util.ArrayList;
// endregion csv reader

//region Venom Code Imports
import com.playingwithfusion.CANVenom;
import com.playingwithfusion.CANVenom.BrakeCoastMode;
import com.playingwithfusion.CANVenom.ControlMode;
//endregion Venom Code Imports

// endregion imports

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  // region Venom
  public com.playingwithfusion.CANVenom m_frontLeft = new CANVenom(Variables.m_frontLeft);
  public com.playingwithfusion.CANVenom m_rearLeft = new CANVenom(Variables.m_rearLeft);
  public com.playingwithfusion.CANVenom m_frontRight = new CANVenom(Variables.m_frontRight);
  public com.playingwithfusion.CANVenom m_rearRight = new CANVenom(Variables.m_rearRight);
  public com.playingwithfusion.CANVenom m_Shooter = new CANVenom(Variables.m_Shooter);

  /**
   * Reset the drive encoders to 0
   */
  public void resetEncoders() {
    m_frontLeft.setPosition(0);
    m_frontRight.setPosition(0);
    m_rearLeft.setPosition(0);
    m_rearRight.setPosition(0);
    m_frontLeft.resetPosition();
    m_frontRight.resetPosition();
    m_rearLeft.resetPosition();
    m_rearRight.resetPosition();
  }

  // endregion Venom

  DifferentialDrive m_drive = new DifferentialDrive(m_frontLeft, m_rearLeft);

  private final Joystick js1 = new Joystick(0);

  // region Gyro
  private static final int kGyroPort = 0;
  private final AnalogGyro m_gyro = new AnalogGyro(kGyroPort);
  private static final double kAngleSetpoint = 0.0;
  private static final double kP = 0.005; // propotional turning constant
  // gyro calibration constant, may need to be adjusted;
  // gyro value of 360 is set to correspond to one full revolution
  private static final double kVoltsPerDegreePerSecond = 0.0128;
  // endregion Gyro

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    m_rearLeft.follow(m_frontLeft);
    m_rearRight.follow(m_frontRight);

    m_gyro.setSensitivity(kVoltsPerDegreePerSecond);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {

    SmartDashboard.putNumber("m_frontLeft", m_frontLeft.getPosition());
    SmartDashboard.putNumber("m_frontRight", m_frontRight.getPosition());
    SmartDashboard.putNumber("m_rearLeft", m_rearLeft.getPosition());
    SmartDashboard.putNumber("m_rearRight", m_rearRight.getPosition());

    double turningValue = (kAngleSetpoint - m_gyro.getAngle()) * kP;

    // System.out.println("turningValue " + turningValue);
    SmartDashboard.putNumber("Gyro Turning Point", turningValue);

  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable chooser
   * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
   * remove all of the chooser code and uncomment the getString line to get the
   * auto name from the text box below the Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure below with additional strings. If using the SendableChooser
   * make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
    resetEncoders();

    m_frontLeft.setBrakeCoastMode(BrakeCoastMode.Brake);
    m_frontRight.setBrakeCoastMode(BrakeCoastMode.Brake);
    m_rearLeft.setBrakeCoastMode(BrakeCoastMode.Brake);
    m_rearRight.setBrakeCoastMode(BrakeCoastMode.Brake);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  @Override
  public void teleopInit() {
    // TODO Auto-generated method stub
    super.teleopInit();
    resetEncoders();
  }

  @Override
  public void teleopPeriodic() {

    m_drive.arcadeDrive(js1.getRawAxis(1), js1.getRawAxis(2)); // Regular Controller

  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
