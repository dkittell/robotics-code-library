// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// region Imports/Libraries
// Rev Robotics - https://software-metadata.revrobotics.com/REVLib-2023.json
// CTRE - https://maven.ctr-electronics.com/release/com/ctre/phoenix/Phoenix-frc2022-latest.json


// region CTRE Library
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
// endregion CTRE Library


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.ADIS16470_IMU; // Gyro
import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj.AnalogGyro;


import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


// region Constants
import frc.robot.constants;

// endregion Constants

// endregion Imports/Libraries

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private static final double kAngleSetpoint = 0.0;
  private static final double kP = 0.005; // propotional turning constant

   // gyro calibration constant, may need to be adjusted;
  // gyro value of 360 is set to correspond to one full revolution
  private static final double kVoltsPerDegreePerSecond = 0.0128;

  private static final int kGyroPort = 0;
  private final AnalogGyro m_gyro = new AnalogGyro(kGyroPort);

  public static Timer rTimer = new Timer();
  public DigitalInput ls_elbow_low = new DigitalInput(0);

  private static final String sDefaultAuto = "Default";
  private static final String sDoNothing = "Do Nothing";
  private static final String sDrivePastLineAuto = "Drive Initiation Line";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  // region Drivetrain Constants
  public int m_frontLeft_id = 0;
  public int m_rearLeft_id = 2;
  public int m_frontRight_id = 1;
  public int m_rearRight_id = 3;
  public int js_Driver_id = 0;

  // region Drivetrain Initialization
  public PWMTalonSRX m_frontLeft = new PWMTalonSRX(m_frontLeft_id);
  public PWMTalonSRX m_rearLeft = new PWMTalonSRX(m_rearLeft_id);
  public PWMTalonSRX m_frontRight = new PWMTalonSRX(m_frontRight_id);
  public PWMTalonSRX m_rearRight = new PWMTalonSRX(m_rearRight_id);

  private MecanumDrive m_robotDrive;

  // endregion Drivetrain Constants

  // region NEO
  public CANSparkMax m_testNeo = new CANSparkMax(5, MotorType.kBrushless);
  private RelativeEncoder m_testNeoEncoder;
  // endregion NEO

  private Joystick js_Driver;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    m_gyro.setSensitivity(kVoltsPerDegreePerSecond);

    m_testNeoEncoder = m_testNeo.getEncoder();
    m_chooser.addOption("Default Auto", sDefaultAuto);
    m_chooser.addOption("Do Nothing", sDoNothing);
    m_chooser.setDefaultOption("Drive Initiation Line", sDrivePastLineAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    // Invert the right side motors.
    // You may need to change or remove this to match your robot.
    m_frontRight.setInverted(true);
    m_rearRight.setInverted(true);
    m_frontLeft.setInverted(false);
    m_rearLeft.setInverted(false);

    m_robotDrive =
      new MecanumDrive(m_frontLeft, m_rearLeft, m_frontRight, m_rearRight);

    js_Driver = new Joystick(js_Driver_id);
    // endregion Drivetrain Initialization

    // CameraServer.startAutomaticCapture();
    // CameraServer camera01 = CameraServer.getInstance();
    // camera01.startAutomaticCapture(0);
    // UsbCamera camera01 = CameraServer.startAutomaticCapture();
    // camera01.setFPS(15);
    // camera01.setResolution(640, 480);

    // mjpegServer = new MjpegServer("Camera 0", 1181);
    // mjpegServer.setCompression(50);
    // mjpegServer.setSource(camera01);
    // UsbCamera camera02 = CameraServer.startAutomaticCapture();
    // camera02.setFPS(20);
    // camera02.setResolution(800, 600);

  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    SmartDashboard.putBoolean("Elbow Low", ls_elbow_low.get());
    SmartDashboard.putBoolean("Enabled", isEnabled());
    SmartDashboard.putNumber("Timer", rTimer.get());

    double turningValue = (kAngleSetpoint - m_gyro.getAngle()) * kP;

    SmartDashboard.putNumber("Encoder Count", m_testNeoEncoder.getPosition());
    SmartDashboard.putNumber("Turning Value", turningValue);
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different
   * autonomous modes using the dashboard. The sendable chooser code works with
   * the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the
   * chooser code and
   * uncomment the getString line to get the auto name from the text box below the
   * Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure
   * below with additional strings. If using the SendableChooser make sure to add
   * them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);

    m_testNeoEncoder.setPosition(0);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case sDefaultAuto:
        // Do something
        break;
      case sDrivePastLineAuto:
        // Drive past initiation line

        // region Encoder Count Run Motor
        if ((m_testNeoEncoder.getPosition() < 42)) {
          // m_robotDrive.driveCartesian(-.5, 0, 0);

          m_testNeo.set(0.5);
        } else {
          // // m_robotDrive.stopMotor();
          // m_frontLeft.set(0);
          // m_frontRight.set(0);
          // m_rearLeft.set(0);
          // m_rearRight.set(0);

          m_testNeo.set(0);
        }
        // endregion Encoder Count Run Motor

        break;
      case sDoNothing:
      default:
        // Do nothing
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    m_testNeoEncoder.setPosition(0);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    double turningValue = (kAngleSetpoint - m_gyro.getAngle()) * kP;
    // m_myRobot.arcadeDrive(-m_joystick.getY(), -turningValue);

    // Use the joystick Y axis for forward movement, X axis for lateral
    // movement, and Z axis for rotation.
    m_robotDrive.driveCartesian(
      -js_Driver.getRawAxis(1),
      -js_Driver.getRawAxis(0),
      js_Driver.getRawAxis(4)
    );
    // First value is forward or reverse
    // Second value is turn left or turn right
    // Third value is to strafe left or strafe right

  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    //region Pivot - Arm Up/Down

 

    if (js_Driver.getRawButton(constants.btnXB_X)) {
      boolean bEnabledLS = ls_elbow_low.get();
      if (bEnabledLS) {
        m_testNeo.set(0.5);
      } else {
        m_testNeo.set(0);
      }
    } else {
      m_testNeo.set(0);
    }
    //endregion Pivot - Arm Up/Down

    // // region Test motor id to name
    // if (js_Driver.getRawButton(constants.btnXB_X)) {
    //   m_frontLeft.set(0.45);
    // } else {
    //   m_frontLeft.set(0);
    // }

    // if (js_Driver.getRawButton(constants.btnXB_Y)) {
    //   m_frontRight.set(0.45);
    // } else {
    //   m_frontRight.set(0);
    // }

    // if (js_Driver.getRawButton(constants.btnXB_A)) {
    //   m_rearLeft.set(0.45);
    // } else {
    //   m_rearLeft.set(0);
    // }

    // if (js_Driver.getRawButton(constants.btnXB_B)) {
    //   m_rearRight.set(0.45);
    // } else {
    //   m_rearRight.set(0);
    // }
    // // endregion Test motor id to name
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
