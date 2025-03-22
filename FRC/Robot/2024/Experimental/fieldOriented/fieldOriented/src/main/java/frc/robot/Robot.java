// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder; // Specifically needed for encoders
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.hal.ConstantsJNI;
import edu.wpi.first.math.filter.SlewRateLimiter;
import frc.robot.constants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  
  private Joystick js_Driver = new Joystick(constants.js_Driver);
  private Joystick js_Operator = new Joystick(constants.js_Operator);

  // Declare Motors
  // TO DO: Fix ID's in constants
  public CANSparkMax m_frontLeft = new CANSparkMax(
    constants.can_id_frontLeft,
    MotorType.kBrushless);
public CANSparkMax m_frontRight = new CANSparkMax(
    constants.can_id_frontRight,
    MotorType.kBrushless);
public CANSparkMax m_rearLeft = new CANSparkMax(
    constants.can_id_rearLeft,
    MotorType.kBrushless);
public CANSparkMax m_rearRight = new CANSparkMax(
    constants.can_id_rearRight,
    MotorType.kBrushless);
public MecanumDrive m_robotDrive = new MecanumDrive(
    m_frontLeft,
    m_rearLeft,
    m_frontRight,
    m_rearRight);


  // Declare Encoders
  private RelativeEncoder m_frontLeftEncoder;
  private RelativeEncoder m_rearLeftEncoder;
  private RelativeEncoder m_frontRightEncoder;
  private RelativeEncoder m_rearRightEncoder;

  // Get the Gyroscope
  public static final ADIS16470_IMU imu = new ADIS16470_IMU();

  double joystick_final;
  double Joystick_val;

  // Ramp the speed of the drive motors
  SlewRateLimiter driveX = new SlewRateLimiter(0.6);
  SlewRateLimiter driveY = new SlewRateLimiter(1);
  SlewRateLimiter driveZ = new SlewRateLimiter(1);
  
  // TO DO: Check ID's in constants on these below 
  PneumaticHub m_ph = new PneumaticHub(5);
  DoubleSolenoid p_ds_Drivetrain = m_ph.makeDoubleSolenoid(
      constants.p_ds_Drivetrain_Forward_id,
      constants.p_ds_Drivetrain_Reverse_id);
  DoubleSolenoid p_ds_Claw = m_ph.makeDoubleSolenoid(
      constants.p_ds_Claw_Forward_id,
      constants.p_ds_Claw_Reverse_id);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    m_frontLeft.setInverted(false);
    m_frontRight.setInverted(true);
    m_rearLeft.setInverted(false);
    m_rearRight.setInverted(true);

    // Get the encoders for each wheel
    m_frontLeftEncoder = m_frontLeft.getEncoder();
    m_frontRightEncoder = m_frontRight.getEncoder();
    m_rearLeftEncoder = m_rearLeft.getEncoder();
    m_rearRightEncoder = m_rearRight.getEncoder();

    // Set the encoders to 0 for init
    m_frontLeftEncoder.setPosition(0);
    m_frontRightEncoder.setPosition(0);
    m_rearLeftEncoder.setPosition(0);
    m_rearRightEncoder.setPosition(0);

    m_robotDrive.setDeadband(0.2);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

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
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
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

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    m_frontLeftEncoder.setPosition(0);
    m_frontRightEncoder.setPosition(0);
    m_rearLeftEncoder.setPosition(0);
    m_rearRightEncoder.setPosition(0);

    // Make sure the motors coast
    m_frontLeft.setIdleMode(IdleMode.kCoast);
    m_frontRight.setIdleMode(IdleMode.kCoast);
    m_rearLeft.setIdleMode(IdleMode.kCoast);
    m_rearRight.setIdleMode(IdleMode.kCoast);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    if (js_Driver.getRawButtonPressed(constants.btnD_Back)) {
      imu.reset();
    }
    double speedY = -js_Driver.getRawAxis(constants.axisD_lUpDown) * (0.8);
    double speedX = js_Driver.getRawAxis(constants.axisD_lLeftRight) * (0.8);
    double speedZ =  js_Driver.getRawAxis(constants.axisD_rLeftRight) * (0.6);

    double botHeading = imu.getXComplementaryAngle();

    double rotX = speedX * Math.cos(-botHeading) - speedY * Math.sin(-botHeading);
    double rotY = speedX * Math.sin(-botHeading) + speedY * Math.cos(-botHeading);

    rotX = rotX * 1.1;

    double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(speedZ), 1);
    double frontLeftPower = (rotY + rotX + speedZ) / denominator;
    double backLeftPower = (rotY - rotX + speedZ) / denominator;
    double frontRightPower = (rotY - rotX - speedZ) / denominator;
    double backRightPower = (rotY + rotX - speedZ) / denominator;

    m_frontLeft.set(frontLeftPower);
    m_rearLeft.set(backLeftPower);
    m_frontRight.set(frontRightPower);
    m_rearRight.set(backRightPower);

    if (js_Driver.getRawButtonPressed(constants.btnXB_RB)) {
      p_ds_Drivetrain.set(DoubleSolenoid.Value.kForward);
    }
if (js_Driver.getRawButtonPressed(constants.btnXB_LB)) {
      p_ds_Drivetrain.set(DoubleSolenoid.Value.kReverse);
    }
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
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
