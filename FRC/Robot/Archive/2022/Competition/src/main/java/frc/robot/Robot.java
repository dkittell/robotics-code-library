// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

// region Imports
// region FRC8767
import frc.robot.Constants;
// endregion FRC8767

import javax.lang.model.util.ElementScanner6;

// region CTRE
// https://maven.ctr-electronics.com/release/com/ctre/phoenix/Phoenix-frc2022-latest.json
import com.ctre.phoenix.motorcontrol.can.*;
// endregion CTRE

// region Rev Robotics
import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder; // Specifically needed for encoders
// endregion Rev Robotics

// region WPILib Imports
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// endregion WPILib
// endregion Imports

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

  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private static MjpegServer mjpegServer;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  // region Drive Train

  private static double error = 0.0;
  private static double turn_power = 0.0;

  // region Drive Motor Assignment
  private static final int m_left01Motor_id = Constants.m_left01Motor;
  private static final int m_right01Motor_id = Constants.m_right01Motor;
  private static final int m_left02Motor_id = Constants.m_left02Motor;
  private static final int m_right02Motor_id = Constants.m_right02Motor;

  private CANSparkMax m_left01Motor;
  private CANSparkMax m_right01Motor;
  private CANSparkMax m_left02Motor;
  private CANSparkMax m_right02Motor;
  // endregion Drive Motor Assignment

  // region Drive Motor Encoder Assignment
  private RelativeEncoder m_left01MotorEncoder;
  private RelativeEncoder m_right01MotorEncoder;
  private RelativeEncoder m_left02MotorEncoder;
  private RelativeEncoder m_right02MotorEncoder;
  // endregion Drive Motor Encoder Assignment

  private DifferentialDrive m_DriveRobotRobot;
  // endregion Drive Train

  // region Ball Shooter
  private CANSparkMax m_intakeShooter = new CANSparkMax(6, MotorType.kBrushless);
  private CANSparkMax m_shooter = new CANSparkMax(7, MotorType.kBrushless);
  // endregion Ball Shooter

  // region Climber Motor Assignment
  private DifferentialDrive m_climb;
  Servo s_RachetPull = new Servo(Constants.s_RatchetPull);
  WPI_TalonSRX m_Climber01 = new WPI_TalonSRX(Constants.m_Climber01);
  WPI_TalonSRX m_Climber02 = new WPI_TalonSRX(Constants.m_Climber02);
  // endregion Climber Motor Assignment

  // region Intake
  Servo s_IntakePush = new Servo(Constants.s_IntakePush);
  WPI_TalonSRX m_Intake = new WPI_TalonSRX(Constants.m_Intake);
  // endregion Intake

  // region Joysticks
  private final Joystick js1 = new Joystick(Constants.js1);
  private final Joystick js2 = new Joystick(Constants.js2);
  // endregion Joysticks

  public static Timer rTimer = new Timer();

  // region Dead Band Variables
  double js1_L_UpDown = 0; // Speed controls up & down
  double js1_R_UpDown = 0; // Speed controls up & down
  double js1_L_LeftRight = 0; // Rotate controls left & right
  double js1_R_LeftRight = 0; // Rotate controls left & right
  double js1_L_TSpeed = 0;
  double js1_R_TSpeed = 0;
  double js2_L_UpDown = 0; // Speed controls up & down
  double js2_R_UpDown = 0; // Speed controls up & down
  double js2_L_LeftRight = 0; // Rotate controls left & right
  double js2_R_LeftRight = 0; // Rotate controls left & right
  double js2_L_TSpeed = 0;
  double js2_R_TSpeed = 0;
  // endregion Dead Band Variables

  // region Shuffleboard
  private PowerDistribution m_pdp = new PowerDistribution();
  private String c_SelectedAuton;
  private String c_SelectedDelay;
  // private String c_SelectedDriveMode;
  private String c_SelectedControllerDriver;
  private String c_SelectedControllerOperator;
  private String c_SelectedDriveYSpeed;
  private String c_SelectedDriveZASpeed;
  private String c_SelectedClimberSpeed;
  private String c_SelectedIntakeShooterSpeed;
  private String c_SelectedShooterSpeed;
  private final SendableChooser<String> c_Auton = new SendableChooser<>();
  private final SendableChooser<String> c_Auton_Delay = new SendableChooser<>();
  private final SendableChooser<String> c_ControllerDriver = new SendableChooser<>();
  private final SendableChooser<String> c_ControllerOperator = new SendableChooser<>();
  // private final SendableChooser<String> c_DriveMode = new SendableChooser<>();
  private final SendableChooser<String> c_DriveYSpeed = new SendableChooser<>();
  private final SendableChooser<String> c_DriveZASpeed = new SendableChooser<>();
  private final SendableChooser<String> c_ClimberSpeed = new SendableChooser<>();
  private final SendableChooser<String> c_IntakeShooterSpeed = new SendableChooser<>();
  private final SendableChooser<String> c_ShooterSpeed = new SendableChooser<>();
  // endregion Shuffleboard

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    // region Shuffleboard
    c_Auton.addOption("Default Auton", "Default");
    c_Auton.setDefaultOption("Position Shooter", "position_shooter");
    c_Auton_Delay.setDefaultOption("None", "None");
    c_Auton_Delay.addOption("1 second", "1");
    c_Auton_Delay.addOption("2 second", "2");
    c_Auton_Delay.addOption("3 second", "3");
    c_Auton_Delay.addOption("4 second", "4");
    c_Auton_Delay.addOption("5 second", "5");
    c_Auton_Delay.addOption("6 second", "6");

    c_ControllerDriver.setDefaultOption("LogitechD", "LogitechD");
    c_ControllerDriver.addOption("LogitechX", "LogitechX");
    c_ControllerDriver.addOption("XBox", "XBox");

    c_ControllerOperator.setDefaultOption("LogitechD", "LogitechD");
    c_ControllerOperator.addOption("LogitechX", "LogitechX");
    c_ControllerOperator.addOption("XBox", "XBox");

    c_DriveYSpeed.addOption(".50", ".50");
    c_DriveYSpeed.addOption(".55", ".55");
    c_DriveYSpeed.addOption(".60", ".60");
    c_DriveYSpeed.addOption(".65", ".65");
    c_DriveYSpeed.addOption(".70", ".70");
    c_DriveYSpeed.addOption(".75", ".75");
    c_DriveYSpeed.setDefaultOption(".80", ".80");
    c_DriveYSpeed.addOption(".85", ".85");
    c_DriveYSpeed.addOption(".90", ".90");

    c_DriveZASpeed.addOption(".50", ".50");
    c_DriveZASpeed.addOption(".55", ".55");
    c_DriveZASpeed.setDefaultOption(".60", ".60");
    c_DriveZASpeed.addOption(".65", ".65");
    c_DriveZASpeed.addOption(".70", ".70");
    c_DriveZASpeed.addOption(".75", ".75");
    c_DriveZASpeed.addOption(".80", ".80");
    c_DriveZASpeed.addOption(".85", ".85");
    c_DriveZASpeed.addOption(".90", ".90");

    c_IntakeShooterSpeed.addOption(".50", ".50");
    c_IntakeShooterSpeed.addOption(".55", ".55");
    c_IntakeShooterSpeed.addOption(".60", ".60");
    c_IntakeShooterSpeed.addOption(".65", ".65");
    c_IntakeShooterSpeed.addOption(".70", ".70");
    c_IntakeShooterSpeed.addOption(".75", ".75");
    c_IntakeShooterSpeed.setDefaultOption(".80", ".80");
    c_IntakeShooterSpeed.addOption(".85", ".85");
    c_IntakeShooterSpeed.addOption(".90", ".90");
    c_IntakeShooterSpeed.addOption("1.00", "1.00");

    c_ShooterSpeed.addOption(".50", ".50");
    c_ShooterSpeed.addOption(".55", ".55");
    c_ShooterSpeed.addOption(".60", ".60");
    c_ShooterSpeed.addOption(".65", ".65");
    c_ShooterSpeed.addOption(".70", ".70");
    c_ShooterSpeed.addOption(".75", ".75");
    c_ShooterSpeed.addOption(".80", ".80");
    c_ShooterSpeed.addOption(".85", ".85");
    c_ShooterSpeed.addOption(".90", ".90");
    c_ShooterSpeed.setDefaultOption("1.00", "1.00");

    c_ClimberSpeed.addOption(".50", ".50");
    c_ClimberSpeed.addOption(".55", ".55");
    c_ClimberSpeed.setDefaultOption(".60", ".60");
    c_ClimberSpeed.addOption(".65", ".65");
    c_ClimberSpeed.addOption(".70", ".70");
    c_ClimberSpeed.addOption(".75", ".75");
    c_ClimberSpeed.addOption(".80", ".80");
    c_ClimberSpeed.addOption(".85", ".85");
    c_ClimberSpeed.addOption(".90", ".90");
    // endregion Shuffleboard

    // region Initialize the drive train
    // c_DriveMode.setDefaultOption("Arcade", "Arcade");
    // c_DriveMode.addOption("Tank", "Tank");

    // region Initialize the Motors
    m_left01Motor = new CANSparkMax(m_left01Motor_id, MotorType.kBrushless);
    m_right01Motor = new CANSparkMax(m_right01Motor_id, MotorType.kBrushless);
    m_left02Motor = new CANSparkMax(m_left02Motor_id, MotorType.kBrushless);
    m_right02Motor = new CANSparkMax(m_right02Motor_id, MotorType.kBrushless);

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_left01Motor.setInverted(false);
    m_left02Motor.setInverted(false);
    m_right01Motor.setInverted(true);
    m_right02Motor.setInverted(true);

    // m_left01Motor.restoreFactoryDefaults();
    // m_right01Motor.restoreFactoryDefaults();
    // m_left02Motor.restoreFactoryDefaults();
    // m_right02Motor.restoreFactoryDefaults();

    m_left02Motor.follow(m_left01Motor);
    m_right02Motor.follow(m_right01Motor);
    // endregion Initialize the Motors

    m_DriveRobotRobot = new DifferentialDrive(m_left01Motor, m_right01Motor);
    // endregion Initialize the drive train

    // region Initialize Encoders
    m_left01MotorEncoder = m_left01Motor.getEncoder();
    m_right01MotorEncoder = m_right01Motor.getEncoder();
    m_left02MotorEncoder = m_left02Motor.getEncoder();
    m_right02MotorEncoder = m_right02Motor.getEncoder();
    m_left01MotorEncoder.setPosition(0);
    m_right01MotorEncoder.setPosition(0);
    m_left02MotorEncoder.setPosition(0);
    m_right02MotorEncoder.setPosition(0);
    // endregion Initialize Encoders

    m_climb = new DifferentialDrive(m_Climber01, m_Climber02);

    // region Camera
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
    // endregion Camera

  }

  @Override
  public void robotPeriodic() {

    // region Shuffleboard
    c_SelectedControllerDriver = c_ControllerDriver.getSelected();
    c_SelectedControllerOperator = c_ControllerOperator.getSelected();
    SmartDashboard.putBoolean("Enabled", isEnabled());
    SmartDashboard.putNumber("Timer", rTimer.get());
    SmartDashboard.putNumber("Voltage", m_pdp.getVoltage());
    SmartDashboard.putData("Auton choices", c_Auton);
    SmartDashboard.putData("Auto Delay", c_Auton_Delay);
    SmartDashboard.putData("Driver Controller", c_ControllerDriver);
    SmartDashboard.putData("Operator Controller", c_ControllerOperator);
    SmartDashboard.putData("Drive Speed", c_DriveYSpeed);
    SmartDashboard.putData("Turning Speed", c_DriveZASpeed);
    SmartDashboard.putData("Climber Speed", c_ClimberSpeed);
    SmartDashboard.putData("Intake Shooter Speed", c_IntakeShooterSpeed);
    SmartDashboard.putData("Shooter Speed", c_ShooterSpeed);
    // SmartDashboard.putData("Drive Mode", c_DriveMode);
    SmartDashboard.putBoolean("Enabled", isEnabled());

    /**
     * Encoder position is read from a RelativeEncoder object by calling the
     * GetPosition() method.
     * 
     * GetPosition() returns the position of the encoder in units of revolutions
     */
    SmartDashboard.putNumber("left01 Position", m_left01MotorEncoder.getPosition());
    SmartDashboard.putNumber("right01 Position", m_right01MotorEncoder.getPosition());
    SmartDashboard.putNumber("left02 Position", m_left02MotorEncoder.getPosition());
    SmartDashboard.putNumber("right02 Position", m_right02MotorEncoder.getPosition());

    /**
     * Encoder velocity is read from a RelativeEncoder object by calling the
     * GetVelocity() method.
     * 
     * GetVelocity() returns the velocity of the encoder in units of RPM
     */
    SmartDashboard.putNumber("left01 Velocity", m_left01MotorEncoder.getVelocity());
    SmartDashboard.putNumber("right01 Velocity", m_right01MotorEncoder.getVelocity());
    SmartDashboard.putNumber("left02 Velocity", m_left02MotorEncoder.getVelocity());
    SmartDashboard.putNumber("right02 Velocity", m_right02MotorEncoder.getVelocity());

    boolean bBattery;
    if (m_pdp.getVoltage() >= 12) {
      bBattery = true;
    } else {
      bBattery = false;
    }
    SmartDashboard.putBoolean("Battery", bBattery);
    c_SelectedClimberSpeed = c_ClimberSpeed.getSelected();
    c_SelectedIntakeShooterSpeed = c_IntakeShooterSpeed.getSelected();
    c_SelectedShooterSpeed = c_ShooterSpeed.getSelected();
    c_SelectedDriveYSpeed = c_DriveYSpeed.getSelected();
    c_SelectedDriveZASpeed = c_DriveZASpeed.getSelected();
    // c_SelectedDriveMode = c_DriveMode.getSelected();
    // endregion Shuffleboard

    // region Deadbands
    // region get raw values
    switch (c_SelectedControllerDriver) {
      case "XBox":
        js1_L_UpDown = js1.getRawAxis(Constants.axisXB_lUpDown); // Y Axis - Up and Down
        js1_L_LeftRight = js1.getRawAxis(Constants.axisXB_lLeftRight); // X Axis - Left and Right
        js1_R_UpDown = -js1.getRawAxis(Constants.axisXB_rUpDown); // Z Rotate - Up and Down
        js1_R_LeftRight = js1.getRawAxis(Constants.axisXB_rLeftRight); // Z Axis - Left and Right
        break;
      case "LogitechD":
        js1_L_UpDown = js1.getRawAxis(Constants.axisD_lUpDown); // Y Axis - Up and Down
        js1_L_LeftRight = js1.getRawAxis(Constants.axisD_lLeftRight); // X Axis - Left and Right
        js1_R_UpDown = -js1.getRawAxis(Constants.axisD_rUpDown); // Z Rotate - Up and Down
        js1_R_LeftRight = js1.getRawAxis(Constants.axisD_rLeftRight); // Z Axis - Left and Right
        break;
      case "LogitechX":
        js1_L_UpDown = js1.getRawAxis(Constants.axisX_lUpDown); // Y Axis - Up and Down
        js1_L_LeftRight = js1.getRawAxis(Constants.axisX_lLeftRight); // X Axis - Left and Right
        js1_R_UpDown = -js1.getRawAxis(Constants.axisX_rUpDown); // Z Rotate - Up and Down
        js1_R_LeftRight = js1.getRawAxis(Constants.axisX_rLeftRight); // Z Axis - Left and Right
        break;
    }
    js2_L_UpDown = js2.getRawAxis(Constants.axisD_lUpDown); // Y Axis - Up and Down
    js2_L_LeftRight = js2.getRawAxis(Constants.axisD_lLeftRight); // X Axis - Left and Right
    js2_R_UpDown = -js2.getRawAxis(Constants.axisD_rUpDown); // Z Rotate - Up and Down
    js2_R_LeftRight = js2.getRawAxis(Constants.axisD_rLeftRight); // Z Axis - Left and Right
    // endregion get raw values

    double dL_UD_Speed = Double.parseDouble(c_SelectedDriveYSpeed);
    double dR_UD_Speed = Double.parseDouble(c_SelectedDriveYSpeed);
    double dR_LR_Speed = Double.parseDouble(c_SelectedDriveZASpeed);

    // double dL_UD_Speed = Double.parseDouble("0.8");
    // double dR_UD_Speed = Double.parseDouble("0.8");
    // double dR_LR_Speed = Double.parseDouble("0.6");

    // double dL_UD_Speed = 0.8;
    // double dR_UD_Speed = 0.8;
    // double dR_LR_Speed = 0.6;

    // region Driver Controller
    if ((js1_L_UpDown > Constants.js1_L_UpDown_Deadband_Low)) {
      js1_L_UpDown = ((js1_L_UpDown - Constants.js1_L_UpDown_Deadband_Low) *
          dL_UD_Speed);
    } else if ((js1_L_UpDown < -Constants.js1_L_UpDown_Deadband_Low)) {
      js1_L_UpDown = ((js1_L_UpDown + Constants.js1_L_UpDown_Deadband_Low) *
          dL_UD_Speed);
    } else {
      js1_L_UpDown = 0; // If between boundaries. Do nothing.
    }

    if ((js1_R_UpDown > Constants.js1_R_UpDown_Deadband_Low)) {
      js1_R_UpDown = ((js1_R_UpDown - Constants.js1_R_UpDown_Deadband_Low) *
          dR_UD_Speed);
    } else if ((js1_R_UpDown < -Constants.js1_R_UpDown_Deadband_Low)) {
      js1_R_UpDown = ((js1_R_UpDown + Constants.js1_R_UpDown_Deadband_Low) *
          dR_UD_Speed);
    } else {
      js1_R_UpDown = 0; // If between boundaries. Do nothing.
    }

    if ((js1_R_LeftRight > Constants.js1_R_LeftRight_Deadband_Low)) {
      js1_R_LeftRight = ((js1_R_LeftRight - Constants.js1_R_LeftRight_Deadband_Low)
          * dR_LR_Speed);
    } else if ((js1_R_LeftRight < -Constants.js1_R_LeftRight_Deadband_Low)) {
      js1_R_LeftRight = ((js1_R_LeftRight + Constants.js1_R_LeftRight_Deadband_Low)
          * dR_LR_Speed);
    } else {
      js1_R_LeftRight = 0; // If between boundaries. Do nothing.
    }
    // endregion Driver Controller

    // region Operator Controller
    if ((js2_L_UpDown > Constants.js2_L_UpDown_Deadband_Low)) {
      js2_L_UpDown = ((js2_L_UpDown - Constants.js2_L_UpDown_Deadband_Low) *
          Constants.js2_L_UpDown_Deadband_High);
    } else if ((js2_L_UpDown < -Constants.js2_L_UpDown_Deadband_Low)) {
      js2_L_UpDown = ((js2_L_UpDown + Constants.js2_L_UpDown_Deadband_Low) *
          Constants.js2_L_UpDown_Deadband_High);
    } else {
      js2_L_UpDown = 0; // If between boundaries. Do nothing.
    }
    if ((js2_R_UpDown > Constants.js2_R_UpDown_Deadband_Low)) {
      js2_R_UpDown = ((js2_R_UpDown - Constants.js2_R_UpDown_Deadband_Low) *
          Constants.js2_R_UpDown_Deadband_High);
    } else if ((js2_R_UpDown < -Constants.js2_R_UpDown_Deadband_Low)) {
      js2_R_UpDown = ((js2_R_UpDown + Constants.js2_R_UpDown_Deadband_Low) *
          Constants.js2_R_UpDown_Deadband_High);
    } else {
      js2_R_UpDown = 0; // If between boundaries. Do nothing.
    }
    // endregion Operator Controller
    // endregion Deadbands

    // region diagnostic

    // region Specific Motor Diagnotics
    // double busVoltage = m_motor.getBusVoltage();
    // double current = m_motor.getOutputCurrent();
    // double appliedOut = m_motor.getAppliedOutput();
    // double temperature = m_motor.getMotorTemperature();

    // SmartDashboard.putNumber("Bus Voltage", busVoltage);
    // SmartDashboard.putNumber("Current", current);
    // SmartDashboard.putNumber("Applied Output", appliedOut);
    // SmartDashboard.putNumber("Motor Temperature", temperature);
    // endregion Specific Motor Diagnotics

    // SmartDashboard.putNumber("Temperature", m_pdp.getTemperature());
    // SmartDashboard.putNumber("PDP 0", m_pdp.getCurrent(0));
    // SmartDashboard.putNumber("PDP 1", m_pdp.getCurrent(1));
    // SmartDashboard.putNumber("PDP 2", m_pdp.getCurrent(2));
    // SmartDashboard.putNumber("PDP 3", m_pdp.getCurrent(3));
    // SmartDashboard.putNumber("PDP 4", m_pdp.getCurrent(4));
    // SmartDashboard.putNumber("PDP 5", m_pdp.getCurrent(5));
    // SmartDashboard.putNumber("PDP 6", m_pdp.getCurrent(6));
    // SmartDashboard.putNumber("PDP 7", m_pdp.getCurrent(7));
    // SmartDashboard.putNumber("PDP 8", m_pdp.getCurrent(8));
    // SmartDashboard.putNumber("PDP 9", m_pdp.getCurrent(9));
    // SmartDashboard.putNumber("PDP 10", m_pdp.getCurrent(10));
    // SmartDashboard.putNumber("PDP 11", m_pdp.getCurrent(11));
    // SmartDashboard.putNumber("PDP 12", m_pdp.getCurrent(12));
    // SmartDashboard.putNumber("PDP 13", m_pdp.getCurrent(13));
    // SmartDashboard.putNumber("PDP 14", m_pdp.getCurrent(14));
    // SmartDashboard.putNumber("PDP 15", m_pdp.getCurrent(15));
    // SmartDashboard.putNumber("js1LDB", js1_L_UpDown);
    // SmartDashboard.putNumber("js1RDB", js1_R_LeftRight);
    // endregion diagnostic
  }

  @Override
  public void autonomousInit() {
    c_SelectedAuton = c_Auton.getSelected();
    c_SelectedDelay = c_Auton_Delay.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + c_SelectedAuton);
    rTimer.reset();
    Constants.tAuton = 0; // Timer
    rTimer.start();

    m_left01MotorEncoder.setPosition(0);
    m_right01MotorEncoder.setPosition(0);
    m_left02MotorEncoder.setPosition(0);
    m_right02MotorEncoder.setPosition(0);

    m_left01Motor.setIdleMode(IdleMode.kBrake);
    m_left02Motor.setIdleMode(IdleMode.kBrake);
    m_right01Motor.setIdleMode(IdleMode.kBrake);
    m_right02Motor.setIdleMode(IdleMode.kBrake);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    Shuffleboard.startRecording();

    // 1 encoder count = 1.5 inches
    // int ecFR = 1/11;
    double inFR = 1.713;
    // int ecPerInch = 34 / (int) inFR;

    switch (c_SelectedAuton) {
      case "position_shooter":

        // Drive backwards 60 inches (Number of inches you want drive / 2)
        if (m_left01MotorEncoder.getPosition() >= -(30 / (int) inFR)) {
          m_DriveRobotRobot.tankDrive(-0.6, -0.6);
        } else {
          m_DriveRobotRobot.stopMotor();
        }

        // if (m_right01MotorEncoder.getPosition() >= -21) {
        // m_DriveRobotRobot.tankDrive(-0.2, -0.4);
        // } else {
        // m_DriveRobotRobot.stopMotor();
        // }

        // region Timed Auton Code
        // // Drive Forward
        // do {
        // s_IntakePush.set(1.0);
        // m_DriveRobotRobot.tankDrive(0.6, 0.6);
        // } while (rTimer.get() < 10);
        // m_DriveRobotRobot.tankDrive(0.0, 0.0);
        // rTimer.stop();
        // rTimer.reset();

        // // Right Turn
        // rTimer.start();
        // do {
        // m_DriveRobotRobot.tankDrive(0.6, -0.6);
        // } while (rTimer.get() < 10);
        // m_DriveRobotRobot.tankDrive(0.0, 0.0);
        // rTimer.stop();
        // rTimer.reset();

        // // Drive Reverse
        // rTimer.start();
        // do {
        // m_Intake.set(-1.0);
        // m_DriveRobotRobot.tankDrive(-0.6, -0.6);
        // } while (rTimer.get() < 10);
        // m_DriveRobotRobot.tankDrive(0.0, 0.0);
        // rTimer.stop();
        // rTimer.reset();

        // // Left Turn
        // rTimer.start();
        // do {
        // m_DriveRobotRobot.tankDrive(-0.6, 0.6);
        // } while (rTimer.get() < 10);
        // m_DriveRobotRobot.tankDrive(0.0, 0.0);
        // rTimer.stop();
        // rTimer.reset();
        // region Timed Auton Code

        break;

      case "Default": // DEFAULT
      default:
        System.out.println("Time: " + Constants.tAuton);
        s_IntakePush.set(1.0);
        m_DriveRobotRobot.arcadeDrive(0.0, 0.0);
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    super.teleopInit();
    // region Climber
    m_Climber01.configFactoryDefault();
    m_Climber02.configFactoryDefault();

    /* flip values so robot moves forward when stick-forward/LEDs-green */
    m_Climber01.setInverted(false); // <<<<<< Adjust this
    m_Climber02.setInverted(false); // <<<<<< Adjust this
    // endregion Climber

    m_left01MotorEncoder.setPosition(0);
    m_right01MotorEncoder.setPosition(0);
    m_left02MotorEncoder.setPosition(0);
    m_right02MotorEncoder.setPosition(0);

    m_left01Motor.setIdleMode(IdleMode.kCoast);
    m_left02Motor.setIdleMode(IdleMode.kCoast);
    m_right01Motor.setIdleMode(IdleMode.kCoast);
    m_right02Motor.setIdleMode(IdleMode.kCoast);
  }

  private double getAverageEncoderPosition() {
    return (m_left01MotorEncoder.getPosition() + m_right01MotorEncoder.getPosition()) / 2;
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    Shuffleboard.startRecording();

    // region Climber
    double dClimberSpeed = Double.parseDouble(c_SelectedClimberSpeed);
    if (js2.getRawButtonPressed(Constants.btnD_Start)) {
      s_RachetPull.set(1.0);
    }
    if (js2.getRawButtonReleased(Constants.btnD_Start)) {
      s_RachetPull.set(0.0);
    }

    switch (js2.getPOV()) {
      case 0:
        // m_climb.arcadeDrive(0.6, 0);
        m_climb.arcadeDrive(dClimberSpeed, 0);
        break;
      case 180:
        m_climb.arcadeDrive(-dClimberSpeed, 0);
        break;
      default:
        m_climb.stopMotor();
        break;
    }
    // endregion Climber

    // region Ball Shooter
    double dIntakeShooterSpeed = Double.parseDouble(c_SelectedIntakeShooterSpeed);
    double dShooterSpeed = Double.parseDouble(c_SelectedShooterSpeed);
    if (js2.getRawButtonPressed(Constants.btnD_RB)) {
      m_intakeShooter.set(dIntakeShooterSpeed);
    }
    if (js2.getRawButtonReleased(Constants.btnD_RB)) {
      m_intakeShooter.set(0.0);
    }

    if (js2.getRawButtonPressed(Constants.btnD_RT)) {
      m_intakeShooter.set(-dIntakeShooterSpeed);
    }
    if (js2.getRawButtonReleased(Constants.btnD_RT)) {
      m_intakeShooter.set(0.0);
    }

    // region Intake Shooter - Fixed Speed
    if (js2.getRawButtonPressed(Constants.btnD_Y)) {
      m_intakeShooter.set(-1);
    }
    if (js2.getRawButtonReleased(Constants.btnD_Y)) {
      m_intakeShooter.set(0.0);
    }

    // endregion Intake Shooter - Fixed Speed

    if (js2.getRawButtonPressed(Constants.btnD_X)) {
      m_shooter.set(dShooterSpeed);
    }
    if (js2.getRawButtonReleased(Constants.btnD_X)) {
      m_shooter.set(0.0);
    }

    if (js2.getRawButtonPressed(Constants.btnD_A)) {
      m_shooter.set(-dShooterSpeed);
    }
    if (js2.getRawButtonReleased(Constants.btnD_A)) {
      m_shooter.set(0.0);
    }

    // region Shooter - Fixed Speed
    if (js2.getRawButtonPressed(Constants.btnD_B)) {
      m_intakeShooter.set(85);
      m_shooter.set(-80);
    }
    if (js2.getRawButtonReleased(Constants.btnD_B)) {
      m_shooter.set(0.0);
      m_intakeShooter.set(0.0);
    }
    // endregion Shooter - Fixed Speed

    // endregion Ball Shooter

    // region Encoder Drive
    if (js1.getRawButtonPressed(Constants.btnD_X)) {
      m_left01MotorEncoder.setPosition(0);
      m_left02MotorEncoder.setPosition(0);
      m_right01MotorEncoder.setPosition(0);
      m_right02MotorEncoder.setPosition(0);
    }

    // if (js1.getRawButtonPressed(Constants.btnD_A)) {
    // double inFR = 1.713;
    // // Drive backwards 34 inches (Number of inches you want drive / 2)
    // if (getAverageEncoderPosition() >= -(17 / (int) inFR)) {
    // // if (m_right01MotorEncoder.getPosition() >= -(17 / (int) inFR)) {
    // m_DriveRobotRobot.tankDrive(-0.6, -0.6);
    // // m_DriveRobotRobot.arcadeDrive(-0.6, 0);
    // } else {
    // m_DriveRobotRobot.stopMotor();
    // }
    // }
    // endregion Encoder Drive

    // Drive with arcade drive.

    // region Drive Straight
    // error = m_left01MotorEncoder.getPosition() - m_right01MotorEncoder.getPosition();
    // turn_power = js1_R_LeftRight * error;
    // m_DriveRobotRobot.arcadeDrive(-js1_L_UpDown, turn_power);
    // endregion Drive Straight

    // That means that the Y axis drives forward
    // if (c_SelectedDriveMode == "Tank") {
    // m_DriveRobotRobot.tankDrive(-js1_R_UpDown, js1_L_UpDown); // regular teleop
    // driving
    // } else {
    m_DriveRobotRobot.arcadeDrive(-js1_L_UpDown, js1_R_LeftRight); // regular
    // teleop driving
    // }

    // region Intake
    if (js2.getRawButtonPressed(Constants.btnD_LB)) {
      m_Intake.set(1.0);
    }
    if (js2.getRawButtonReleased(Constants.btnD_LB)) {
      m_Intake.set(0.0);
    }

    if (js2.getRawButtonPressed(Constants.btnD_LT)) {
      m_Intake.set(-1.0);
    }
    if (js2.getRawButtonReleased(Constants.btnD_LT)) {
      m_Intake.set(0.0);
    }

    if (js2.getRawButtonPressed(Constants.btnD_Back)) {
      s_IntakePush.set(1.0);
    }
    if (js2.getRawButtonReleased(Constants.btnD_Back)) {
      s_IntakePush.set(0.5);
    }
    // endregion Intake

  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    super.disabledInit();
    Shuffleboard.stopRecording();
    m_left01Motor.setIdleMode(IdleMode.kCoast);
    m_left02Motor.setIdleMode(IdleMode.kCoast);
    m_right01Motor.setIdleMode(IdleMode.kCoast);
    m_right02Motor.setIdleMode(IdleMode.kCoast);
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    super.disabledPeriodic();
    m_left01Motor.setIdleMode(IdleMode.kCoast);
    m_left02Motor.setIdleMode(IdleMode.kCoast);
    m_right01Motor.setIdleMode(IdleMode.kCoast);
    m_right02Motor.setIdleMode(IdleMode.kCoast);
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    super.testInit();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    // region Driver
    if (js1.getRawButtonPressed(Constants.btnD_X)) {
      m_left01Motor.set(0.4); // Forward
    }
    if (js1.getRawButtonReleased(Constants.btnD_X)) {
      m_left01Motor.set(0.0);
    }
    if (js1.getRawButtonPressed(Constants.btnD_Y)) {
      m_left02Motor.set(0.4);
    }
    if (js1.getRawButtonReleased(Constants.btnD_Y)) {
      m_left02Motor.set(0.0);
    }
    if (js1.getRawButtonPressed(Constants.btnD_A)) {
      m_right01Motor.set(0.4);
    }
    if (js1.getRawButtonReleased(Constants.btnD_A)) {
      m_right01Motor.set(0.0);
    }
    if (js1.getRawButtonPressed(Constants.btnD_B)) {
      m_right02Motor.set(0.4);
    }
    if (js1.getRawButtonReleased(Constants.btnD_B)) {
      m_right02Motor.set(0.0);
    }

    // endregion Driver
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }
}
