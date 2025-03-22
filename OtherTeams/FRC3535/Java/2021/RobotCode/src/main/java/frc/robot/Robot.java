/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

//region Imports
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWMTalonSRX;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;

import frc.robot.subsystems.AirCompressor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LimelightVision;
import frc.robot.subsystems.Shooter;

import java.io.*;
import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

//endregion Imports

public class Robot extends TimedRobot {

  public static Timer rTimer = new Timer();

  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private final DoubleSolenoid p_Intake = new DoubleSolenoid(Constants.pcmPort, Constants.pIntakeIn, Constants.pIntakeOut);
  private final DoubleSolenoid p_Rachet = new DoubleSolenoid(Constants.pcmPort, Constants.pRachetIn, Constants.pRachetOut);
  // private final Solenoid p_Rachet = new Solenoid(Constants.pcmPort,
  // Constants.pRachet);

  // region Talon Motors
  PWMTalonSRX m_UpTake = new PWMTalonSRX(Constants.m_Uptake);
  PWMTalonSRX m_InTake = new PWMTalonSRX(Constants.m_Intake);
  PWMTalonSRX m_Climber01 = new PWMTalonSRX(Constants.m_Climber01);
  // PWMTalonSRX m_Climber02 = new PWMTalonSRX(Constants.m_Climber02);
  // endregion Talon Motors

  // region Joysticks
  private final Joystick js1 = new Joystick(Constants.js1);
  private final Joystick js2 = new Joystick(Constants.js2);
  // endregion Joysticks

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

  // region LimeLight Basic
  boolean bLimeLightConnected = true;
  double targetDistance = 1.5; // Target Area (NEED TO DO Target Area to Inches)
  double driveScaleDown = 1;
  double turnScaleDown = 0.05;
  double tx = 0.0;
  double ta = 0.0;
  double m_speed = 0.0;
  double output = 0.0;
  boolean turnInPlace = false;

  public void LimelightBasic() {
    tx = LimelightVision.getTX();
    ta = LimelightVision.getTA();
    if (Math.abs(tx) > 0) {
      Drivetrain.m_drive.arcadeDrive(0.0, (tx * turnScaleDown));
    } else {
      Drivetrain.m_drive.arcadeDrive(0.0, 0.0);
    }
  }

  public void LimelightDistance() {
    tx = LimelightVision.getTX();
    ta = LimelightVision.getTA();
    if (Math.abs(tx) > 0) {
      Drivetrain.m_drive.arcadeDrive((getDriveSpeed() * driveScaleDown), (tx * turnScaleDown));
    } else {
      Drivetrain.m_drive.arcadeDrive(0.0, 0.0);
    }
  }

  public double getDriveSpeed() {
    double ta = LimelightVision.getTA();
    m_speed = 0.6; // Max speed - Auto scales down the speed of the motor

    if (ta > 0) {
      m_speed = (m_speed * ((targetDistance - ta) / targetDistance)); // Distance in Inches

      if (m_speed <= -0.5) {
        m_speed = -0.5;
      }
    } else {
      m_speed = 0;
      output = 0;
    }

    if (turnInPlace) {
      m_speed = 0;
    }

    return m_speed;
  }

  // endregion LimeLight Basic

  private String c_SelectedAuton;
  private String c_SelectedDelay;
  private String c_SelectedDriveMode;
  private String c_SelectedControllerDriver;
  private String c_SelectedControllerOperator;
  private String c_SelectedDriveYSpeed;
  private String c_SelectedDriveZASpeed;
  private final SendableChooser<String> c_Auton = new SendableChooser<>();
  private final SendableChooser<String> c_Auton_Delay = new SendableChooser<>();
  private final SendableChooser<String> c_ControllerDriver = new SendableChooser<>();
  private final SendableChooser<String> c_ControllerOperator = new SendableChooser<>();
  private final SendableChooser<String> c_DriveMode = new SendableChooser<>();
  private final SendableChooser<String> c_DriveYSpeed = new SendableChooser<>();
  private final SendableChooser<String> c_DriveZASpeed = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    AirCompressor.clearFaults();

    Drivetrain.motorSafety(false);
    Drivetrain.resetEncoders();

    // region Auton Selection
    // c_Auton.setDefaultOption("Default Auton", "Default");
    c_Auton.setDefaultOption("Do Nothing", "Default");
    c_Auton.addOption("Shoot Then Forward", "Shoot01");
    c_Auton.addOption("Shoot Then Forward 2", "Shoot03");
    c_Auton.addOption("Shoot Then Backward", "Shoot02");
    c_Auton.addOption("Feeder Bot", "feederbot");
    c_Auton.addOption("Drive Forward (25 EC)", "Forward25");
    c_Auton.addOption("Drive Forward (50 EC)", "Forward50");
    c_Auton.addOption("Drive Backward (25 EC)", "Backward25");
    c_Auton.addOption("Drive Backward (50 EC)", "Backward50");
    c_Auton.addOption("Left", "Left");
    c_Auton.addOption("Center", "Center");
    c_Auton.addOption("Right", "Right");
    SmartDashboard.putData("Auton choices", c_Auton);
    // endregion Auton Selection

    // region Auton Delay Selection
    c_Auton_Delay.setDefaultOption("None", "None");
    c_Auton_Delay.addOption("1 second", "1");
    c_Auton_Delay.addOption("2 second", "2");
    c_Auton_Delay.addOption("3 second", "3");
    c_Auton_Delay.addOption("4 second", "4");
    c_Auton_Delay.addOption("5 second", "5");
    c_Auton_Delay.addOption("6 second", "6");
    SmartDashboard.putData("Auto Delay", c_Auton_Delay);
    // endregion Auton Delay Selection

    // region Drive Mode Selection
    c_DriveMode.setDefaultOption("Arcade", "Arcade");
    c_DriveMode.addOption("Tank", "Tank");
    SmartDashboard.putData("Drive Mode", c_DriveMode);
    // endregion Drive Mode Selection

    // region Drive Controller Selection
    c_ControllerDriver.setDefaultOption("LogitechD", "LogitechD");
    c_ControllerDriver.addOption("LogitechX", "LogitechX");
    c_ControllerDriver.addOption("XBox", "XBox");
    SmartDashboard.putData("Driver Controller", c_ControllerDriver);
    // endregion Drive Controller Selection

    // region Operator Controller Selection
    c_ControllerOperator.setDefaultOption("LogitechD", "LogitechD");
    c_ControllerOperator.addOption("LogitechX", "LogitechX");
    c_ControllerOperator.addOption("XBox", "XBox");
    SmartDashboard.putData("Operator Controller", c_ControllerOperator);
    // endregion Operator Controller Selection

    // Region Left Joystick Speed Control
    c_DriveYSpeed.addOption(".50", ".50");
    c_DriveYSpeed.addOption(".55", ".55");
    c_DriveYSpeed.addOption(".60", ".60");
    c_DriveYSpeed.addOption(".65", ".65");
    c_DriveYSpeed.addOption(".70", ".70");
    c_DriveYSpeed.addOption(".75", ".75");
    c_DriveYSpeed.setDefaultOption(".80", ".80");
    c_DriveYSpeed.addOption(".85", ".85");
    c_DriveYSpeed.addOption(".90", ".90");
    SmartDashboard.putData("Drive L Speed", c_DriveYSpeed);
    // Endregion Left Joystick Speed Control

    // Region Right (Arcade Drive) Joystick Speed Control
    c_DriveZASpeed.addOption(".50", ".50");
    c_DriveZASpeed.addOption(".55", ".55");
    c_DriveZASpeed.setDefaultOption(".60", ".60");
    c_DriveZASpeed.addOption(".65", ".65");
    c_DriveZASpeed.addOption(".70", ".70");
    c_DriveZASpeed.addOption(".75", ".75");
    c_DriveZASpeed.addOption(".80", ".80");
    c_DriveZASpeed.addOption(".85", ".85");
    c_DriveZASpeed.addOption(".90", ".90");
    SmartDashboard.putData("Drive R (Arcade) Speed", c_DriveZASpeed);
    // Endregion Right (Arcade Drive) Joystick Speed Control

    // System.out.println(LimelightVision.distanceToTargetInInches());

    // RobotGyro.m_gyro.reset();
    // RobotGyro.m_gyro.calibrate();

    CameraServer camera01 = CameraServer.getInstance();
    camera01.startAutomaticCapture(0);

    String trajectoryJSON = "/home/lvuser/deploy/output/BarrelRacing01.wpilib.json";
    Trajectory trajectory = new Trajectory();
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      Trajectory jsonTrajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      System.out.println("Pathweaver Loaded");

      // System.out.println(jsonTrajectory);

    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
      System.out.println("Pathweaver Did Not Load" + trajectoryJSON + ex.getStackTrace());
    }

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
    SmartDashboard.putBoolean("enabled", DriverStation.getInstance().isEnabled());

    // double avgCount = (Drivetrain.m_frontLeft.getPosition() +
    // Drivetrain.m_frontRight.getPosition()) / 2.0;
    // double meters = ((avgCount / 2048) / 16) * (0.075 * 2 * Math.PI);
    // SmartDashboard.putNumber("Meters", meters);

    c_SelectedDriveYSpeed = c_DriveYSpeed.getSelected();
    c_SelectedDriveZASpeed = c_DriveZASpeed.getSelected();
    c_SelectedDriveMode = c_DriveMode.getSelected();
    c_SelectedControllerDriver = c_ControllerDriver.getSelected();
    c_SelectedControllerOperator = c_ControllerOperator.getSelected();

    double speed = Shooter.m_Shooter.getSpeed();

    AirCompressor.enable();
    SmartDashboard.putBoolean("Compressor", AirCompressor.status());

    m_Climber01.setInverted(true);

    // SmartDashboard.putNumber("Button_Count", js1.getButtonCount());
    // SmartDashboard.putNumber("JS1 POV", js1.getPOV());
    // SmartDashboard.putNumber("JS2 POV", js2.getPOV());

    // region Limelight Vision
    try {
      // LimelightVision.setLEDMode(Constants.LimelightConstants.ll_on);
      // SmartDashboard.putNumber("limelight_X", LimelightVision.getTX());
      // SmartDashboard.putNumber("limelight_Y", LimelightVision.getTY());
      // SmartDashboard.putNumber("limelight_Area", LimelightVision.getTA());
      // SmartDashboard.putNumber("limelight_Latency", LimelightVision.getTL());
      // SmartDashboard.putNumber("limelight_Valid_Target", LimelightVision.getTV());
      // SmartDashboard.putNumber("limelight_Skew", LimelightVision.getTS());
      // SmartDashboard.putNumber("limelight_Steering_Adjust",
      // LimelightVision.getSteeringAdjust());
    } catch (Exception name) {
      // System.out.println("Exception: " + name);
      // System.out.println("Error 272 - Unable to connect to the LimeLight");
      bLimeLightConnected = false;
    }
    // endregion Limelight Vision

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
      js1_L_UpDown = ((js1_L_UpDown - Constants.js1_L_UpDown_Deadband_Low) * dL_UD_Speed);
    } else if ((js1_L_UpDown < -Constants.js1_L_UpDown_Deadband_Low)) {
      js1_L_UpDown = ((js1_L_UpDown + Constants.js1_L_UpDown_Deadband_Low) * dL_UD_Speed);
    } else {
      js1_L_UpDown = 0; // If between boundaries. Do nothing.
    }

    if ((js1_R_UpDown > Constants.js1_R_UpDown_Deadband_Low)) {
      js1_R_UpDown = ((js1_R_UpDown - Constants.js1_R_UpDown_Deadband_Low) * dR_UD_Speed);
    } else if ((js1_R_UpDown < -Constants.js1_R_UpDown_Deadband_Low)) {
      js1_R_UpDown = ((js1_R_UpDown + Constants.js1_R_UpDown_Deadband_Low) * dR_UD_Speed);
    } else {
      js1_R_UpDown = 0; // If between boundaries. Do nothing.
    }

    if ((js1_R_LeftRight > Constants.js1_R_LeftRight_Deadband_Low)) {
      js1_R_LeftRight = ((js1_R_LeftRight - Constants.js1_R_LeftRight_Deadband_Low) * dR_LR_Speed);
    } else if ((js1_R_LeftRight < -Constants.js1_R_LeftRight_Deadband_Low)) {
      js1_R_LeftRight = ((js1_R_LeftRight + Constants.js1_R_LeftRight_Deadband_Low) * dR_LR_Speed);
    } else {
      js1_R_LeftRight = 0; // If between boundaries. Do nothing.
    }
    // endregion Driver Controller

    // region Operator Controller
    if ((js2_L_UpDown > Constants.js2_L_UpDown_Deadband_Low)) {
      js2_L_UpDown = ((js2_L_UpDown - Constants.js2_L_UpDown_Deadband_Low) * Constants.js2_L_UpDown_Deadband_High);
    } else if ((js2_L_UpDown < -Constants.js2_L_UpDown_Deadband_Low)) {
      js2_L_UpDown = ((js2_L_UpDown + Constants.js2_L_UpDown_Deadband_Low) * Constants.js2_L_UpDown_Deadband_High);
    } else {
      js2_L_UpDown = 0; // If between boundaries. Do nothing.
    }
    if ((js2_R_UpDown > Constants.js2_R_UpDown_Deadband_Low)) {
      js2_R_UpDown = ((js2_R_UpDown - Constants.js2_R_UpDown_Deadband_Low) * Constants.js2_R_UpDown_Deadband_High);
    } else if ((js2_R_UpDown < -Constants.js2_R_UpDown_Deadband_Low)) {
      js2_R_UpDown = ((js2_R_UpDown + Constants.js2_R_UpDown_Deadband_Low) * Constants.js2_R_UpDown_Deadband_High);
    } else {
      js2_R_UpDown = 0; // If between boundaries. Do nothing.
    }
    // endregion Operator Controller
    // endregion Deadbands

    // SmartDashboard.putNumber("js1LDB", js1_L_UpDown);
    // SmartDashboard.putNumber("js1RDB", js1_R_LeftRight);
    // SmartDashboard.putNumber("Shooter_Speed", speed);
    // SmartDashboard.putNumber("Shooter_Speed_Graph", speed);

    // SmartDashboard.putBoolean("Auton", isAutonomous());
    // SmartDashboard.putBoolean("Teleop", isOperatorControl());
    // SmartDashboard.putBoolean("Enabled", isEnabled());

    // SmartDashboard.putNumber("js1L", js1.getY());
    // SmartDashboard.putNumber("js1R", js1.getZ());

    // SmartDashboard.putNumber("Rotate Function Number",
    // RobotGyro.rotateToAngle(90));
    // SmartDashboard.putNumber("Gyro Angle", RobotGyro.m_gyro.getAngle());

    SmartDashboard.putNumber("Front Left", Drivetrain.m_frontLeft.getPosition());
    SmartDashboard.putNumber("Front Right", Drivetrain.m_frontRight.getPosition());
    // System.out.println("Brake/Coast " + m_frontLeft.getBrakeCoastMode());

    // SmartDashboard.putNumber("Gyro Angle", RobotGyro.m_gyro.getAngle());

    SmartDashboard.putNumber("Timer", rTimer.get());
    // System.out.println(LimelightVision.distanceToTargetInInches());
    // SmartDashboard.putNumber("Limelight Vision - Distance To Target In Meters",
    // LimelightVision.getDist());
    // SmartDashboard.putString("Shooter_Temperature", m_Shooter.getTemperature() +
    // " C");
    // SmartDashboard.putNumber("Shooter_Output_Current ",
    // m_Shooter.getOutputCurrent());
    // SmartDashboard.putNumber("Shooter_Output_Voltage ",
    // m_Shooter.getOutputVoltage());

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
    AirCompressor.clearFaults();
    c_SelectedAuton = c_Auton.getSelected();
    c_SelectedDelay = c_Auton_Delay.getSelected();

    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + c_SelectedAuton);

    Constants.tAuton = 0; // Timer

    rTimer.start();

    Shooter.m_Shooter.setMaxAcceleration(Constants.pidMaxAcceleration); // Set max acceleration to 20,000 RPM/s
    Shooter.m_Shooter.setMaxJerk(Constants.pidMaxJerk); // Set max jerk to 31,250 RPM/s^2
    Shooter.m_Shooter.setMaxSpeed(Constants.pidMaxSpeed);

    Shooter.m_Shooter.setPID(Constants.pidKp, Constants.pidKi, Constants.pidKd, Constants.pidKf, Constants.pidKb); // Configure
                                                                                                                   // PID
                                                                                                                   // gains
    // RobotGyro.m_gyro.reset();
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Shuffleboard.startRecording();

    double avgCount = (Drivetrain.m_frontLeft.getPosition() + Drivetrain.m_frontRight.getPosition()) / 2.0;
    double meters = ((avgCount / 2048) / 16) * (0.075 * 2 * Math.PI);

    try {
      LimelightVision.setLEDMode(Constants.LimelightConstants.ll_on);
    } catch (Exception name) {
      // System.out.println("Exception: " + name);
      // System.out.println("Error 396 - Unable to connect to the LimeLight");
    }

    p_Intake.set(DoubleSolenoid.Value.kReverse); // Down

    Drivetrain.venomBrakeMode(1);
    Constants.tAuton += 1; // Timer

    Drivetrain.enableMotors();

    switch (c_SelectedAuton) {
      case "Forward25":
        // Drive Forward 25 Encoder Count
        if (Drivetrain.m_frontLeft.getPosition() < 30) {
          Drivetrain.m_drive.tankDrive(0.5, 0.5);
          // double turningValue = (RobotGyro.kAngleSetpoint -
          // RobotGyro.m_gyro.getAngle()) * RobotGyro.kP;
          // // Invert the direction of the turn if we are going backwards
          // turningValue = Math.copySign(turningValue,0.5);
          // Drivetrain.m_drive.arcadeDrive(0.5, turningValue);
          System.out.println("Front Left: " + Drivetrain.m_frontLeft.getPosition());
          System.out.println("Front Right: " + Drivetrain.m_frontRight.getPosition());
        }
        break;
      case "Forward50":
        // Drive Forward 50 Encoder Count
        if (Drivetrain.m_frontLeft.getPosition() < 55) {
          Drivetrain.m_drive.tankDrive(0.5, 0.5);          
          System.out.println("Front Left: " + Drivetrain.m_frontLeft.getPosition());
          System.out.println("Front Right: " + Drivetrain.m_frontRight.getPosition());
        }
        break;
      case "Backward25":
        // Drive Backward 25 Encoder Count
        if (Drivetrain.m_frontLeft.getPosition() > -30) {
          Drivetrain.m_drive.tankDrive(-0.5, -0.5);
          // double turningValue = (RobotGyro.kAngleSetpoint -
          // RobotGyro.m_gyro.getAngle()) * RobotGyro.kP;
          // // Invert the direction of the turn if we are going backwards
          // turningValue = Math.copySign(turningValue,-0.5);
          // Drivetrain.m_drive.arcadeDrive(-0.5, turningValue);
          System.out.println("Front Left: " + Drivetrain.m_frontLeft.getPosition());
          System.out.println("Front Right: " + Drivetrain.m_frontRight.getPosition());
        }
        break;
      case "Backward50":
        // Drive Backward 50 Encoder Count
        if (Drivetrain.m_frontLeft.getPosition() > -55) {
          Drivetrain.m_drive.tankDrive(-0.5, -0.5);
          System.out.println("Front Left: " + Drivetrain.m_frontLeft.getPosition());
          System.out.println("Front Right: " + Drivetrain.m_frontRight.getPosition());
        }
        break;
      case "Shoot01":
        // Spin up the shooter motor, move forward 5 encoder counts, feed the balls to
        // the shooter, stop shooter and feed motors, Drive forward 10 encoder count
        if (rTimer.get() < 3) { // less than 250
          Shooter.m_Shooter.set(Constants.m_Shooter_Speed);
        }

        if (rTimer.get() > 3 && rTimer.get() < 5) { // 250 - 650
          // Drive Forward 5 Encoder Count
          if (Drivetrain.m_frontLeft.getPosition() < 5) {
            Drivetrain.m_drive.arcadeDrive(0.5, 0.0);
            System.out.println("Front Left: " + Drivetrain.m_frontLeft.getPosition());
            System.out.println("Front Right: " + Drivetrain.m_frontRight.getPosition());
          }
        }

        if (rTimer.get() > 5 && rTimer.get() < 13) { // 250 - 650
          // Shoot out balls
          m_UpTake.set(-0.5); // Up
        }

        if (rTimer.get() > 13 && rTimer.get() < 15) { // 650 - 750
          m_UpTake.set(0.0);
          Shooter.m_Shooter.set(0.0);

          // Drive Forward 10 Encoder Count
          if (Drivetrain.m_frontLeft.getPosition() < 15) {
            Drivetrain.m_drive.arcadeDrive(0.65, 0.0);
            System.out.println("Front Left: " + Drivetrain.m_frontLeft.getPosition());
            System.out.println("Front Right: " + Drivetrain.m_frontRight.getPosition());
          }
        }
        break;
      case "Shoot02":
        // Spin up the shooter motor, move forward 5 encoder counts, feed the balls to
        // the shooter, stop shooter and feed motors, Drive backward 15 encoder count

        if (rTimer.get() < 5) { // less than 250
          // if (rTimer.get() < 3) { // less than 250

          // m_Shooter.set(Constants.m_Shooter_Speed);
          Shooter.m_Shooter.enable();
          // Shooter.m_Shooter.setCommand(ControlMode.SpeedControl, 1500); // Spin the
          // motor at x RPM.
          Shooter.Shooter_SetRPM(1500);
        }

        if (rTimer.get() > 5 && rTimer.get() < 7) { // 250 - 650
          // if (rTimer.get() > 3 && rTimer.get() < 5) { // 250 - 650
          // Drive Forward 5 Encoder Count
          if (Drivetrain.m_frontLeft.getPosition() < 5) {
            Drivetrain.m_drive.arcadeDrive(0.5, 0.0);
            System.out.println("Front Left: " + Drivetrain.m_frontLeft.getPosition());
            System.out.println("Front Right: " + Drivetrain.m_frontRight.getPosition());
          }
        }

        if (rTimer.get() > 7 && rTimer.get() < 13) { // 250 - 650
          // if (rTimer.get() > 5 && rTimer.get() < 13) { // 250 - 650
          // Shoot out balls
          m_UpTake.set(Constants.uptake_up); // Up
        }

        if (rTimer.get() > 13 && rTimer.get() < 15) { // 650 - 750
          m_UpTake.set(0.0);
          Shooter.m_Shooter.set(0.0);

          // Drive Forward 15 Encoder Count
          if (Drivetrain.m_frontLeft.getPosition() < 20) {
            Drivetrain.m_drive.arcadeDrive(-0.65, 0.0);
            System.out.println("Front Left: " + Drivetrain.m_frontLeft.getPosition());
            System.out.println("Front Right: " + Drivetrain.m_frontRight.getPosition());
          }
        }
        break;
      case "Shoot03":
        // Spin up the shooter motor, move forward 5 encoder counts, feed the balls to
        // the shooter, stop shooter and feed motors, Drive backward 15 encoder count
        p_Intake.set(DoubleSolenoid.Value.kReverse);
        if (rTimer.get() < 5) { // less than 250
          // if (rTimer.get() < 3) { // less than 250

          // m_Shooter.set(Constants.m_Shooter_Speed);
          Shooter.m_Shooter.enable();
          // Shooter.m_Shooter.setCommand(ControlMode.SpeedControl, 1500); // Spin the
          // motor at x RPM.
          Shooter.Shooter_SetRPM(1500);
        }

        if (rTimer.get() > 5 && rTimer.get() < 13) { // 250 - 650
          // if (rTimer.get() > 5 && rTimer.get() < 13) { // 250 - 650
          // Shoot out balls
          m_UpTake.set(Constants.uptake_up); // Up
          m_InTake.set(Constants.intake_in); // In
        }

        if (rTimer.get() > 13 && rTimer.get() < 15) { // 650 - 750
          m_UpTake.set(0.0);
          Shooter.m_Shooter.set(0.0);
          m_InTake.set(0.0);

          // Drive Forward 5 Encoder Count
          if (Drivetrain.m_frontLeft.getPosition() < 5) {
            Drivetrain.m_drive.arcadeDrive(0.65, 0.0);
            System.out.println("Front Left: " + Drivetrain.m_frontLeft.getPosition());
            System.out.println("Front Right: " + Drivetrain.m_frontRight.getPosition());
          }
        }
        break;
      case "feederbot": {
        if (rTimer.get() < 3) { // less than 250
          // DO nothing

        }

        if (rTimer.get() > 3 && rTimer.get() < 13) { // 250 - 650
          // Spit out balls
          m_UpTake.set(0.5); // Down
          m_InTake.set(0.5); // Out
        }
        if (rTimer.get() > 13 && rTimer.get() < 15) { // 650 - 750
          m_UpTake.set(0.0); // Down
          m_InTake.set(0.0); // Out

          // Drive Forward 25 Encoder Count
          if (Drivetrain.m_frontLeft.getPosition() < 30) {
            Drivetrain.m_drive.arcadeDrive(0.65, 0.45);
            System.out.println("Front Left: " + Drivetrain.m_frontLeft.getPosition());
            System.out.println("Front Right: " + Drivetrain.m_frontRight.getPosition());
          }
        }
      }
      case "Left":
        // Left Auton
        System.out.println("Time: " + Constants.tAuton);

        break;
      case "Center":

        // Center Auton
        // Setpoint is implicitly 0, since we don't want the heading to change
        // double error = -RobotGyro.m_gyro.getRate();

        // Drives forward continuously at half speed, using the gyro to stabilize the
        // heading
        // if (Drivetrain.m_frontLeft.getPosition() < 30) {
        // Drivetrain.m_drive.tankDrive(.5 + RobotGyro.kP * error, .5 - RobotGyro.kP *
        // error);
        // // System.out.println("Front Left: " + Drivetrain.m_frontLeft.getPosition());
        // // System.out.println("Front Right: " +
        // Drivetrain.m_frontRight.getPosition());
        // // System.out.println(kP);
        // // System.out.println("Gyro Error: " + error);
        // }

        break;
      case "Right":

        // double reverseSpeed = -0.5;
        // double forwardSpeed = 0.5;
        // double targetAngle = 90;

        // //Positive angle is roatation clockwise
        // //currentAngle is the angle measured off the gyro
        // //targetAngle is the desired angle to rotate to
        // //forwardSpeed and reverseSpeed are variables between 1.0 and -1.0, the
        // larger they are the faster you will turn.
        // // making one the negative of the other will

        // if ( Math.abs(RobotGyro.m_gyro.getAngle()) >= Math.abs(targetAngle)) {
        // //we're there, stop turning
        // Drivetrain.m_drive.tankDrive(0,0);
        // } else if ( targetAngle > RobotGyro.m_gyro.getAngle()) {
        // //rotate clockwise
        // Drivetrain.m_drive.tankDrive(reverseSpeed, forwardSpeed);
        // }else {
        // //rotate counter-clockwise
        // Drivetrain.m_drive.tankDrive(forwardSpeed, reverseSpeed);
        // }

        break;
      case "Default": // DEFAULT
      default:
        System.out.println("Time: " + Constants.tAuton);

        Drivetrain.m_drive.arcadeDrive(0.0, 0.0);
        break;
    }
  }

  @Override
  public void teleopInit() {
    // TODO Auto-generated method stub
    super.teleopInit();

    AirCompressor.clearFaults();
    Shooter.m_Shooter.setMaxAcceleration(Constants.pidMaxAcceleration); // Set max acceleration to 20,000 RPM/s
    Shooter.m_Shooter.setMaxJerk(Constants.pidMaxJerk); // Set max jerk to 31,250 RPM/s^2
    Shooter.m_Shooter.setMaxSpeed(Constants.pidMaxSpeed);

    Shooter.m_Shooter.setPID(Constants.pidKp, Constants.pidKi, Constants.pidKd, Constants.pidKf, Constants.pidKb); // Configure
                                                                                                                   // gains
    // region Venom Defaults
    // m_Shooter.setMaxAcceleration(20000); // Set max acceleration to 20,000 RPM/s
    // m_Shooter.setMaxJerk(31250); // Set max jerk to 31,250 RPM/s^2
    // m_Shooter.setPID(0.195, 0.010, 0.0, 0.184, 0.0); // Configure PIDgains
    // endregion Venom Defaults

    // Kp, Ki, Kd, Kf, b
    // Kp lower shoots lower

  }

  @Override
  public void teleopPeriodic() {
    Shuffleboard.startRecording();

    Drivetrain.venomBrakeMode(2);

    try {
      LimelightVision.setLEDMode(Constants.LimelightConstants.ll_on);
    } catch (Exception name) {
      // System.out.println("Exception: " + name);
      // System.out.println("Error 396 - Unable to connect to the LimeLight");
    }

    // region JS1

    // if (js1.getRawButtonPressed(Constants.btnD_A)) {
    // // Gyro drive straight
    // double turningValue = (RobotGyro.kAngleSetpoint -
    // RobotGyro.m_gyro.getAngle()) * RobotGyro.kP;
    // // Invert the direction of the turn if we are going backwards
    // turningValue = Math.copySign(turningValue, -js1_L_UpDown);
    // Drivetrain.m_drive.arcadeDrive(-js1_L_UpDown, turningValue);
    // }
    // if (js1.getRawButtonReleased(Constants.btnD_A)) {

    if (c_SelectedDriveMode == "Tank") {
      Drivetrain.m_drive.tankDrive(-js1_L_UpDown, js1_R_UpDown); // regular teleop driving
    } else {
      Drivetrain.m_drive.arcadeDrive(-js1_L_UpDown, js1_R_LeftRight); // regular teleop driving
    }

    //
    // }

    // Gyro Driving - Start
    // double turningValue = (Drivetrain.kAngleSetpoint -
    // RobotGyro.m_gyro.getAngle()) * RobotGyro.kP;
    // // Invert the direction of the turn if we are going backwards
    // // turningValue = Math.copySign(turningValue, m_joystick.getY());
    // turningValue = Math.copySign(turningValue, js1_R_LeftRight);
    // // m_myRobot.arcadeDrive(m_joystick.getY(), turningValue);
    // Drivetrain.m_drive.arcadeDrive(-js1_L_UpDown, turningValue);
    // Gyro Driving - End

    // region Shooter - Reverse
    int js1_ReverseShooter = 0;
    int js1_ForwardShooter = 0;
    int js1_LL_LeftRight = 0;
    int js1_LL_LeftRightDistance = 0;

    switch (c_SelectedControllerDriver) {
      case "XBox":
        js1_ReverseShooter = Constants.btnXB_A;
        js1_ForwardShooter = Constants.btnXB_B;
        js1_LL_LeftRight = Constants.btnXB_X;
        js1_LL_LeftRightDistance = Constants.btnXB_Y;
        break;
      case "LogitechD":
        js1_ReverseShooter = Constants.btnD_LT;
        js1_ForwardShooter = Constants.btnD_RT;

        js1_LL_LeftRight = Constants.btnD_X;
        js1_LL_LeftRightDistance = Constants.btnD_Y;
        break;
      case "LogitechX":
        js1_ReverseShooter = Constants.btnX_A;
        js1_ForwardShooter = Constants.btnX_B;

        js1_LL_LeftRight = Constants.btnX_X;
        js1_LL_LeftRightDistance = Constants.btnX_Y;
        break;
    }

    if (js1.getRawButtonPressed(js1_ReverseShooter)) {
      Shooter.m_Shooter.enable();
      // m_Shooter.setCommand(ControlMode.SpeedControl, 1500); // Spin the motor at x
      // RPM.
      Shooter.m_Shooter.set(Constants.m_Shooter_Speed_Reverse);
    }
    if (js1.getRawButtonReleased(js1_ReverseShooter)) {
      Shooter.m_Shooter.stopMotor();
    }
    // endregion Shooter - Reverse

    // region Shooter - Forward
    if (js1.getRawButtonPressed(js1_ForwardShooter)) {
      Shooter.m_Shooter.enable();
      // Shooter.m_Shooter.setCommand(ControlMode.SpeedControl, 1500); // Spin the
      // motor at x RPM.
      Shooter.Shooter_SetRPM(1500);
      // m_Shooter.set(Constants.m_Shooter_Speed);
    }
    if (js1.getRawButtonReleased(js1_ForwardShooter)) {
      Shooter.m_Shooter.stopMotor();
    }
    // endegion Shooter - Forward

    if (js1.getRawButton(js1_LL_LeftRight)) {
      LimelightBasic();
    }
    if (js1.getRawButton(js1_LL_LeftRightDistance)) {
      LimelightDistance();
    }

    // region Pneumatics
    if (js1.getRawButton(Constants.btnD_RB)) { // Up
      p_Intake.set(DoubleSolenoid.Value.kForward); // Up
    } else if (js1.getRawButton(Constants.btnD_LB)) { // Down
      p_Intake.set(DoubleSolenoid.Value.kReverse); // Down
    } else {
      p_Intake.set(DoubleSolenoid.Value.kOff);
    }

    // if (js1.getRawButton(Constants.btnD_A)) { // Down
    // p_Rachet.set(DoubleSolenoid.Value.kForward); // Down
    // } else if (js1.getRawButton(Constants.btnD_B)) { // Up
    // p_Rachet.set(DoubleSolenoid.Value.kReverse); // Up
    // } else {
    // p_Rachet.set(DoubleSolenoid.Value.kOff);
    // }
    // endregion Pneumatics

    // p_Rachet.set(js1.getRawButton(Constants.btnD_A));

    // endregion JS1

    // region JS2
    // region PID Shooter
    // SmartDashboard.putString("Shooter_Temperature", m_Shooter.getTemperature() +
    // " C");
    // SmartDashboard.putNumber("Shooter_Output_Current ",
    // Shooter.m_Shooter.getOutputCurrent());
    // SmartDashboard.putNumber("Shooter_Output_Voltage ",
    // Shooter.m_Shooter.getOutputVoltage());
    // SmartDashboard.putNumber("Shooter_Output_Speed ", Shooter.m_Shooter.getSpeed());
    if (js2.getRawButtonPressed(Constants.btnD_A)) {
      Shooter.m_Shooter.enable();
      // Shooter.m_Shooter.setCommand(ControlMode.SpeedControl, 1500); // Spin the
      // motor at x RPM.
      Shooter.Shooter_SetRPM(1100);
      // m_Shooter.set(Constants.m_Shooter_Speed_Long);
    }
    if (js2.getRawButtonReleased(Constants.btnD_A)) {
      Shooter.m_Shooter.stopMotor();
    }
    
    
    if (js2.getRawButtonPressed(Constants.btnD_Y)) {
      Shooter.m_Shooter.enable();
      // Shooter.m_Shooter.setCommand(ControlMode.SpeedControl, 1500); // Spin the
      // motor at x RPM.
      Shooter.Shooter_SetRPM(1100);
      // m_Shooter.set(Constants.m_Shooter_Speed_Long);
    }
    if (js2.getRawButtonReleased(Constants.btnD_Y)) {
      Shooter.m_Shooter.stopMotor();
    }
    
    
    if (js2.getRawButtonPressed(Constants.btnD_X)) {
      Shooter.m_Shooter.enable();
      // Shooter.m_Shooter.setCommand(ControlMode.SpeedControl, 1500); // Spin the
      // motor at x RPM.
      Shooter.Shooter_SetRPM(1500);
      // m_Shooter.set(Constants.m_Shooter_Speed_Long);
    }
    if (js2.getRawButtonReleased(Constants.btnD_X)) {
      Shooter.m_Shooter.stopMotor();
    }

    if (js2.getRawButtonPressed(Constants.btnD_B)) {
      Shooter.m_Shooter.enable();
      // Shooter.m_Shooter.setCommand(ControlMode.SpeedControl, 1500); // Spin the
      // motor at x RPM.
      Shooter.Shooter_SetRPM(1750);
      // m_Shooter.set(Constants.m_Shooter_Speed_Long);
    }
    if (js2.getRawButtonReleased(Constants.btnD_B)) {
      Shooter.m_Shooter.stopMotor();
    }


    // endegion PID Shooter

    // // region Uptake
    if (js2.getRawButtonPressed(Constants.btnD_RT)) {
      m_UpTake.set(0.5); // Down
    }
    if (js2.getRawButtonReleased(Constants.btnD_RT)) {
      m_UpTake.set(0.0);
    }
    if (js2.getRawButtonPressed(Constants.btnD_RB)) {
      m_UpTake.set(-0.5); // Up
    }
    if (js2.getRawButtonReleased(Constants.btnD_RB)) {
      m_UpTake.set(0.0);
    }
    // // endregion Uptake

    // // region Intake
    if (js2.getRawButtonPressed(Constants.btnD_LB)) {
      m_InTake.set(-0.5); // In
    }
    if (js2.getRawButtonReleased(Constants.btnD_LB)) {
      m_InTake.set(0.0);
    }
    if (js2.getRawButtonPressed(Constants.btnD_LT)) {
      m_InTake.set(0.5); // Out
    }
    if (js2.getRawButtonReleased(Constants.btnD_LT)) {
      m_InTake.set(0.0);
    }
    // // endregion Intake

    // region Uptake and Intake
    // if (js2.getRawButtonPressed(Constants.btnD_RT)) {
    // m_UpTake.set(Constants.uptake_down); // Down
    // m_InTake.set(Constants.intake_out); // Out
    // }
    // if (js2.getRawButtonReleased(Constants.btnD_RT)) {
    // m_UpTake.set(0.0);
    // m_InTake.set(0.0);
    // }
    // if (js2.getRawButtonPressed(Constants.btnD_RB)) {
    // m_UpTake.set(Constants.uptake_up);
    // m_InTake.set(Constants.intake_in);
    // }
    // if (js2.getRawButtonReleased(Constants.btnD_RB)) {
    // m_UpTake.set(0.0);
    // m_InTake.set(0.0);
    // }
    // endregion Uptake and Intake

    // region Climber - Up and Down
    // if (js2.getRawButtonPressed(Constants.btnD_Start)) {
    // m_Climber01.set(Constants.climb_up); // Climber Up
    // // m_Climber02.set(Constants.climb_up); // Climber Up
    // }
    // if (js2.getRawButtonReleased(Constants.btnD_Start)) {
    // m_Climber01.set(0.0);
    // // m_Climber02.set(0.0);
    // }

    // if (js2.getRawButtonPressed(Constants.btnD_Back)) {
    // m_Climber01.set(Constants.climb_down); // Climber Down
    // // m_Climber02.set(Constants.climb_down); // Climber Down
    // }
    // if (js2.getRawButtonReleased(Constants.btnD_Back)) {
    // m_Climber01.set(0.0);
    // // m_Climber02.set(0.0);
    // }
    // endregion Climber - Up and Down

    // endregion JS2
  }

  @Override
  public void testInit() {
    Constants.tAuton = 0; // Timer
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    try {
      LimelightVision.setLEDMode(Constants.LimelightConstants.ll_off);
    } catch (Exception name) {
      // System.out.println("Exception: " + name);
      // System.out.println("Error 396 - Unable to connect to the LimeLight");
    }
    Constants.tAuton += 1; // Timer
    System.out.println("Timer: " + Constants.tAuton + "|" + System.currentTimeMillis());
  }

  @Override
  public void disabledInit() {
    // TODO Auto-generated method stub
    super.disabledInit();
    // System.out.println("Robot.disabledInit()");

    Shuffleboard.stopRecording();
    Drivetrain.resetEncoders();

    AirCompressor.disable();
    try {
      LimelightVision.setLEDMode(Constants.LimelightConstants.ll_off);
    } catch (Exception name) {
      // System.out.println("Exception: " + name);
      // System.out.println("Error 396 - Unable to connect to the LimeLight");
    }

  }

  @Override
  public void disabledPeriodic() {
    // System.out.println("Robot.disabledPeriodic()");

    super.disabledPeriodic();

  }
}
