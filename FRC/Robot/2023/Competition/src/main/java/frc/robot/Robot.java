// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//region Imports
// Rev Robotics - https://software-metadata.revrobotics.com/REVLib-2023.json
// CTRE - https://maven.ctr-electronics.com/release/com/ctre/phoenix/Phoenix-frc2022-latest.json
// Venom - https://www.playingwithfusion.com/frc/playingwithfusion2023.json

import com.ctre.phoenix.motorcontrol.can.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder; // Specifically needed for encoders
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.hal.ConstantsJNI;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
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
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants;
import javax.lang.model.util.ElementScanner14;

//endregion Imports

public class Robot extends TimedRobot {

  //region Configurations
  //region Limit Switches
  DigitalInput ls_elbow_high;
  DigitalInput ls_elbow_low;
  //endregion Limit Switches

  //region Controllers
  private Joystick js_Driver = new Joystick(constants.js_Driver);
  private Joystick js_Operator = new Joystick(constants.js_Operator);
  //endregion Controllers

  private SparkMaxPIDController m_pidController;
  private SparkMaxPIDController m_drivePID;
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  // private static MjpegServer mjpegServer;
  private static double error = 0.0;
  private static double turn_power = 0.0;
  private static final String kCustomAuto = "My Auto";
  private static final String kDefaultAuto = "Default";
  private static final int kDoubleSolenoidForward = 2;
  private static final int kDoubleSolenoidReverse = 3;
  private static final int kSolenoidButton = 1;

  // public static final ADIS16470_IMU gyro = new ADIS16470_IMU();

  PowerDistribution m_pdh = new PowerDistribution(
    constants.can_id_pdh,
    ModuleType.kRev
  );

  //region Drivetrain Encoders
  static final double COUNTS_PER_MOTOR_REV = 42; // NEO Brushless 1650

  static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV / 70); // Inch to encoder count

  //endregion Drivetrain Encoders

  // The gain for a simple P loop
  static final double gyroP = 1;
  // static final double gyroI = 0.05;

  // region Pneumatics
  PneumaticHub m_ph = new PneumaticHub(constants.can_id_ph);
  DoubleSolenoid p_ds_Drivetrain = m_ph.makeDoubleSolenoid(
    constants.p_ds_Drivetrain_Forward_id,
    constants.p_ds_Drivetrain_Reverse_id
  );
  DoubleSolenoid p_ds_Claw = m_ph.makeDoubleSolenoid(
    constants.p_ds_Claw_Forward_id,
    constants.p_ds_Claw_Reverse_id
  );
  // endregion Pneumatics

  //region Drivetrain
  public CANSparkMax m_frontLeft = new CANSparkMax(
    constants.can_id_frontLeft,
    MotorType.kBrushless
  );
  public CANSparkMax m_frontRight = new CANSparkMax(
    constants.can_id_frontRight,
    MotorType.kBrushless
  );
  public CANSparkMax m_rearLeft = new CANSparkMax(
    constants.can_id_rearLeft,
    MotorType.kBrushless
  );
  public CANSparkMax m_rearRight = new CANSparkMax(
    constants.can_id_rearRight,
    MotorType.kBrushless
  );
  public MecanumDrive m_robotDrive = new MecanumDrive(
    m_frontLeft,
    m_rearLeft,
    m_frontRight,
    m_rearRight
  );
  private RelativeEncoder m_frontLeftEncoder;
  private RelativeEncoder m_rearLeftEncoder;
  private RelativeEncoder m_frontRightEncoder;
  private RelativeEncoder m_rearRightEncoder;
  //endregion Drivetrain

  //region Arm
  public CANSparkMax m_elbow = new CANSparkMax(
    constants.can_id_elbow,
    MotorType.kBrushless
  );
  public CANSparkMax m_intake = new CANSparkMax(
    constants.can_id_intake,
    MotorType.kBrushless
  );
  public CANSparkMax m_extend = new CANSparkMax(
    constants.can_id_extend,
    MotorType.kBrushless
  );
  private RelativeEncoder m_extendEncoder;
  private RelativeEncoder m_elbowEncoder;
  private RelativeEncoder m_intakeEncoder;
  // private RelativeEncoder m_elbowEncoder;
  // private RelativeEncoder m_elevatorEncoder;
  // private RelativeEncoder m_extendEncoder;
  // private SparkMaxPIDController elbowPID;
  // private SparkMaxPIDController elevatorPID;
  // private SparkMaxPIDController extendPID;
  // endregion Arm

  public static Timer rTimer = new Timer();

  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

  public double rotationsY;
  public double rotationsX;
  public double rotationsA;
  public double rotationsB;

  SlewRateLimiter driveX = new SlewRateLimiter(1);
  SlewRateLimiter driveY = new SlewRateLimiter(1);

  //region SmartDashboard
  private String c_SelectedAuton;
  private String c_SelectedDelay;
  private String c_SelectedIntake;
  // private String c_SelectedDriveMode;
  private String c_SelectedControllerDriver;
  private String c_SelectedControllerOperator;
  private String c_SelectedDriveYSpeed;
  private String c_SelectedDriveZASpeed;
  private String c_SelectedDriveXSpeed;
  private final SendableChooser<String> c_Auton = new SendableChooser<>();
  private final SendableChooser<String> c_Auton_Delay = new SendableChooser<>();
  private final SendableChooser<String> c_ControllerDriver = new SendableChooser<>();
  private final SendableChooser<String> c_ControllerOperator = new SendableChooser<>();
  // private final SendableChooser<String> c_DriveMode = new SendableChooser<>();
  private final SendableChooser<String> c_DriveYSpeed = new SendableChooser<>();
  private final SendableChooser<String> c_DriveZASpeed = new SendableChooser<>();
  private final SendableChooser<String> c_DriveXSpeed = new SendableChooser<>();
  private final SendableChooser<String> c_Intake = new SendableChooser<>();

  // endregion SmartDashboard

  // endregion Configurations

  @Override
  public void robotInit() {
    ls_elbow_high = new DigitalInput(1);
    ls_elbow_low = new DigitalInput(2);

    // region SmartDashboard
    c_Intake.setDefaultOption("Active", "Active");
    c_Intake.addOption("Pneumatics", "Pneumatics");
    // Add buttons to set the double
    c_Auton.addOption("Default Auton", "Default");
    c_Auton.setDefaultOption("Position Robot", "position_robot");
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

    c_DriveYSpeed.addOption(".40", ".40");
    c_DriveYSpeed.addOption(".45", ".45");
    c_DriveYSpeed.addOption(".50", ".50");
    c_DriveYSpeed.addOption(".55", ".55");
    c_DriveYSpeed.addOption(".60", ".60");
    c_DriveYSpeed.addOption(".65", ".65");
    c_DriveYSpeed.addOption(".70", ".70");
    c_DriveYSpeed.addOption(".75", ".75");
    c_DriveYSpeed.setDefaultOption(".80", ".80");
    c_DriveYSpeed.addOption(".85", ".85");
    c_DriveYSpeed.addOption(".90", ".90");

    c_DriveZASpeed.addOption(".40", ".40");
    c_DriveZASpeed.addOption(".45", ".45");
    c_DriveZASpeed.addOption(".50", ".50");
    c_DriveZASpeed.addOption(".55", ".55");
    c_DriveZASpeed.setDefaultOption(".60", ".60");
    c_DriveZASpeed.addOption(".65", ".65");
    c_DriveZASpeed.addOption(".70", ".70");
    c_DriveZASpeed.addOption(".75", ".75");
    c_DriveZASpeed.addOption(".80", ".80");
    c_DriveZASpeed.addOption(".85", ".85");
    c_DriveZASpeed.addOption(".90", ".90");

    c_DriveXSpeed.addOption(".40", ".40");
    c_DriveXSpeed.addOption(".45", ".45");
    c_DriveXSpeed.addOption(".50", ".50");
    c_DriveXSpeed.addOption(".55", ".55");
    c_DriveXSpeed.setDefaultOption(".60", ".60");
    c_DriveXSpeed.addOption(".65", ".65");
    c_DriveXSpeed.addOption(".70", ".70");
    c_DriveXSpeed.addOption(".75", ".75");
    c_DriveXSpeed.addOption(".80", ".80");
    c_DriveXSpeed.addOption(".85", ".85");
    c_DriveXSpeed.addOption(".90", ".90");

    // Add number inputs for minimum and maximum pressure
    SmartDashboard.setDefaultNumber("Minimum Pressure (PSI)", 100.0);
    SmartDashboard.setDefaultNumber("Maximum Pressure (PSI)", 120.0);
    // endregion SmartDashboard

    m_pidController = m_elbow.getPIDController();
    // m_elbow.setSoftLimit(SoftLimitDirection.kForward, 15);

    // region Drivetrain
    m_frontLeft.setInverted(false);
    m_frontRight.setInverted(true);
    m_rearLeft.setInverted(false);
    m_rearRight.setInverted(true);
    m_frontLeftEncoder = m_frontLeft.getEncoder();
    m_frontRightEncoder = m_frontRight.getEncoder();
    m_rearLeftEncoder = m_rearLeft.getEncoder();
    m_rearRightEncoder = m_rearRight.getEncoder();
    m_frontLeftEncoder.setPosition(0);
    m_frontRightEncoder.setPosition(0);
    m_rearLeftEncoder.setPosition(0);
    m_rearRightEncoder.setPosition(0);
    // endregion Drivetrain

    m_robotDrive.setDeadband(0.15);

    // region Arm
    m_extend.setInverted(true);
    m_extendEncoder = m_extend.getEncoder();
    m_elbowEncoder = m_elbow.getEncoder();
    m_extendEncoder.setPosition(0);
    m_elbowEncoder.setPosition(0);
    // endregion Arm

    CameraServer.startAutomaticCapture();
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

  @Override
  public void robotPeriodic() {
    // region SmartDashboard
    c_SelectedControllerDriver = c_ControllerDriver.getSelected();
    c_SelectedControllerOperator = c_ControllerOperator.getSelected();
    SmartDashboard.putBoolean("Enabled", isEnabled());
    SmartDashboard.putNumber("Timer", rTimer.get());
    SmartDashboard.putNumber("Voltage", m_pdh.getVoltage());
    SmartDashboard.putData("Auton choices", c_Auton);
    SmartDashboard.putData("Auto Delay", c_Auton_Delay);
    SmartDashboard.putData("Intake", c_Intake);
    SmartDashboard.putData("Driver Controller", c_ControllerDriver);
    SmartDashboard.putData("Operator Controller", c_ControllerOperator);
    SmartDashboard.putData("Drive Speed", c_DriveYSpeed);
    SmartDashboard.putData("Turning Speed", c_DriveZASpeed);
    SmartDashboard.putData("Strafing Speed", c_DriveXSpeed);
    SmartDashboard.putBoolean("Enabled", isEnabled());

    /**
     * Encoder position is read from a RelativeEncoder object by calling the
     * GetPosition() method.
     *
     * GetPosition() returns the position of the encoder in units of revolutions
     */
    SmartDashboard.putNumber("m_FL Position", m_frontLeftEncoder.getPosition());
    SmartDashboard.putNumber(
      "m_FR Position",
      m_frontRightEncoder.getPosition()
    );
    SmartDashboard.putNumber("m_RL Position", m_rearLeftEncoder.getPosition());
    SmartDashboard.putNumber("m_RR Position", m_rearRightEncoder.getPosition());

    /**
     * Encoder velocity is read from a RelativeEncoder object by calling the
     * GetVelocity() method.
     *
     * GetVelocity() returns the velocity of the encoder in units of RPM
     */
    SmartDashboard.putNumber("m_FL Velocity", m_frontLeftEncoder.getVelocity());
    SmartDashboard.putNumber("m_RL Velocity", m_rearLeftEncoder.getVelocity());
    SmartDashboard.putNumber(
      "m_FR Velocity",
      m_frontRightEncoder.getVelocity()
    );
    SmartDashboard.putNumber("m_RR Velocity", m_rearRightEncoder.getVelocity());

    boolean bBattery;
    if (m_pdh.getVoltage() >= 12) {
      bBattery = true;
    } else {
      bBattery = false;
    }
    SmartDashboard.putBoolean("Battery", bBattery);

    SmartDashboard.putBoolean("Elbow High", ls_elbow_high.get());
    SmartDashboard.putBoolean("Elbow Low", ls_elbow_low.get());

    c_SelectedDriveYSpeed = c_DriveYSpeed.getSelected();
    c_SelectedDriveZASpeed = c_DriveZASpeed.getSelected();
    c_SelectedDriveXSpeed = c_DriveXSpeed.getSelected();
    // c_SelectedDriveMode = c_DriveMode.getSelected();

    // region Pneumatics
    /**
     * Get the state of the solenoid.
     *
     * This is just a switch case to better display it on Shuffleboard.
     */
    switch (p_ds_Drivetrain.get()) {
      case kOff:
        SmartDashboard.putString("Get Solenoid", "kOff");
        break;
      case kForward:
        SmartDashboard.putString("Get Solenoid", "kForward");
        break;
      case kReverse:
        SmartDashboard.putString("Get Solenoid", "kReverse");
        break;
      default:
        SmartDashboard.putString("Get Solenoid", "N/A");
        break;
    }

    /**
     * Get pressure from analog channel 0 and display on Shuffleboard.
     */
    SmartDashboard.putNumber(
      "Claw Pressure",
      m_ph.getPressure(constants.p_as_Claw_id)
    );
    SmartDashboard.putNumber(
      "DT Pressure",
      m_ph.getPressure(constants.p_as_Drivetrain_id)
    );

    /**
     * Get fressor running status and display on Shuffleboard.
     */
    SmartDashboard.putBoolean("Compressor Running", m_ph.getCompressor());

    // Get values from Shuffleboard
    double minPressure = SmartDashboard.getNumber(
      "Minimum Pressure (PSI)",
      0.0
    );
    double maxPressure = SmartDashboard.getNumber(
      "Maximum Pressure (PSI)",
      0.0
    );
    // endregion Pneumatics

    // SmartDashboard.putNumber("Drive To", -(COUNTS_PER_INCH * 5));
    SmartDashboard.putNumber("Drive To", -(COUNTS_PER_INCH * 5));
  }

  private double getAverageEncoderPosition() {
    return (
      (m_frontLeftEncoder.getPosition() + m_frontRightEncoder.getPosition()) / 2
    );
  }

  @Override
  public void autonomousInit() {
    c_SelectedAuton = c_Auton.getSelected();
    c_SelectedDelay = c_Auton_Delay.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + c_SelectedAuton);
    rTimer.reset();
    constants.tAuton = 0; // Timer
    rTimer.start();

    m_frontLeftEncoder.setPosition(0);
    m_frontRightEncoder.setPosition(0);
    m_rearLeftEncoder.setPosition(0);
    m_rearRightEncoder.setPosition(0);
    m_frontLeft.setIdleMode(IdleMode.kBrake);
    m_frontRight.setIdleMode(IdleMode.kBrake);
    m_rearLeft.setIdleMode(IdleMode.kBrake);
    m_rearRight.setIdleMode(IdleMode.kBrake);

    // Set properties of drive
    m_robotDrive.setSafetyEnabled(false);
    m_robotDrive.setMaxOutput(0.40);
  }

  @Override
  public void autonomousPeriodic() {
    Shuffleboard.startRecording();

    switch (c_SelectedAuton) {
      case "position_robot":
        // Drive backwards 7 inches
        if (
          // (m_frontLeftEncoder.getPosition() > -(COUNTS_PER_INCH * 5)) &&
          // (m_frontRightEncoder.getPosition() > -(COUNTS_PER_INCH * 5))
          (m_frontLeftEncoder.getPosition() > -42)
        ) {
          // m_robotDrive.driveCartesian(inFR, inFR, inFR);
          // m_frontLeft.set(0.4);
          // m_frontRight.set(0.4);
          // m_rearLeft.set(0.4);
          // m_rearRight.set(0.4);

          // Y Axis is up/down on left side
          // X Axis is left/right on left side
          // Z Axis is left/right on right side
          // Z Rotate is up/down on right side

          // Setpoint is implicitly 0, since we don't want the heading to change
          // double error = -gyro.getRate();

          // Drives forward continuously at half speed, using the gyro to stabilize the
          // heading
          // m_robotTraction.tankDrive(.5 + gyroP * error, .5 - gyroP * error);

          m_robotDrive.driveCartesian(-.5, 0, 0);
        } else {
          // m_robotDrive.stopMotor();
          m_frontLeft.set(0);
          m_frontRight.set(0);
          m_rearLeft.set(0);
          m_rearRight.set(0);
        }

        break;
      case "Default": // DEFAULT
      default:
        System.out.println("Time: " + constants.tAuton);
        m_frontLeft.set(0);
        m_frontRight.set(0);
        m_rearLeft.set(0);
        m_rearRight.set(0);
        break;
    }
  }

  @Override
  public void teleopInit() {
    m_elbowEncoder.setPosition(0);
    m_extendEncoder.setPosition(0);
    m_frontLeftEncoder.setPosition(0);
    m_frontRightEncoder.setPosition(0);
    m_rearLeftEncoder.setPosition(0);
    m_rearRightEncoder.setPosition(0);

    m_frontLeft.setIdleMode(IdleMode.kCoast);
    m_frontRight.setIdleMode(IdleMode.kCoast);
    m_rearLeft.setIdleMode(IdleMode.kCoast);
    m_rearRight.setIdleMode(IdleMode.kCoast);
    m_elbow.setIdleMode(IdleMode.kBrake);
    m_extend.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void teleopPeriodic() {
    // region Pneumatics
    // m_ph.disableCompressor();
    // endregion Pneumatics

    // remember, one rotation of the NEO Motor is 42 encoder count
    rotationsY = 0;
    rotationsX = 0;
    rotationsA = 0;
    rotationsB = 0;
    // region Driver Control
    // Use the joystick X axis for forward movement, Y axis for lateral movement,
    // and Z axis for rotation.
    // xSpeed The robot's speed along the X axis [-1.0..1.0]. Forward is positive,
    // backward is negative.
    // ySpeed The robot's speed along the Y axis [-1.0..1.0]. Left is positive.
    // Strafe (Horizontal Drive)
    // zRotation The robot's rotation rate around the Z axis [-1.0..1.0]. Turn Left
    // or Right

    // Get joystick values and scale
    double speedY =
      -js_Driver.getRawAxis(constants.axisXB_lUpDown) *
      Double.parseDouble(c_SelectedDriveYSpeed);
    double speedX =
      js_Driver.getRawAxis(constants.axisXB_lLeftRight) *
      Double.parseDouble(c_SelectedDriveXSpeed);
    double speedZ =
      js_Driver.getRawAxis(constants.axisXB_rLeftRight) *
      Double.parseDouble(c_SelectedDriveZASpeed);

    // use a slewrate limiter to allow the strafe left and right and forward and
    // backward speed to ramp up and down
    m_robotDrive.driveCartesian(
      driveY.calculate(speedY),
      driveX.calculate(speedX),
      speedZ
    );

    switch (c_SelectedIntake) {
      case "Pneumatics":
        if (js_Operator.getRawButtonPressed(constants.btnD_RB)) {
          p_ds_Drivetrain.set(DoubleSolenoid.Value.kForward);
        }
        if (js_Operator.getRawButtonPressed(constants.btnD_LB)) {
          p_ds_Drivetrain.set(DoubleSolenoid.Value.kReverse);
        }
      case "Default": // DEFAULT
      default:
        if (js_Operator.getRawButtonPressed(constants.btnD_RB)) {
          m_intake.set(0.4);
        } else {
          m_intake.set(0);
        }
        if (js_Operator.getRawButtonPressed(constants.btnD_LB)) {
          m_intake.set(-0.4);
        } else {
          m_intake.set(0);
        }
    }

    // m_robotDrive.driveCartesian(speedY, speedX, speedZ);

    // region Pneumatics
    // when the left trigger is pressed push down the traction wheels, when right
    // trigger is pressed pull them back up
    // default is up
    if (js_Driver.getRawButtonPressed(constants.btnXB_LToggle)) {
      p_ds_Drivetrain.set(DoubleSolenoid.Value.kForward);
    }
    if (js_Driver.getRawButtonPressed(constants.btnXB_RToggle)) {
      p_ds_Drivetrain.set(DoubleSolenoid.Value.kReverse);
    }
    //endregion Pneumatics
    //endregion Driver Control

    // region Operator Control
    // moves the elbow up and down when the left stick is pressed forward or
    // backward
    m_elbow.set(
      js_Operator.getRawAxis(constants.axisXB_lUpDown) *
      js_Operator.getRawAxis(constants.axisXB_lUpDown)
    );

    // when the right stick is pressed forward or backward, move the extend forward
    // or backward
    m_extend.set(js_Operator.getRawAxis(constants.axisXB_rUpDown));

    // region Pneumatics
    // when the Left Bumper is pressed, release the object, when the left trigger is
    // pressed, grab the object
    // default is open
    if (js_Operator.getRawButtonPressed(constants.btnXB_LB)) {
      p_ds_Claw.set(DoubleSolenoid.Value.kReverse);
    }
    if (js_Operator.getRawButtonPressed(constants.btnXB_LToggle)) {
      p_ds_Claw.set(DoubleSolenoid.Value.kForward);
    }
    // endregion Pneumatics
    // endregion Operator Control

  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {
    // when left stick is pressed forward, set elbow forward at 70% speed
    // when left stick is pressed backward, set elbow backward at 70% speed
    // else, get the encoders position and hold it until the next user input

    if (js_Operator.getRawAxis(constants.axisXB_lUpDown) > 0.15) {
      m_elbow.set(0.7);
    } else if (js_Operator.getRawAxis(constants.axisXB_lUpDown) < -0.15) {
      m_elbow.set(-0.7);
    } else {
      m_pidController.setReference(
        m_elbowEncoder.getPosition(),
        CANSparkMax.ControlType.kPosition
      );
    }
    // TO DO: tune the PID values to hold in position with a *CONE*
    // when the "Y" button is pressed, run to a constant position (high goal
    // position)
    if (js_Operator.getRawButtonPressed(constants.btnXB_Y)) {
      m_pidController.setReference(
        rotationsY,
        CANSparkMax.ControlType.kPosition
      );
    }
    // when the "X" button is pressed, run to a constant position (double substation
    // position)
    if (js_Operator.getRawButtonPressed(constants.btnXB_X)) {
      m_pidController.setReference(
        rotationsX,
        CANSparkMax.ControlType.kPosition
      );
    }
    // when the "A" button is pressed, run to a constant position (ground/low goal
    // position)
    if (js_Operator.getRawButtonPressed(constants.btnXB_A)) {
      m_pidController.setReference(
        rotationsA,
        CANSparkMax.ControlType.kPosition
      );
    }
    // when the "B" button is pressed, run to a constant positon (mid goal position)
    if (js_Operator.getRawButtonPressed(constants.btnXB_B)) {
      m_pidController.setReference(
        rotationsB,
        CANSparkMax.ControlType.kPosition
      );
    }
    // if (js_Driver.getRawButton(constants.btnD_X)) {
    // m_elbow.set(0.45);
    // } else if (js_Driver.getRawButton(constants.btnD_A)) {
    // m_elbow.set(-0.45);
    // } else {
    // m_elbow.set(0);
    // }
    // if (js_Driver.getRawButton(constants.btnD_Y)) {
    // m_extend.set(0.8);
    // } else if (js_Driver.getRawButton(constants.btnD_B)) {
    // m_extend.set(-0.8);
    // } else {
    // m_extend.set(0);
    // }
  }
}
