// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// region Imports
// region FRC8767
import frc.robot.Constants;
import frc.robot.subsystems.GetMacAddress;
// endregion FRC8767
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// https://maven.ctr-electronics.com/release/com/ctre/phoenix/Phoenix-frc2022-latest.json
import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import javax.lang.model.util.ElementScanner6;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
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
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private RelativeEncoder m_left01Encoder;
  private RelativeEncoder m_left02Encoder;
  private RelativeEncoder m_right01Encoder;
  private RelativeEncoder m_right02Encoder;
  // region Drive Train
  private CANSparkMax m_left01Motor = new CANSparkMax(2, MotorType.kBrushless);
  private CANSparkMax m_left02Motor = new CANSparkMax(5, MotorType.kBrushless);
  private CANSparkMax m_right01Motor = new CANSparkMax(4, MotorType.kBrushless);
  private CANSparkMax m_right02Motor = new CANSparkMax(3, MotorType.kBrushless);

  MotorControllerGroup m_left = new MotorControllerGroup(m_left01Motor, m_left02Motor);
  MotorControllerGroup m_right = new MotorControllerGroup(m_right01Motor, m_right02Motor);
  DifferentialDrive m_drive = new DifferentialDrive(m_left, m_right);
  // endregion Drive Train

  private CANSparkMax m_intakeShooter = new CANSparkMax(1, MotorType.kBrushless);
  private CANSparkMax m_shooter = new CANSparkMax(6, MotorType.kBrushless);

  // region Climber Motor Assignment
  private DifferentialDrive m_climb;
  WPI_TalonSRX m_Climber01 = new WPI_TalonSRX(Constants.m_Climber01);
  WPI_TalonSRX m_Climber02 = new WPI_TalonSRX(Constants.m_Climber02);

  // endregion Climber Motor Assignment

  // region Joysticks
  private final Joystick js1 = new Joystick(Constants.js1);
  private final Joystick js2 = new Joystick(Constants.js2);
  // endregion Joysticks

  Servo s_RachetPull = new Servo(Constants.s_RatchetPull);
  Servo s_IntakePush = new Servo(Constants.s_IntakePush);

  WPI_TalonSRX m_Intake = new WPI_TalonSRX(Constants.m_Intake);

  private PowerDistribution m_pdp = new PowerDistribution();
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

  // region Shuffleboard Chooser
  private String c_SelectedAuton;
  private String c_SelectedDelay;
  private String c_SelectedDriveMode;
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
  private final SendableChooser<String> c_DriveMode = new SendableChooser<>();
  private final SendableChooser<String> c_DriveYSpeed = new SendableChooser<>();
  private final SendableChooser<String> c_DriveZASpeed = new SendableChooser<>();
  private final SendableChooser<String> c_ClimberSpeed = new SendableChooser<>();
  private final SendableChooser<String> c_IntakeShooterSpeed = new SendableChooser<>();
  private final SendableChooser<String> c_ShooterSpeed = new SendableChooser<>();

  // endregion Shuffleboard Chooser
  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    c_Auton.setDefaultOption("Default Auton", "Default");
    c_Auton.addOption("Feeder Bot", "feederbot");
    c_Auton_Delay.setDefaultOption("None", "None");
    c_Auton_Delay.addOption("1 second", "1");
    c_Auton_Delay.addOption("2 second", "2");
    c_Auton_Delay.addOption("3 second", "3");
    c_Auton_Delay.addOption("4 second", "4");
    c_Auton_Delay.addOption("5 second", "5");
    c_Auton_Delay.addOption("6 second", "6");
    // region Shuffleboard Choosers
    c_ControllerDriver.setDefaultOption("LogitechD", "LogitechD");
    c_ControllerDriver.addOption("LogitechX", "LogitechX");
    c_ControllerDriver.addOption("XBox", "XBox");
    
    c_ControllerOperator.setDefaultOption("LogitechD", "LogitechD");
    c_ControllerOperator.addOption("LogitechX", "LogitechX");
    c_ControllerOperator.addOption("XBox", "XBox");
  m_left01Motor.setInverted(false);
    m_left02Motor.setInverted(false);
    m_right01Motor.setInverted(true);
    m_right02Motor.setInverted(true);

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

    c_Auton.setDefaultOption("Default Auton", "Default");

    c_Auton.addOption("Feeder Bot", "feederbot");

    c_DriveMode.setDefaultOption("Arcade", "Arcade");
    c_DriveMode.addOption("Tank", "Tank");

    c_ControllerDriver.setDefaultOption("LogitechD", "LogitechD");
    c_ControllerDriver.addOption("LogitechX", "LogitechX");
    c_ControllerDriver.addOption("XBox", "XBox");

    c_ControllerOperator.setDefaultOption("LogitechD", "LogitechD");
    c_ControllerOperator.addOption("LogitechX", "LogitechX");
    c_ControllerOperator.addOption("XBox", "XBox");
    c_Auton_Delay.setDefaultOption("None", "None");

    c_Auton_Delay.addOption("1 second", "1");
    c_Auton_Delay.addOption("2 second", "2");
    c_Auton_Delay.addOption("3 second", "3");
    c_Auton_Delay.addOption("4 second", "4");
    c_Auton_Delay.addOption("5 second", "5");
    c_Auton_Delay.addOption("6 second", "6");
	
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
    c_ShooterSpeed.setDefaultOption(".80", ".80");
    c_ShooterSpeed.addOption(".85", ".85");
    c_ShooterSpeed.addOption(".90", ".90");
    c_ShooterSpeed.addOption("1.00", "1.00");

    c_ClimberSpeed.addOption(".50", ".50");
    c_ClimberSpeed.addOption(".55", ".55");
    c_ClimberSpeed.addOption(".60", ".60");
    c_ClimberSpeed.addOption(".65", ".65");
    c_ClimberSpeed.addOption(".70", ".70");
    c_ClimberSpeed.addOption(".75", ".75");
    c_ClimberSpeed.setDefaultOption(".80", ".80");
    c_ClimberSpeed.addOption(".85", ".85");
    c_ClimberSpeed.addOption(".90", ".90");
    // endregion Shuffleboard Choosers
    m_climb = new DifferentialDrive(m_Climber01, m_Climber02);

    // region Camera
    UsbCamera camera01 = CameraServer.startAutomaticCapture();
    camera01.setFPS(15);
    camera01.setResolution(640, 480);
    // endregion Camera

    m_Climber01.setNeutralMode(NeutralMode.Brake);
    m_Climber02.setNeutralMode(NeutralMode.Brake);
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

  

    // region Shuffleboard
    c_SelectedClimberSpeed = c_ClimberSpeed.getSelected();
    c_SelectedIntakeShooterSpeed = c_IntakeShooterSpeed.getSelected();
    c_SelectedShooterSpeed = c_ShooterSpeed.getSelected();

    c_SelectedDriveYSpeed = c_DriveYSpeed.getSelected();
    c_SelectedDriveZASpeed = c_DriveZASpeed.getSelected();
    c_SelectedDriveMode = c_DriveMode.getSelected();
    c_SelectedControllerDriver = c_ControllerDriver.getSelected();
    c_SelectedControllerOperator = c_ControllerOperator.getSelected();

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
    double dL_UD_Speed = Double.parseDouble(c_SelectedDriveYSpeed);
    double dR_UD_Speed = Double.parseDouble(c_SelectedDriveYSpeed);
    double dR_LR_Speed = Double.parseDouble(c_SelectedDriveZASpeed);
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

    SmartDashboard.putData("Auton choices", c_Auton);
    SmartDashboard.putData("Auto Delay", c_Auton_Delay);
    SmartDashboard.putData("Drive Mode", c_DriveMode);
    SmartDashboard.putData("Driver Controller", c_ControllerDriver);
    SmartDashboard.putData("Operator Controller", c_ControllerOperator);
    SmartDashboard.putData("Drive Speed", c_DriveYSpeed);
    SmartDashboard.putData("Turning Speed", c_DriveZASpeed);

    SmartDashboard.putData("Climber Speed", c_ClimberSpeed);
    SmartDashboard.putData("Intake Shooter Speed", c_IntakeShooterSpeed);
    SmartDashboard.putData("Shooter Speed", c_ShooterSpeed);

    // SmartDashboard.putString("RoboRio", GetMacAddress.getRIOMAC());

    SmartDashboard.putBoolean("Enabled", isEnabled());
    boolean bBattery;
    if (m_pdp.getVoltage() >= 12) {
      bBattery = true;
    } else {
      bBattery = false;
    }
    SmartDashboard.putBoolean("Battery", bBattery);
    // region diagnostic

    SmartDashboard.putNumber("Timer", rTimer.get());

    SmartDashboard.putNumber("PDP Voltage", m_pdp.getVoltage());

    SmartDashboard.putNumber("PDP Temperature", m_pdp.getTemperature());

    SmartDashboard.putNumber("left 01 Encoder", m_left01Encoder.getPosition());
    SmartDashboard.putNumber("left 02 Encoder", m_left02Encoder.getPosition());
    SmartDashboard.putNumber("right 01 Encoder", m_right01Encoder.getPosition());
    SmartDashboard.putNumber("right 02 Encoder", m_right02Encoder.getPosition());

    SmartDashboard.putNumber("left 01 Velocity", m_left01Encoder.getVelocity());
    SmartDashboard.putNumber("left 02 Velocity", m_left02Encoder.getVelocity());
    SmartDashboard.putNumber("right 01 Velocity", m_right01Encoder.getVelocity());
    SmartDashboard.putNumber("right 02 Velocity", m_right02Encoder.getVelocity());

    // endregion diagnostic
    // endregion Shuffleboard

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
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    c_SelectedAuton = c_Auton.getSelected();
    c_SelectedDelay = c_Auton_Delay.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + c_SelectedAuton);
    rTimer.reset();
    Constants.tAuton = 0;

    // Timer
    rTimer.start();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    Shuffleboard.startRecording();
    switch (c_SelectedAuton) {
      case "feederbot":
        // Put custom auto code here
        // If is has been less than 2 seconds since autonomous started, drive forwards
        s_IntakePush.set(1.0);
        if (rTimer.get() < 2.0) {
          m_drive.arcadeDrive(0.5, 0.0);
        }
        // If more than 2 seconds have elapsed, stop driving and turn off the timer
        else {
          m_drive.arcadeDrive(0, 0);
          m_drive.stopMotor();

          rTimer.stop();
        }
        break;

      case "Default": // DEFAULT
      default:

        // m_frontLeft.set(0.5);
        // m_frontRight.set(0.5);
        // m_rearLeft.set(0.5);
        // m_rearRight.set(0.5);

        System.out.println("Time: " + Constants.tAuton);
        s_IntakePush.set(1.0);
        // m_drive.arcadeDrive(0.0, 0.0);
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    super.teleopInit();
    m_Climber01.configFactoryDefault();
    m_Climber02.configFactoryDefault();

    m_left01Encoder.setPosition(0);
    m_left02Encoder.setPosition(0);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    Shuffleboard.startRecording();

  //  double js1LToggle = 0.0;
  //  if (Math.abs(-js1.getRawAxis(1)) > 0.15) {
  //    js1LToggle = (Math.abs(-js1.getRawAxis(1)) - 0.15) / (1 - 0.15);
  //    js1LToggle = Math.copySign(js1LToggle * js1LToggle, -js1.getRawAxis(1));
  //  }

   // double js1RToggle = 0.0;
   // if (Math.abs(js1.getRawAxis(2)) > 0.15) {
   //   js1RToggle = (Math.abs(js1.getRawAxis(2)) - 0.15) / (1 - 0.15);
   //   js1RToggle = Math.copySign(js1RToggle * js1RToggle, js1.getRawAxis(2));
  //  }

    // m_drive.tankDrive(js1.getRawAxis(1), js1.getRawAxis(3));
   // m_drive.arcadeDrive(js1LToggle, js1RToggle);
	
	// Drive with arcade drive.
    // That means that the Y axis drives forward
    // if (c_SelectedDriveMode == "Tank") {
    // m_drive.tankDrive(-js1_R_UpDown, js1_L_UpDown); // regular teleop
    // driving
    // } else {
    m_drive.arcadeDrive(-js1_R_LeftRight, js1_L_UpDown); // regular teleop driving
    // }

    if (js2.getRawButtonPressed(Constants.btnD_RB)) {
      m_intakeShooter.set(0.7);
    }
    if (js2.getRawButtonReleased(Constants.btnD_RB)) {
      m_intakeShooter.set(0.0);
    }

    if (js2.getRawButtonPressed(Constants.btnD_RT)) {
      m_intakeShooter.set(-0.7);
    }
    if (js2.getRawButtonReleased(Constants.btnD_RT)) {
      m_intakeShooter.set(0.0);
    }

    if (js2.getRawButtonPressed(Constants.btnD_X)) {
      m_shooter.set(1);
    }
    if (js2.getRawButtonReleased(Constants.btnD_X)) {
      m_shooter.set(0.0);
    }

    if (js2.getRawButtonPressed(Constants.btnD_A)) {
      m_shooter.set(-1);
    }
    if (js2.getRawButtonReleased(Constants.btnD_A)) {
      m_shooter.set(0.0);
    }

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

    if (js2.getRawButtonPressed(Constants.btnD_Start)) {
      s_RachetPull.set(1.0);
    }
    if (js2.getRawButtonReleased(Constants.btnD_Start)) {
      s_RachetPull.set(0.0);
    }

    if (js2.getRawButtonPressed(Constants.btnD_Back)) {
      s_IntakePush.set(1.0);
    }
    if (js2.getRawButtonReleased(Constants.btnD_Back)) {
      s_IntakePush.set(0.5);
    }

    switch (js2.getPOV()) {
      case 0:
        m_climb.arcadeDrive(0.6, 0);
        break;
      case 180:
        m_climb.arcadeDrive(-0.6, 0);
        break;
      default:
        m_climb.stopMotor();
        break;
    }

  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    super.disabledInit();
    Shuffleboard.stopRecording();
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    super.disabledPeriodic();
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
