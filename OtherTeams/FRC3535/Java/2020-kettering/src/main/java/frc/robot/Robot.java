/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.frc3535_Variables;
import frc.robot.subsystems.LimelightVision;
import frc.robot.subsystems.AirCompressor;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.Timer;

//region CTRE Talons
import edu.wpi.first.wpilibj.PWMTalonSRX;
import edu.wpi.first.wpilibj.RobotBase;
//endregion CTRE Talons
import edu.wpi.first.wpilibj.Solenoid;

//region Venom Code Imports
import com.playingwithfusion.CANVenom;
import com.playingwithfusion.CANVenom.BrakeCoastMode;
import com.playingwithfusion.CANVenom.ControlMode;
//endregion Venom Code Imports

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  public static Timer rTimer = new Timer();

  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private final DoubleSolenoid p_Intake = new DoubleSolenoid(frc3535_Variables.pcmPort, frc3535_Variables.pIntakeIn,
      frc3535_Variables.pIntakeOut);
  private final DoubleSolenoid p_Rachet = new DoubleSolenoid(frc3535_Variables.pcmPort, frc3535_Variables.pRachetIn,
      frc3535_Variables.pRachetOut);
  // private final Solenoid p_Rachet = new Solenoid(frc3535_Variables.pcmPort,
  // frc3535_Variables.pRachet);
  // region Venom Motors
  public com.playingwithfusion.CANVenom m_Shooter = new CANVenom(frc3535_Variables.m_Shooter); // shooter

  public com.playingwithfusion.CANVenom m_frontLeft = new CANVenom(frc3535_Variables.m_frontLeft);
  public com.playingwithfusion.CANVenom m_rearLeft = new CANVenom(frc3535_Variables.m_rearLeft);
  public com.playingwithfusion.CANVenom m_frontRight = new CANVenom(frc3535_Variables.m_frontRight);
  public com.playingwithfusion.CANVenom m_rearRight = new CANVenom(frc3535_Variables.m_rearRight);
  DifferentialDrive m_drive = new DifferentialDrive(m_frontLeft, m_frontRight);
  // endregion Venom Motors

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

  public void venomBrakeMode(int nMode) {
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

  // region Talon Motors
  PWMTalonSRX m_UpTake = new PWMTalonSRX(frc3535_Variables.m_Uptake);
  PWMTalonSRX m_InTake = new PWMTalonSRX(frc3535_Variables.m_Intake);
  PWMTalonSRX m_Climber01 = new PWMTalonSRX(frc3535_Variables.m_Climber01);
  // PWMTalonSRX m_Climber02 = new PWMTalonSRX(frc3535_Variables.m_Climber02);
  // endregion Talon Motors

  // region Joysticks
  private final Joystick js1 = new Joystick(frc3535_Variables.js1);
  private final Joystick js2 = new Joystick(frc3535_Variables.js2);
  // endregion Joysticks

  // region Dead Band Variables
  double js1_L_Speed = 0; // Speed controls up & down
  double js1_R_Speed = 0; // Speed controls up & down
  double js1_L_Rotate = 0; // Rotate controls left & right
  double js1_R_Rotate = 0; // Rotate controls left & right
  double js1_L_TSpeed = 0;
  double js1_R_TSpeed = 0;
  double js2_L_Speed = 0; // Speed controls up & down
  double js2_R_Speed = 0; // Speed controls up & down
  double js2_L_Rotate = 0; // Rotate controls left & right
  double js2_R_Rotate = 0; // Rotate controls left & right
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
      m_drive.arcadeDrive((getDriveSpeed() * driveScaleDown), (tx * turnScaleDown));
    } else {
      m_drive.arcadeDrive(0.0, 0.0);
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
  private final SendableChooser<String> c_Auton = new SendableChooser<>();
  private final SendableChooser<String> c_Auton_Delay = new SendableChooser<>();
  private final SendableChooser<String> c_DriveMode = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    AirCompressor.clearFaults();

    m_frontLeft.setSafetyEnabled(false);
    m_frontRight.setSafetyEnabled(false);
    m_rearLeft.setSafetyEnabled(false);
    m_rearRight.setSafetyEnabled(false);

    c_Auton.setDefaultOption("Default Auton", "Default");

    c_Auton.addOption("Shoot Then Forward", "Shoot01");
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
    c_Auton_Delay.setDefaultOption("None", "None");
    c_Auton_Delay.addOption("1 second", "1");
    c_Auton_Delay.addOption("2 second", "2");
    c_Auton_Delay.addOption("3 second", "3");
    c_Auton_Delay.addOption("4 second", "4");
    c_Auton_Delay.addOption("5 second", "5");
    c_Auton_Delay.addOption("6 second", "6");
    SmartDashboard.putData("Auto Delay", c_Auton_Delay);
    // c_DriveMode.setDefaultOption("Arcade", "Arcade");
    // c_DriveMode.addOption("Tank", "Tank");
    // c_DriveMode.addOption("Tank", "Tank");
    // SmartDashboard.putData("Drive Mode", c_DriveMode);

    m_rearLeft.follow(m_frontLeft);
    m_rearRight.follow(m_frontRight);

    CameraServer camera01 = CameraServer.getInstance();
    camera01.startAutomaticCapture(0);

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
    double speed = m_Shooter.getSpeed();

    AirCompressor.enable();
    SmartDashboard.putBoolean("Compressor", AirCompressor.status());

    m_Climber01.setInverted(true);

    // region Limelight Vision
    try {
      LimelightVision.setLEDMode(frc3535_Variables.ll_on);
      SmartDashboard.putNumber("limelight_X", LimelightVision.getTX());
      SmartDashboard.putNumber("limelight_Y", LimelightVision.getTY());
      SmartDashboard.putNumber("limelight_Area", LimelightVision.getTA());
      SmartDashboard.putNumber("limelight_Latency", LimelightVision.getTL());
      SmartDashboard.putNumber("limelight_Valid_Target", LimelightVision.getTV());
      SmartDashboard.putNumber("limelight_Skew", LimelightVision.getTS());
      SmartDashboard.putNumber("limelight_Steering_Adjust", LimelightVision.getSteeringAdjust());

    } catch (Exception name) {
      // System.out.println("Exception: " + name);
      // System.out.println("Error 272 - Unable to connect to the LimeLight");
      bLimeLightConnected = false;
    }
    // endregion Limelight Vision

    // region Deadbands

    js1_L_Speed = js1.getRawAxis(1); // Y Axis - Up and Down
    js1_L_Rotate = js1.getRawAxis(0); // X Axis - Left and Right
    js1_R_Speed = -js1.getRawAxis(3); // Z Rotate - Up and Down
    js1_R_Rotate = js1.getRawAxis(2); // Z Axis - Left and Right

    js2_L_Speed = js2.getRawAxis(1); // Y Axis - Up and Down
    js2_L_Rotate = js2.getRawAxis(0); // X Axis - Left and Right
    js2_R_Speed = -js2.getRawAxis(3); // Z Rotate - Up and Down
    js2_R_Rotate = js2.getRawAxis(2); // Z Axis - Left and Right

    if ((js1_L_Speed > frc3535_Variables.js1_L_UpDown_Deadband_Low)) {
      js1_L_Speed = ((js1_L_Speed - frc3535_Variables.js1_L_UpDown_Deadband_Low)
          * frc3535_Variables.js1_L_UpDown_Deadband_High);
    } else if ((js1_L_Speed < -frc3535_Variables.js1_L_UpDown_Deadband_Low)) {
      js1_L_Speed = ((js1_L_Speed + frc3535_Variables.js1_L_UpDown_Deadband_Low)
          * frc3535_Variables.js1_L_UpDown_Deadband_High);
    } else {
      js1_L_Speed = 0; // If between boundaries. Do nothing.
    }
    if ((js1_R_Rotate > frc3535_Variables.js1_R_LeftRight_Deadband_Low)) {
      js1_R_Rotate = ((js1_R_Rotate - frc3535_Variables.js1_R_LeftRight_Deadband_Low)
          * frc3535_Variables.js1_R_LeftRight_Deadband_High);
    } else if ((js1_R_Rotate < -frc3535_Variables.js1_R_LeftRight_Deadband_Low)) {
      js1_R_Rotate = ((js1_R_Rotate + frc3535_Variables.js1_R_LeftRight_Deadband_Low)
          * frc3535_Variables.js1_R_LeftRight_Deadband_High);
    } else {
      js1_R_Rotate = 0; // If between boundaries. Do nothing.
    }

    if ((js2_L_Speed > frc3535_Variables.js2_L_UpDown_Deadband_Low)) {
      js2_L_Speed = ((js2_L_Speed - frc3535_Variables.js2_L_UpDown_Deadband_Low)
          * frc3535_Variables.js2_L_UpDown_Deadband_High);
    } else if ((js2_L_Speed < -frc3535_Variables.js2_L_UpDown_Deadband_Low)) {
      js2_L_Speed = ((js2_L_Speed + frc3535_Variables.js2_L_UpDown_Deadband_Low)
          * frc3535_Variables.js2_L_UpDown_Deadband_High);
    } else {
      js2_L_Speed = 0; // If between boundaries. Do nothing.
    }
    if ((js2_R_Speed > frc3535_Variables.js2_R_UpDown_Deadband_Low)) {
      js2_R_Speed = ((js2_R_Speed - frc3535_Variables.js2_R_UpDown_Deadband_Low)
          * frc3535_Variables.js2_R_UpDown_Deadband_High);
    } else if ((js2_R_Speed < -frc3535_Variables.js2_R_UpDown_Deadband_Low)) {
      js2_R_Speed = ((js2_R_Speed + frc3535_Variables.js2_R_UpDown_Deadband_Low)
          * frc3535_Variables.js2_R_UpDown_Deadband_High);
    } else {
      js2_R_Speed = 0; // If between boundaries. Do nothing.
    }

    // endregion Deadbands

    SmartDashboard.putNumber("js1LDB", js1_L_Speed);
    SmartDashboard.putNumber("js1RDB", js1_R_Rotate);

    SmartDashboard.putNumber("js1L", js1.getY());
    SmartDashboard.putNumber("js1R", js1.getZ());

    SmartDashboard.putNumber("Front Left", m_frontLeft.getPosition());
    SmartDashboard.putNumber("Front Right", m_frontRight.getPosition());
    // System.out.println("Brake/Coast " + m_frontLeft.getBrakeCoastMode());

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
    c_SelectedAuton = c_Auton.getSelected();
    c_SelectedDelay = c_Auton_Delay.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + c_SelectedAuton);

    frc3535_Variables.tAuton = 0; // Timer

    rTimer.start();

  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {

    Shuffleboard.startRecording();
    SmartDashboard.putNumber("Timer", rTimer.get());

    p_Intake.set(DoubleSolenoid.Value.kReverse); // Down

    venomBrakeMode(1);
    frc3535_Variables.tAuton += 1; // Timer

    m_frontLeft.enable();
    m_frontRight.enable();
    m_rearLeft.enable();
    m_rearRight.enable();
    m_rearLeft.follow(m_frontLeft);
    m_rearRight.follow(m_frontRight);

    switch (c_SelectedAuton) {
      case "Forward25":
        // Drive Forward 25 Encoder Count
        if (m_frontLeft.getPosition() < 30) {
          m_drive.arcadeDrive(0.5, 0.0);
          System.out.println("Front Left: " + m_frontLeft.getPosition());
          System.out.println("Front Right: " + m_frontRight.getPosition());
        }
        break;
      case "Forward50":
        // Drive Forward 50 Encoder Count
        if (m_frontLeft.getPosition() < 55) {
          m_drive.arcadeDrive(0.5, 0.0);
          System.out.println("Front Left: " + m_frontLeft.getPosition());
          System.out.println("Front Right: " + m_frontRight.getPosition());
        }
        break;
      case "Backward25":
        // Drive Backward 25 Encoder Count
        if (m_frontLeft.getPosition() > -30) {
          m_drive.arcadeDrive(-0.5, 0.0);
          System.out.println("Front Left: " + m_frontLeft.getPosition());
          System.out.println("Front Right: " + m_frontRight.getPosition());
        }
        break;
      case "Backward50":
        // Drive Backward 50 Encoder Count
        if (m_frontLeft.getPosition() > -55) {
          m_drive.arcadeDrive(-0.5, 0.0);
          System.out.println("Front Left: " + m_frontLeft.getPosition());
          System.out.println("Front Right: " + m_frontRight.getPosition());
        }
        break;

      case "Shoot01":
        // Spin up the shooter motor, move forward 5 encoder counts, feed the balls to
        // the shooter, stop shooter and feed motors, Drive forward 10 encoder count
        if (rTimer.get() < 3) { // less than 250
          m_Shooter.set(frc3535_Variables.m_Shooter_Speed);
        }

        if (rTimer.get() > 3 && rTimer.get() < 5) { // 250 - 650
          // Drive Forward 5 Encoder Count
          if (m_frontLeft.getPosition() < 5) {
            m_drive.arcadeDrive(0.5, 0.0);
            System.out.println("Front Left: " + m_frontLeft.getPosition());
            System.out.println("Front Right: " + m_frontRight.getPosition());
          }
        }

        if (rTimer.get() > 5 && rTimer.get() < 13) { // 250 - 650
          // Shoot out balls
          m_UpTake.set(-0.5); // Up
        }

        if (rTimer.get() > 13 && rTimer.get() < 15) { // 650 - 750

          m_UpTake.set(0.0);
          m_Shooter.set(0.0);

          // Drive Forward 10 Encoder Count
          if (m_frontLeft.getPosition() < 15) {
            m_drive.arcadeDrive(0.65, 0.0);
            System.out.println("Front Left: " + m_frontLeft.getPosition());
            System.out.println("Front Right: " + m_frontRight.getPosition());
          }
        }
        break;
      case "Shoot02":
        // Spin up the shooter motor, move forward 5 encoder counts, feed the balls to
        // the shooter, stop shooter and feed motors, Drive backward 15 encoder count

        if (rTimer.get() < 3) { // less than 250

          m_Shooter.set(frc3535_Variables.m_Shooter_Speed);
        }

        if (rTimer.get() > 3 && rTimer.get() < 5) { // 250 - 650
          // Drive Forward 5 Encoder Count
          if (m_frontLeft.getPosition() < 5) {
            m_drive.arcadeDrive(0.5, 0.0);
            System.out.println("Front Left: " + m_frontLeft.getPosition());
            System.out.println("Front Right: " + m_frontRight.getPosition());
          }
        }

        if (rTimer.get() > 5 && rTimer.get() < 13) { // 250 - 650
          // Shoot out balls
          m_UpTake.set(-0.5); // Up
        }

        if (rTimer.get() > 13 && rTimer.get() < 15) { // 650 - 750

          m_UpTake.set(0.0);
          m_Shooter.set(0.0);

          // Drive Forward 15 Encoder Count
          if (m_frontLeft.getPosition() < 20) {
            m_drive.arcadeDrive(-0.65, 0.0);
            System.out.println("Front Left: " + m_frontLeft.getPosition());
            System.out.println("Front Right: " + m_frontRight.getPosition());
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
          if (m_frontLeft.getPosition() < 30) {
            m_drive.arcadeDrive(0.65, 0.45);
            System.out.println("Front Left: " + m_frontLeft.getPosition());
            System.out.println("Front Right: " + m_frontRight.getPosition());
          }
        }

      }

      case "Left":
        // Left Auton
        System.out.println("Time: " + frc3535_Variables.tAuton);

        break;
      case "Center":
        // Center Auton

        break;
      case "Right":

        break;
      case "Default": // DEFAULT
      default:

        m_drive.arcadeDrive(0.0, 0.0);
        break;
    }

  }

  @Override
  public void teleopInit() {
    // TODO Auto-generated method stub
    super.teleopInit();
    m_Shooter.setMaxAcceleration(20000); // Set max acceleration to 20,000 RPM/s
    m_Shooter.setMaxJerk(31250); // Set max jerk to 31,250 RPM/s^2
    m_Shooter.setMaxSpeed(1600);
    m_Shooter.setPID(0.195, 0.010, 0.0, 0.184, 0.0); // Configure PID gains
    // Kp, Ki, Kd, Kf, b
    // Kp lower shoots lower
  }

  @Override
  public void teleopPeriodic() {
    Shuffleboard.startRecording();

    venomBrakeMode(2);

    if (bLimeLightConnected) {
      LimelightVision.setLEDMode(frc3535_Variables.ll_on);
    }

    // region JS1
    m_drive.arcadeDrive(-js1_L_Speed, js1_R_Rotate);

    if (js1.getRawButton(frc3535_Variables.btnD_X)) {
      LimelightBasic();
    }

    // region PID Shooter

    // SmartDashboard.putString("Shooter_Temperature", m_Shooter.getTemperature() +
    // " C");
    // SmartDashboard.putNumber("Shooter_Output_Current ",
    // m_Shooter.getOutputCurrent());
    // SmartDashboard.putNumber("Shooter_Output_Voltage ",
    // m_Shooter.getOutputVoltage());
    if (js1.getRawButtonPressed(frc3535_Variables.btnD_LT)) {
      m_Shooter.enable();
      // m_Shooter.setCommand(ControlMode.SpeedControl, 1500); // Spin the motor at x
      // RPM.
      m_Shooter.set(frc3535_Variables.m_Shooter_Speed_Reverse);
    }
    if (js1.getRawButtonReleased(frc3535_Variables.btnD_LT)) {
      m_Shooter.stopMotor();
    }

    if (js1.getRawButtonPressed(frc3535_Variables.btnD_RT)) {
      m_Shooter.enable();
      // m_Shooter.setCommand(ControlMode.SpeedControl, 1500); // Spin the motor at x
      // RPM.
      m_Shooter.set(frc3535_Variables.m_Shooter_Speed);
    }
    if (js1.getRawButtonReleased(frc3535_Variables.btnD_RT)) {
      m_Shooter.stopMotor();
    }
    // endegion PID Shooter

    if (js1.getRawButton(frc3535_Variables.btnD_LB)) // Down
    {
      p_Intake.set(DoubleSolenoid.Value.kForward); // Down
    } else if (js1.getRawButton(frc3535_Variables.btnD_RB)) // Up
    {
      p_Intake.set(DoubleSolenoid.Value.kReverse); // Up
    } else {
      p_Intake.set(DoubleSolenoid.Value.kOff);

    }

    if (js1.getRawButton(frc3535_Variables.btnD_A)) // Down
    {
      p_Rachet.set(DoubleSolenoid.Value.kForward); // Down
    } else if (js1.getRawButton(frc3535_Variables.btnD_B)) // Up
    {
      p_Rachet.set(DoubleSolenoid.Value.kReverse); // Up
    } else {
      p_Rachet.set(DoubleSolenoid.Value.kOff);

    }

    // p_Rachet.set(js1.getRawButton(frc3535_Variables.btnD_A));

    // endregion JS1

    // region JS2
    // region PID Shooter

    // SmartDashboard.putString("Shooter_Temperature", m_Shooter.getTemperature() +
    // " C");
    // SmartDashboard.putNumber("Shooter_Output_Current ",
    // m_Shooter.getOutputCurrent());
    // SmartDashboard.putNumber("Shooter_Output_Voltage ",
    // m_Shooter.getOutputVoltage());
    if (js2.getRawButtonPressed(frc3535_Variables.btnD_X)) {
      m_Shooter.enable();
      // m_Shooter.setCommand(ControlMode.SpeedControl, 1500); // Spin the motor at x
      // RPM.
      m_Shooter.set(frc3535_Variables.m_Shooter_Speed_Long);
    }
    if (js2.getRawButtonReleased(frc3535_Variables.btnD_X)) {
      m_Shooter.stopMotor();
    }
    // endegion PID Shooter

    // region Uptake
    if (js2.getRawButtonPressed(frc3535_Variables.btnD_RT)) {
      m_UpTake.set(0.5); // Down
    }
    if (js2.getRawButtonReleased(frc3535_Variables.btnD_RT)) {
      m_UpTake.set(0.0);
    }
    if (js2.getRawButtonPressed(frc3535_Variables.btnD_RB)) {
      m_UpTake.set(-0.5); // Up
    }
    if (js2.getRawButtonReleased(frc3535_Variables.btnD_RB)) {
      m_UpTake.set(0.0);
    }
    // endregion Uptake

    // region Intake
    if (js2.getRawButtonPressed(frc3535_Variables.btnD_LB)) {
      m_InTake.set(-0.5); // In
    }
    if (js2.getRawButtonReleased(frc3535_Variables.btnD_LB)) {
      m_InTake.set(0.0);
    }
    if (js2.getRawButtonPressed(frc3535_Variables.btnD_LT)) {
      m_InTake.set(0.5); // Out
    }
    if (js2.getRawButtonReleased(frc3535_Variables.btnD_LT)) {
      m_InTake.set(0.0);
    }
    // endregion Intake

    // region Climber - Up and Down
    if (js2.getRawButtonPressed(frc3535_Variables.btnD_Start)) {
      m_Climber01.set(frc3535_Variables.climb_up); // Climber Up
      // m_Climber02.set(frc3535_Variables.climb_up); // Climber Up
    }
    if (js2.getRawButtonReleased(frc3535_Variables.btnD_Start)) {
      m_Climber01.set(0.0);
      // m_Climber02.set(0.0);
    }

    if (js2.getRawButtonPressed(frc3535_Variables.btnD_Back)) {
      m_Climber01.set(frc3535_Variables.climb_down); // Climber Down
      // m_Climber02.set(frc3535_Variables.climb_down); // Climber Down
    }
    if (js2.getRawButtonReleased(frc3535_Variables.btnD_Back)) {
      m_Climber01.set(0.0);
      // m_Climber02.set(0.0);
    }
    // endregion Climber - Up and Down

    // endregion JS2
  }

  @Override
  public void testInit() {
    frc3535_Variables.tAuton = 0; // Timer
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    try {
      LimelightVision.setLEDMode(frc3535_Variables.ll_off);
    } catch (Exception name) {
      // System.out.println("Exception: " + name);
      // System.out.println("Error 396 - Unable to connect to the LimeLight");
    }
    frc3535_Variables.tAuton += 1; // Timer
    System.out.println("Timer: " + frc3535_Variables.tAuton + "|" + System.currentTimeMillis());
  }

  @Override
  public void disabledInit() {
    // TODO Auto-generated method stub
    super.disabledInit();
    try {
      LimelightVision.setLEDMode(frc3535_Variables.ll_off);
    } catch (Exception name) {
      // System.out.println("Exception: " + name);
      // System.out.println("Error 396 - Unable to connect to the LimeLight");
    }
  }

  @Override
  public void disabledPeriodic() {
    // System.out.println("Robot.disabledPeriodic()");
    super.disabledPeriodic();
    Shuffleboard.stopRecording();
    resetEncoders();

    AirCompressor.disable();
    try {
      LimelightVision.setLEDMode(frc3535_Variables.ll_off);
    } catch (Exception name) {
      // System.out.println("Exception: " + name);
      // System.out.println("Error 396 - Unable to connect to the LimeLight");
    }
  }
}
