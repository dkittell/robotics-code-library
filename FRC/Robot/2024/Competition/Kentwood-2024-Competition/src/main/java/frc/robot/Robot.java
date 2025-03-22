// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// region Imports
// CTRE
//    If you use TalonSRX or VictorSPX you will need version 5, if do not you should be good with version 6. When in doubt you could load both.
//    Version 6 - https://maven.ctr-electronics.com/release/com/ctre/phoenix6/latest/Phoenix6-frc2024-latest.json
//    Version 5 - https://maven.ctr-electronics.com/release/com/ctre/phoenix/Phoenix5-frc2024-latest.json
// NavX Library - https://dev.studica.com/releases/2024/NavX.json
// Rev Robotics - https://software-metadata.revrobotics.com/REVLib-2024.json
// Venom - https://www.playingwithfusion.com/frc/playingwithfusion2024.json

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// endregion Imports

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private Joystick js_Driver;
  private Joystick js_Operator;

  // region Drivetrain
  double dDriverGearForward = 0.9;
  double nDriverDpadUpLastForward = 0; // Last known value for dpadUp - Driver Controller
  double nDriverDpadDownLastForward = 0; // Last known value for dpadDown - Driver Controller
  double dDriverGearStrafe = 0.9;
  double nDriverDpadUpLastStrafe = 0; // Last known value for dpadUp - Driver Controller
  double nDriverDpadDownLastStrafe = 0; // Last known value for dpadDown - Driver Controller
  double dDriverGearTurn = 0.6;

  public CANSparkMax m_frontLeft = new CANSparkMax(
    constants.can_id_frontLeft,
    com.revrobotics.CANSparkLowLevel.MotorType.kBrushless
  );
  public CANSparkMax m_frontRight = new CANSparkMax(
    constants.can_id_frontRight,
    com.revrobotics.CANSparkLowLevel.MotorType.kBrushless
  );
  public CANSparkMax m_rearLeft = new CANSparkMax(
    constants.can_id_rearLeft,
    com.revrobotics.CANSparkLowLevel.MotorType.kBrushless
  );
  public CANSparkMax m_rearRight = new CANSparkMax(
    constants.can_id_rearRight,
    com.revrobotics.CANSparkLowLevel.MotorType.kBrushless
  );

  // region Drivetrain Encoders
  static final double COUNTS_PER_MOTOR_REV = 42; // NEO Brushless 1650
  static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV / 70); // Inch to encoder count

  RelativeEncoder m_frontLeftEncoder;
  RelativeEncoder m_rearLeftEncoder;
  RelativeEncoder m_frontRightEncoder;
  RelativeEncoder m_rearRightEncoder;
  RelativeEncoder m_pivotEncoder;
  // endregion Drivetrain Encoders

  public MecanumDrive m_robotDrive;

  SlewRateLimiter driveX = new SlewRateLimiter(2);
  SlewRateLimiter driveY = new SlewRateLimiter(1);
  SlewRateLimiter driveZ = new SlewRateLimiter(1);

  // endregion Drivetrain

  // region Shooting System
  public CANSparkMax m_intake = new CANSparkMax(
    constants.can_id_intake,
    com.revrobotics.CANSparkLowLevel.MotorType.kBrushless
  );
  public CANSparkMax m_indexer = new CANSparkMax(
    constants.can_id_indexer,
    com.revrobotics.CANSparkLowLevel.MotorType.kBrushless
  );
  public CANSparkMax m_shooter1 = new CANSparkMax(
    constants.can_id_Shooter1,
    com.revrobotics.CANSparkLowLevel.MotorType.kBrushless
  );
  public CANSparkMax m_shooter2 = new CANSparkMax(
    constants.can_id_Shooter2,
    com.revrobotics.CANSparkLowLevel.MotorType.kBrushless
  );
  public CANSparkMax m_pivot = new CANSparkMax(
    constants.can_id_Pivot,
    com.revrobotics.CANSparkLowLevel.MotorType.kBrushless
  );
  // endregion Shooting System

  // public CANSparkMax m_Climber = new
  // CANSparkMax(constants.can_id_Climber,com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);

  PowerDistribution m_pdh = new PowerDistribution(
    constants.can_id_pdh,
    ModuleType.kRev
  );

  private SparkPIDController m_pivotPID;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

  // region Pneumatics

  // private final Compressor m_compressor = new Compressor(PneumaticsModuleType.REVPH);

  // PneumaticHub m_ph = new PneumaticHub(constants.can_id_ph);
  // DoubleSolenoid p_ds_Drivetrain = m_ph.makeDoubleSolenoid(
  //   constants.p_ds_Drivetrain_Forward_id,
  //   constants.p_ds_Drivetrain_Reverse_id
  // );
  // endregion Pneumatics

  // NavX Gyro
  AHRS ahrs;

  Timer rTimer = new Timer();

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */

  @Override
  public void robotInit() {
    Shuffleboard.startRecording();

    // region Initialize Auton
    m_chooser.setDefaultOption("Default Auto", "Default");
    m_chooser.addOption("SimpleDrive", "SimpleDrive");
    m_chooser.addOption("AmpScoreLeft", "AmpScoreLeft");
    m_chooser.addOption(
      "PreloadSpeaker(not leaving)",
      "PreloadNoteSpeakerNoLeave"
    );
    m_chooser.addOption("PreloadSpeaker(leaving)", "preloadSpeakerLeave");
    m_chooser.addOption("twoNoteMiddle", "twoNoteMiddle");
    SmartDashboard.putData("Auto choices", m_chooser);
    // endregion Initialize Auton

    js_Driver = new Joystick(constants.js_Driver);
    js_Operator = new Joystick(constants.js_Operator);

    // region PID Loop Initialization
    double kP = 0.35;
    double kI = 0;
    double kD = 0.05;
    double kIz = 0;
    double kFF = 0.025;
    double kMaxOutput = 1;
    double kMinOutput = -1;
    double maxRPM = 5700;

    // display PID coefficients on SmartDashboard
    // SmartDashboard.putNumber("P Gain", kP);
    // SmartDashboard.putNumber("I Gain", kI);
    // SmartDashboard.putNumber("D Gain", kD);
    // SmartDashboard.putNumber("I Zone", kIz);
    // SmartDashboard.putNumber("Feed Forward", kFF);
    // SmartDashboard.putNumber("Max Output", kMaxOutput);
    // SmartDashboard.putNumber("Min Output", kMinOutput);
    // SmartDashboard.putNumber("Set Rotations", 0);

    m_pivotPID = m_pivot.getPIDController();

    m_pivotPID.setP(kP);
    m_pivotPID.setI(kI);
    m_pivotPID.setD(kD);
    m_pivotPID.setIZone(kIz);
    m_pivotPID.setFF(kFF);
    m_pivotPID.setOutputRange(kMinOutput, kMaxOutput);
    // endregion PID Loop Initialization

    // region Drivetrain
    m_robotDrive =
      new MecanumDrive(m_frontLeft, m_rearLeft, m_frontRight, m_rearRight);

    m_robotDrive.setDeadband(0.15);

    SendableRegistry.addChild(m_robotDrive, m_frontLeft);
    SendableRegistry.addChild(m_robotDrive, m_rearLeft);
    SendableRegistry.addChild(m_robotDrive, m_frontRight);
    SendableRegistry.addChild(m_robotDrive, m_rearRight);

    // Invert the right side motors
    m_frontRight.setInverted(false);
    m_rearRight.setInverted(true);

    m_frontLeftEncoder = m_frontLeft.getEncoder();
    m_frontLeftEncoder.setPosition(0);
    m_frontRightEncoder = m_frontRight.getEncoder();
    m_frontRightEncoder.setPosition(0);
    m_rearLeftEncoder = m_rearLeft.getEncoder();
    m_rearLeftEncoder.setPosition(0);
    m_rearRightEncoder = m_rearRight.getEncoder();
    m_rearRightEncoder.setPosition(0);
    // endregion Drivetrain

    // m_Climber.setInverted(true);
    // m_Climber.setIdleMode(IdleMode.kBrake);

    // region Shooting System
    m_indexer.setInverted(true);
    m_intake.setInverted(true);
    m_pivot.setIdleMode(IdleMode.kBrake);
    m_pivot.setInverted(true);
    m_pivotEncoder = m_pivot.getEncoder();
    m_pivotEncoder.setPosition(0);
    m_shooter1.setInverted(true);
    m_shooter2.setInverted(true);
    // endregion Shooting System

    CameraServer.startAutomaticCapture(0);
    // CameraServer.startAutomaticCapture(1);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics that you want ran during disabled, autonomous, teleoperated
   * and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // region SmartDashboard
    // boolean bBattery;
    // if (m_pdh.getVoltage() >= 12) {
    //   bBattery = true;
    // } else {
    //   bBattery = false;
    // }
    // SmartDashboard.putBoolean("Battery", bBattery);

    // SmartDashboard.putBoolean("Enabled", isEnabled());
    // SmartDashboard.putNumber("Pivot", m_pivotEncoder.getPosition());
    // SmartDashboard.putNumber("Timer", rTimer.get());
    // SmartDashboard.putNumber("Voltage", m_pdh.getVoltage());
    // SmartDashboard.putNumber("m_FL Position", m_frontLeftEncoder.getPosition());
    // SmartDashboard.putNumber("m_FL Velocity", m_frontLeftEncoder.getVelocity());
    // SmartDashboard.putNumber("m_FR Position", m_frontRightEncoder.getPosition());
    // SmartDashboard.putNumber("m_FR Velocity", m_frontRightEncoder.getVelocity());
    // SmartDashboard.putNumber("m_RL Position", m_rearLeftEncoder.getPosition());
    // SmartDashboard.putNumber("m_RL Velocity", m_rearLeftEncoder.getVelocity());
    // SmartDashboard.putNumber("m_RR Position", m_rearRightEncoder.getPosition());
    // SmartDashboard.putNumber("m_RR Velocity", m_rearRightEncoder.getVelocity());

    // Get the current power % of the chassis
    SmartDashboard.putNumber("Strafe Gear", dDriverGearStrafe);
    SmartDashboard.putNumber("Forward Gear", dDriverGearForward);
    // endregion SmartDashboard
  }

  private double getAverageEncoderPosition() {
    return (
      (m_frontLeftEncoder.getPosition() + m_frontRightEncoder.getPosition()) / 2
    );
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
    // m_ph.disableCompressor();
    m_autoSelected = m_chooser.getSelected();
    System.out.println("Auto selected: " + m_autoSelected);
    rTimer.reset();
    rTimer.start();
    m_robotDrive.setSafetyEnabled(false);
    m_frontLeft.setIdleMode(IdleMode.kBrake);
    m_rearRight.setIdleMode(IdleMode.kBrake);
    m_frontRight.setIdleMode(IdleMode.kBrake);
    m_rearLeft.setIdleMode(IdleMode.kBrake);
    m_frontLeftEncoder.setPosition(0);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case "AmpScoreLeft":
        // Put custom auto code here
        if (rTimer.get() < 2 && rTimer.get() > 3.5) {
          m_robotDrive.driveCartesian(0, -0.5, 0);
        } else if (rTimer.get() > 3.5 && rTimer.get() < 4.5) {
          m_robotDrive.driveCartesian(-0.5, 0, 0);
        }
        if (rTimer.get() > 4.5 && rTimer.get() > 6.5) {
          m_shooter1.set(0.45);
          m_shooter2.set(0.45);
        }
        if (rTimer.get() > 6.5 && rTimer.get() > 7.5) {
          m_intake.set(0.25);
          m_indexer.set(-0.8);
        }
        break;
      case "PreloadNoteSpeakerNoLeave":
        if (rTimer.get() > 0.5 && rTimer.get() < 2.5) {
          m_shooter1.set(0.9);
          m_shooter2.set(-0.9);
        }
        if (rTimer.get() > 2.6 && rTimer.get() < 4.6) {
          m_intake.set(0.25);
          m_indexer.set(-0.7);
        }
        if (rTimer.get() > 4.7 && rTimer.get() < 6.7) {
          m_shooter1.set(0);
          m_shooter2.set(0);
          m_intake.set(0);
          m_indexer.set(0);
        }
        break;
      case "preloadSpeakerLeave":
        if (rTimer.get() > 0.5 && rTimer.get() < 2.5) {
          m_shooter1.set(0.9);
          m_shooter2.set(-0.9);
        }
        if (rTimer.get() > 2.6 && rTimer.get() < 4.6) {
          m_intake.set(0.25);
          m_indexer.set(-0.4);
        }
        if (rTimer.get() > 4.7 && rTimer.get() < 5.7) {
          m_shooter1.set(0);
          m_shooter2.set(0);
          m_intake.set(0);
          m_indexer.set(0);
        }
        if (rTimer.get() > 5.8 && rTimer.get() < 6.5) {
          m_robotDrive.driveCartesian(0.4, 0, 0);
        }
        if (rTimer.get() > 7.9 && rTimer.get() < 8.4) {
          m_robotDrive.driveCartesian(0, 0, 0);
        }
        break;
      case "twoNoteMiddle":
        if (rTimer.get() > 0.5 && rTimer.get() < 2.5) {
          m_shooter1.set(0.9);
          m_shooter2.set(-0.9);
        }
        if (rTimer.get() > 2.6 && rTimer.get() < 4.6) {
          m_intake.set(0.25);
          m_indexer.set(-0.4);
        }
        if (rTimer.get() > 4.7 && rTimer.get() < 5.7) {
          m_shooter1.set(0);
          m_shooter2.set(0);
          m_intake.set(0);
          m_indexer.set(0);
        }
        if (rTimer.get() > 5.7 && rTimer.get() < 8.7) {
          m_robotDrive.driveCartesian(0.4, 0, 0);
        }
        if (rTimer.get() > 8.8 && rTimer.get() > 9.8) {
          m_intake.set(0.25);
          m_indexer.set(-0.8);
        }
        if (rTimer.get() > 9.9 && rTimer.get() < 11.9) {
          m_robotDrive.driveCartesian(-0.4, 0, 0);
        }
        if (rTimer.get() > 8.6 && rTimer.get() < 10.6) {
          m_shooter1.set(0.8);
          m_shooter2.set(-0.8);
          // m_amp.set(1);
        }
        if (rTimer.get() > 10.7 && rTimer.get() > 11.7) {
          m_intake.set(0.25);
          m_indexer.set(-0.8);
        }
        break;
      case "SimpleDrive":
        // Drive forward 2 inches
        if (
          (m_frontLeftEncoder.getPosition() < (constants.COUNTS_PER_INCH * 24))
        ) {
          m_robotDrive.driveCartesian(.5, 0, 0);
        } else {
          // m_robotDrive.stopMotor();
          m_frontLeft.set(0);
          m_frontRight.set(0);
          m_rearLeft.set(0);
          m_rearRight.set(0);
        }
        break;
      case "Default":
      default:
        // Put default auto code here
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    m_frontLeft.setIdleMode(IdleMode.kCoast);
    m_rearRight.setIdleMode(IdleMode.kCoast);
    m_frontRight.setIdleMode(IdleMode.kCoast);
    m_rearLeft.setIdleMode(IdleMode.kCoast);
    m_indexer.setIdleMode(IdleMode.kCoast);
    // m_compressor.enableAnalog(60, 120);
    // m_compressor.enableDigital();
    // m_ph.enableCompressorDigital();
    // p_ds_Drivetrain.set(DoubleSolenoid.Value.kForward);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // region Driver Controller
    // Use the joystick Y axis for forward movement, X axis for lateral movement,
    // and Z axis for rotations
    // Takes the driver input for forward/backward and multplies it by the gear
    double speedY =
      -js_Driver.getRawAxis(constants.axisXB_lUpDown) * (dDriverGearForward);
    // Takes the driver input for strafing left/right and multplies it by the gear,
    // and takes a strafe multipier of 1.25
    double speedX =
      (
        js_Driver.getRawAxis(constants.axisXB_lLeftRight) * (dDriverGearStrafe)
      ) *
      1.25;
    // Takes the driver input for turning left/right and multplies it by the gear
    double speedZ =
      js_Driver.getRawAxis(constants.axisXB_rLeftRight) * (dDriverGearTurn);
    // When LB is pressed, increase the strafe gear by 0.1
    if (js_Driver.getRawButtonPressed(constants.btnXB_LB)) {
      if (dDriverGearStrafe < 1) {
        dDriverGearStrafe = dDriverGearStrafe + 0.10;
      } else {
        dDriverGearStrafe = 1;
      }
      // when LT is pressed, decrease the strafe gear by 0.1
    } else if (js_Driver.getRawButtonPressed(constants.btnXB_X)) {
      if (dDriverGearStrafe > 0.1) {
        dDriverGearStrafe = dDriverGearStrafe - 0.10;
      } else {
        dDriverGearStrafe = 0.1;
      }
    }
    // when RB is pressed, increase the forward gear by 0.1
    if (js_Driver.getRawButtonPressed(constants.btnXB_RB)) {
      if (dDriverGearForward < 1) {
        dDriverGearForward = dDriverGearForward + 0.10;
      } else {
        dDriverGearForward = 1;
      }
      // when RT is pressed, decrease the forward gear by 0.1
    } else if (js_Driver.getRawButtonPressed(constants.btnXB_B)) {
      if (dDriverGearForward > 0.1) {
        dDriverGearForward = dDriverGearForward - 0.10;
      } else {
        dDriverGearForward = 0.1;
      }
    }

    // Use SlewRateLimiters to reduce accelleration on the chassis, (.calculate)
    m_robotDrive.driveCartesian(
      driveY.calculate(speedY),
      driveX.calculate(speedX),
      driveZ.calculate(speedZ)
    );

    // // if driver pressed A, put wheels down
    // if (js_Driver.getRawButtonPressed(constants.btnXB_Y)) {
    //   p_ds_Drivetrain.set(DoubleSolenoid.Value.kForward);
    // }
    // // if driver presses Y, pull wheels up
    // if (js_Driver.getRawButtonPressed(constants.btnXB_A)) {
    //   p_ds_Drivetrain.set(DoubleSolenoid.Value.kReverse);
    // }
    // endregion Driver Controller

    // region Operator Controller

    // region intake
    if (js_Operator.getRawButton(constants.btnXB_LB)) {
      // When RB is pressed on the Operator, run intake and indexer to spit out
      m_indexer.set(0.6);
      m_intake.set(-0.5);
    } else if (js_Operator.getRawButton(constants.btnXB_RB)) {
      // When LB is pressed on the Operator, run intake and indexer to pick up
      m_intake.set(0.5);
      m_indexer.set(-0.75);
    } else {
      m_intake.set(0);
      m_indexer.set(0);
    }
    // endregion intake

    // region Shooting System
    if (js_Operator.getRawAxis(constants.axisXB_RTrigger) > 0.17) {
      // If the Right Trigger is pressed, you are shooting to speaker
      m_shooter1.set(.9);
      m_shooter2.set(-.9);
    } else if (js_Operator.getRawButton(constants.btnXB_B)) {
      // If the Left Trigger is pressed, you are shooting to AMP
      m_shooter1.set(-0.25);
      m_shooter2.set(0.25);
    } else {
      // If the A button is pressed, you are shooting to AMP
      if (js_Operator.getRawAxis(constants.axisXB_LTrigger) > 0.17) {
        m_shooter1.set(0.35);
        m_shooter2.set(-0.35);
      } else {
        m_shooter1.set(0);
        m_shooter2.set(0);
      }
    }
    // endregion Shooting System

    // region pivot

    // if (js_Operator.getRawButton(constants.btnXB_Y)) {
    // if (m_pivotEncoder.getPosition() > -0.2) {
    // m_pivot.set(-0.2);
    // } else {
    // m_pivot.set(0);
    // }
    // }
    // if (js_Operator.getRawButton(constants.btnXB_A)) {
    // if (m_pivotEncoder.getPosition() < 0) {
    // m_pivot.set(0.2);
    // } else {
    // m_pivot.set(0);
    // }
    // }

    if (js_Operator.getRawButton(constants.btnXB_Y)) {
      m_pivot.set(-0.1);
    } else if (js_Operator.getRawButton(constants.btnXB_A)) {
      m_pivot.set(0.05);
    } else {
      m_pivotPID.setReference(
        m_pivotEncoder.getPosition(),
        CANSparkMax.ControlType.kPosition
      );
    }
    // if (js_Operator.getRawButton(constants.btnXB_Y)) {
    //   if (m_pivotEncoder.getPosition() > -0.1) {
    //     m_pivot.set(-0.1);
    //   } else {
    //     m_pivotPID.setReference(
    //       m_pivotEncoder.getPosition(),
    //       CANSparkMax.ControlType.kPosition
    //     );
    //   }
    // }
    // if (js_Operator.getRawButton(constants.btnXB_A)) {
    //   if (m_pivotEncoder.getPosition() < 0.15) {
    //     m_pivot.set(0.05);
    //   } else {
    //     m_pivotPID.setReference(
    //       m_pivotEncoder.getPosition(),
    //       CANSparkMax.ControlType.kPosition
    //     );
    //   }
    // }
    // endregion pivot

    // endregion Operator Controller
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    m_frontLeft.setIdleMode(IdleMode.kCoast);
    m_rearRight.setIdleMode(IdleMode.kCoast);
    m_frontRight.setIdleMode(IdleMode.kCoast);
    m_rearLeft.setIdleMode(IdleMode.kCoast);
    m_frontLeftEncoder.setPosition(0);
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    // m_ph.enableCompressorDigital();
    m_pivotEncoder.setPosition(0);
    // m_ph.enableCompressorAnalog(60,120);
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    // double rotationsY = 0.05;
    // double rotationsA = 0;
    // double p = SmartDashboard.getNumber("P Gain", 0);
    // double i = SmartDashboard.getNumber("I Gain", 0);
    // double d = SmartDashboard.getNumber("D Gain", 0);
    // double iz = SmartDashboard.getNumber("I Zone", 0);
    // double ff = SmartDashboard.getNumber("Feed Forward", 0);
    // double max = SmartDashboard.getNumber("Max Output", 0);
    // double min = SmartDashboard.getNumber("Min Output", 0);
    // double rotations = SmartDashboard.getNumber("Set Rotations", 0);

    // // if PID coefficients on SmartDashboard have changed, write new values to
    // // controller
    // if ((p != kP)) {
    //   m_pivotPID.setP(p);
    //   kP = p;
    // }
    // if ((i != kI)) {
    //   m_pivotPID.setI(i);
    //   kI = i;
    // }
    // if ((d != kD)) {
    //   m_pivotPID.setD(d);
    //   kD = d;
    // }
    // if ((iz != kIz)) {
    //   m_pivotPID.setIZone(iz);
    //   kIz = iz;
    // }
    // if ((ff != kFF)) {
    //   m_pivotPID.setFF(ff);
    //   kFF = ff;
    // }
    // if ((max != kMaxOutput) || (min != kMinOutput)) {
    //   m_pivotPID.setOutputRange(min, max);
    //   kMinOutput = min;
    //   kMaxOutput = max;
    // }

    // if (js_Operator.getRawButtonPressed(constants.btnXB_Y)) {
    // m_pivotPID.setReference(
    // rotationsY,
    // CANSparkMax.ControlType.kPosition);
    // }
    // if (js_Operator.getRawButtonPressed(constants.btnXB_A)) {
    // m_pivotPID.setReference(
    // rotationsA,
    // CANSparkMax.ControlType.kPosition);
    // }
    // // if driver pressed A, put wheels down
    // if (js_Driver.getRawButtonPressed(constants.btnXB_Y)) {
    //   p_ds_Drivetrain.set(DoubleSolenoid.Value.kForward);
    // }
    // // if driver presses Y, pull wheels up
    // if (js_Driver.getRawButtonPressed(constants.btnXB_A)) {
    //   p_ds_Drivetrain.set(DoubleSolenoid.Value.kReverse);
    // }
    // SmartDashboard.putNumber("SetPoint", rotations);
    // SmartDashboard.putNumber("ProcessVariable", m_pivotEncoder.getPosition());
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
