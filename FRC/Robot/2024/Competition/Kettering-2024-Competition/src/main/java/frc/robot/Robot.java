// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//region Imports
// CTRE
//    If you use TalonSRX or VictorSPX you will need version 5, if do not you should be good with version 6. When in doubt you could load both.
//    Version 6 - https://maven.ctr-electronics.com/release/com/ctre/phoenix6/latest/Phoenix6-frc2024-latest.json
//    Version 5 - https://maven.ctr-electronics.com/release/com/ctre/phoenix/Phoenix5-frc2024-latest.json
// NavX Library - https://dev.studica.com/releases/2024/NavX.json
// Rev Robotics - https://software-metadata.revrobotics.com/REVLib-2024.json
// Venom - https://www.playingwithfusion.com/frc/playingwithfusion2024.json

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//endregion Imports

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private Joystick js_Driver;
  private Joystick js_Operator;

  DigitalInput ls_climber;

  //region Drivetrain
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

  private RelativeEncoder m_frontLeftEncoder;
  private RelativeEncoder m_rearLeftEncoder;
  private RelativeEncoder m_frontRightEncoder;
  private RelativeEncoder m_rearRightEncoder;

  //endregion Drivetrain

  //region Shooting System
  public CANSparkMax m_intake = new CANSparkMax(
    constants.can_id_intake,
    com.revrobotics.CANSparkLowLevel.MotorType.kBrushed
  );
  public CANSparkMax m_conveyer = new CANSparkMax(
    constants.can_id_Conveyor,
    com.revrobotics.CANSparkLowLevel.MotorType.kBrushless
  );
  public CANSparkMax m_shooter1 = new CANSparkMax(
    constants.can_id_Shooter1,
    com.revrobotics.CANSparkLowLevel.MotorType.kBrushed
  );
  public CANSparkMax m_shooter2 = new CANSparkMax(
    constants.can_id_Shooter2,
    com.revrobotics.CANSparkLowLevel.MotorType.kBrushed
  );
  public CANSparkMax m_Climber = new CANSparkMax(
    constants.can_id_Climber,
    com.revrobotics.CANSparkLowLevel.MotorType.kBrushless
  );
  public CANSparkMax m_amp = new CANSparkMax(
    constants.can_id_Amp,
    com.revrobotics.CANSparkLowLevel.MotorType.kBrushed
  );
  //endregion Shooting System

  public MecanumDrive m_robotDrive = new MecanumDrive(
    m_frontLeft,
    m_rearLeft,
    m_frontRight,
    m_rearRight
  );

  SlewRateLimiter driveX = new SlewRateLimiter(1);
  SlewRateLimiter driveY = new SlewRateLimiter(1);
  SlewRateLimiter driveZ = new SlewRateLimiter(1);

  PneumaticHub m_ph = new PneumaticHub(constants.can_id_ph);
  DoubleSolenoid p_ds_Drivetrain = m_ph.makeDoubleSolenoid(
    constants.p_ds_Drivetrain_Forward_id,
    constants.p_ds_Drivetrain_Reverse_id
  );

  // NavX Gyro
  AHRS ahrs;

  double dDriverGearForward = 0.6;
  double nDriverDpadUpLastForward = 0; // Last known value for dpadUp - Driver Controller
  double nDriverDpadDownLastForward = 0; // Last known value for dpadDown - Driver Controller
  double dDriverGearStrafe = 0.6;
  double nDriverDpadUpLastStrafe = 0; // Last known value for dpadUp - Driver Controller
  double nDriverDpadDownLastStrafe = 0; // Last known value for dpadDown - Driver Controller
  double dDriverGearTurn = 0.6;

  Timer timer = new Timer();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", "Default");
    m_chooser.addOption("Simple Drive", "SimpleDrive");
    m_chooser.addOption("AmpScoreLeft", "AmpScoreLeft");
    m_chooser.addOption("PreloadSpeaker", "PreloadNoteSpeaker");
    m_chooser.addOption("twoNoteMiddle", "twoNoteMiddle");
    SmartDashboard.putData("Auto choices", m_chooser);

    ls_climber = new DigitalInput(2);
    js_Driver = new Joystick(0);
    js_Operator = new Joystick(1);
    timer = new Timer();

    Shuffleboard.startRecording();

    //region Drivetrain
    m_frontLeft.setInverted(true);
    m_rearRight.setInverted(true);
    //endregion Drivetrain

    //region Shooting System
    m_conveyer.setInverted(true);
    m_intake.setInverted(true);
    m_shooter1.setInverted(true);
    m_shooter2.setInverted(true);
    m_Climber.setInverted(true);
    m_Climber.setIdleMode(IdleMode.kBrake);

    // endregion Shooting System

    CameraServer.startAutomaticCapture(0);
    CameraServer.startAutomaticCapture(1);

    m_frontLeftEncoder = m_frontLeft.getEncoder();
    m_frontRightEncoder = m_frontLeft.getEncoder();
    m_rearLeftEncoder = m_frontLeft.getEncoder();
    m_rearRightEncoder = m_frontLeft.getEncoder();

    m_frontLeftEncoder.setPosition(0);
    m_frontRightEncoder.setPosition(0);
    m_rearLeftEncoder.setPosition(0);
    m_rearRightEncoder.setPosition(0);
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
    SmartDashboard.putBoolean("ls", ls_climber.get());
    SmartDashboard.putNumber("Forward Gear", dDriverGearForward);
    SmartDashboard.putNumber("Strafe Gear", dDriverGearStrafe);
    SmartDashboard.putNumber("Turn Gear", dDriverGearTurn);

    /**
     * Encoder position is read from a RelativeEncoder object by calling the
     * GetPosition() method.
     *
     * GetPosition() returns the position of the encoder in units of revolutions
     */
    SmartDashboard.putNumber(
      "Encoder Position",
      m_frontLeftEncoder.getPosition()
    );

    /**
     * Encoder velocity is read from a RelativeEncoder object by calling the
     * GetVelocity() method.
     *
     * GetVelocity() returns the velocity of the encoder in units of RPM
     */
    SmartDashboard.putNumber(
      "Encoder Velocity",
      m_frontLeftEncoder.getVelocity()
    );
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
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
    timer.start();
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
        if (timer.get() < 2 && timer.get() > 3.5) {
          m_robotDrive.driveCartesian(0, -0.5, 0);
        } else if (timer.get() > 3.5 && timer.get() < 4.5) {
          m_robotDrive.driveCartesian(-0.5, 0, 0);
        }
        if (timer.get() > 4.5 && timer.get() > 6.5) {
          m_shooter1.set(0.45);
          m_shooter2.set(0.45);
          m_amp.set(0.3);
        }
        if (timer.get() > 6.5 && timer.get() > 7.5) {
          m_intake.set(0.8);
          m_conveyer.set(0.8);
        }
        break;
      case "PreloadNoteSpeaker":
        if (timer.get() > 1 && timer.get() < 2) {
          m_Climber.set(-0.7);
        } else {
          m_Climber.set(0);
        }
        if (timer.get() > 2.1 && timer.get() > 2.3) {
          m_robotDrive.driveCartesian(0.4, 0, 0);
        } else {
          m_robotDrive.driveCartesian(0, 0, 0);
        }
        if (timer.get() > 2.4 && timer.get() < 3.8) {
          m_shooter1.set(1);
          m_shooter2.set(1);
        }
        if (timer.get() > 3.9 && timer.get() < 5.5) {
          m_intake.set(-0.8);
          m_conveyer.set(-0.8);
        }

        break;
      case "twoNoteMiddle":
        if (timer.get() > 1 && timer.get() < 2) {
          m_Climber.set(0.7);
        } else {
          m_Climber.set(0);
        }
        if (timer.get() > 2.1 && timer.get() < 2.3) {
          m_robotDrive.driveCartesian(0.4, 0, 0);
        } else {
          m_robotDrive.driveCartesian(0, 0, 0);
        }
        if (timer.get() > 2.4 && timer.get() < 3.8) {
          m_shooter1.set(1);
          m_shooter2.set(1);
        }
        if (timer.get() > 3.9 && timer.get() < 5.5) {
          m_intake.set(-0.8);
          m_conveyer.set(-0.8);
        }
        if (timer.get() > 5.6 && timer.get() < 5.8) {
          m_robotDrive.driveCartesian(0.4, 0, 0);
        }
        if (timer.get() > 5.9 && timer.get() > 7.9) {
          m_intake.set(-0.8);
          m_conveyer.set(-0.8);
        }
        if (timer.get() > 8 && timer.get() < 8.5) {
          m_robotDrive.driveCartesian(0.0, 0.4, 0);
        }
        if (timer.get() > 8.6 && timer.get() < 10.6) {
          m_shooter1.set(1);
          m_shooter2.set(1);
          // m_amp.set(1);
        }

        break;
      case "Simple Drive":
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
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    //region Driver Controller
    // Use the joystick Y axis for forward movement, X axis for lateral movement, and Z axis for rotations
    // Takes the driver input for forward/backward and multplies it by the gear
    double speedY =
      -js_Driver.getRawAxis(constants.axisD_lUpDown) * (dDriverGearForward);
    // Takes the driver input for strafing left/right and multplies it by the gear, and takes a strafe multipier of 1.25
    double speedX =
      (js_Driver.getRawAxis(constants.axisD_lLeftRight) * (dDriverGearStrafe)) *
      1.25;
    // Takes the driver input for turning left/right and multplies it by the gear
    double speedZ =
      js_Driver.getRawAxis(constants.axisD_rLeftRight) * (dDriverGearTurn);

    // When LB is pressed, increase the strafe gear by 0.1
    if (js_Driver.getRawButtonPressed(constants.btnD_LB)) {
      if (dDriverGearStrafe < 1) {
        dDriverGearStrafe = dDriverGearStrafe + 0.10;
      } else {
        dDriverGearStrafe = 1;
      }
      // when LT is pressed, decrease the strafe gear by 0.1
    } else if (js_Driver.getRawButtonPressed(constants.btnD_LTrigger)) {
      if (dDriverGearStrafe > 0.1) {
        dDriverGearStrafe = dDriverGearStrafe - 0.10;
      } else {
        dDriverGearStrafe = 0.1;
      }
    }
    // when RB is pressed, increase the forward gear by 0.1
    if (js_Driver.getRawButtonPressed(constants.btnD_RB)) {
      if (dDriverGearForward < 1) {
        dDriverGearForward = dDriverGearForward + 0.10;
      } else {
        dDriverGearForward = 1;
      }
      // when RT is pressed, decrease the forward gear by 0.1
    } else if (js_Driver.getRawButtonPressed(constants.btnD_RTrigger)) {
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

    // if driver pressed A, put wheels down
    if (js_Driver.getRawButtonPressed(constants.btnD_A)) {
      p_ds_Drivetrain.set(DoubleSolenoid.Value.kForward);
    }
    // if driver presses Y, pull wheels up
    if (js_Driver.getRawButtonPressed(constants.btnD_Y)) {
      p_ds_Drivetrain.set(DoubleSolenoid.Value.kReverse);
    }
    //endregion Driver Controller

    //region Operator Controller
    // When RB is pressed on the Operator, run intake and conveyer to spit out
    if (js_Operator.getRawButton(constants.btnXB_RB)) {
      m_conveyer.set(0.8);
      m_intake.set(0.8);
      // When LB is pressed on the Operator, run intake and conveyer to pick up
    } else if (js_Operator.getRawButton(constants.btnXB_LB)) {
      m_intake.set(-0.8);
      m_conveyer.set(-0.8);
    } else {
      m_intake.set(0);
      m_conveyer.set(0);
    }

    // region Shooting System

    //region FOR SPEAKER
    // If the Right Trigger is pressed, you are shooting to speaker
    if (js_Operator.getRawAxis(constants.axisXB_RTrigger) > 0.17) {
      m_shooter1.set(1);
      m_shooter2.set(1);
      // m_amp.set(0.7);
      // If the Left Trigger is pressed, you are intaking to speaker
    } else if (js_Operator.getRawAxis(constants.axisXB_LTrigger) > 0.17) {
      m_shooter1.set(-0.5);
      m_shooter2.set(-0.5);
      // m_amp.set(-0.6);
    }
    //endregion FOR SPEAKER
    else {
      //region FOR AMP
      // If the A button is pressed, you are shooting to AMP
      if (js_Operator.getRawButton(constants.btnXB_A)) {
        m_shooter1.set(0.45);
        m_shooter2.set(0.45);
        m_amp.set(0.5);
      } else {
        m_shooter1.set(0);
        m_shooter2.set(0);
        m_amp.set(0);
      }
      //endregion FOR AMP
    }
    // endregion Shooting System

    //region Climbing System
    //region Climber Without Limit Switch
    // If you press up on the left stick (Operator) then run climber up
    // if (js_Operator.getRawAxis(constants.axisXB_lUpDown) > 0.17) {
    //   m_Climber.set(0.5);
    // } else if (js_Operator.getRawAxis(constants.axisXB_lUpDown) < -0.17) {
    //   m_Climber.set(-0.5);
    // } else {
    //   m_Climber.set(0);
    // }
    //endregion Climber Without Limit Switch

    //region Limit Switch Climber
    // If you press up on the left stick (Operator) then run climber up
    if (js_Operator.getRawAxis(constants.axisXB_lUpDown) > 0.17) {
      // if the limit switch is pressed, do NOT allow the motor to run down (set 0),
      // else allow the motor to go
      if (ls_climber.get()) {
        m_Climber.set(0);
      } else {
        m_Climber.set(js_Operator.getRawAxis(constants.axisXB_lUpDown));
      }
      // If you pressed down on the left stick (Operator), run climber down
    } else if (js_Operator.getRawAxis(constants.axisXB_lUpDown) < -0.17) {
      m_Climber.set(js_Operator.getRawAxis(constants.axisXB_lUpDown));
    } else {
      m_Climber.set(0);
    }
    //endregion Limit Switch Climber

    //endregion Climbing System
    //endregion Operator Controller

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
