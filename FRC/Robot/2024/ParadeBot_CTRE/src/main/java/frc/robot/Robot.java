// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// This example is based on https://github.com/CrossTheRoadElec/Phoenix5-Examples/tree/master/Java%20General/DifferentialDrive

// CTRE
//    If you use TalonSRX or VictorSPX you will need version 5, if do not you should be good with version 6. When in doubt you could load both.
//    Version 6 - https://maven.ctr-electronics.com/release/com/ctre/phoenix6/latest/Phoenix6-frc2024-latest.json
//    Version 5 - https://maven.ctr-electronics.com/release/com/ctre/phoenix/Phoenix5-frc2024-latest.json
// NavX Library - https://dev.studica.com/releases/2024/NavX.json
// Rev Robotics - https://software-metadata.revrobotics.com/REVLib-2024.json
// Venom - https://www.playingwithfusion.com/frc/playingwithfusion2024.json

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.blinkin;

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

  public static final blinkin m_blinkin = new blinkin(
      Constants.LEDConstants.ADDRESSABLE_LED);
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private DifferentialDrive m_robotDrive;
  private XboxController js_Driver;
  private static final int leftLeadDeviceID = 7;
  private static final int leftFollowDeviceID = 12;
  private static final int rightLeadDeviceID = 11;
  private static final int rightFollowDeviceID = 9;
  WPI_VictorSPX m_leftFollowMotor;
  WPI_VictorSPX m_leftLeadMotor;
  WPI_VictorSPX m_rightFollowMotor;
  WPI_VictorSPX m_rightLeadMotor;

  Timer timer = new Timer();

  double led_Color = 0.0;

  int redTimer = 1;
  int whiteTimerStart = 1;
  int whiteTimerEnd = 2;
  int blueTimerStart = 2;
  int blueTimerEnd = 3;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    m_leftFollowMotor = new WPI_VictorSPX(leftFollowDeviceID);
    m_leftLeadMotor = new WPI_VictorSPX(leftLeadDeviceID);
    m_rightFollowMotor = new WPI_VictorSPX(rightFollowDeviceID);
    m_rightLeadMotor = new WPI_VictorSPX(rightLeadDeviceID);

    // We need to invert one side of the drivetrain so that positive voltages
    /* set up followers */
    m_rightFollowMotor.follow(m_rightLeadMotor);
    m_leftFollowMotor.follow(m_leftLeadMotor);

    /* [3] flip values so robot moves forward when stick-forward/LEDs-green */
    m_rightLeadMotor.setInverted(true); // !< Update this
    m_leftLeadMotor.setInverted(false); // !< Update this
    /*
     * set the invert of the followers to match their respective master controllers
     */
    m_rightFollowMotor.setInverted(InvertType.FollowMaster);
    m_leftFollowMotor.setInverted(InvertType.FollowMaster);

    m_robotDrive = new DifferentialDrive(m_leftLeadMotor, m_rightLeadMotor);

    js_Driver = new XboxController(0);
    timer.reset();
    timer.start();
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
    if (timer.get() < redTimer) {
      led_Color = Constants.LEDConstants.RED;
      m_blinkin.set(led_Color);
      SmartDashboard.putNumber("LED Color", led_Color);
    } else if (timer.get() > whiteTimerStart && timer.get() < whiteTimerEnd) {
      led_Color = Constants.LEDConstants.WHITE;
      m_blinkin.set(led_Color);
      SmartDashboard.putNumber("LED Color", led_Color);
    } else if (timer.get() > blueTimerStart && timer.get() < blueTimerEnd) {
      led_Color = Constants.LEDConstants.DARK_BLUE;
      m_blinkin.set(led_Color);
      SmartDashboard.putNumber("LED Color", led_Color);
    } else {
      led_Color = Constants.LEDConstants.RED;
      m_blinkin.set(led_Color);
      SmartDashboard.putNumber("LED Color", led_Color);
      timer.reset();
    }
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
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here

        // Drive forward for 4 seconds
        if (timer.get() < 4) {
          m_robotDrive.arcadeDrive(0.5, 0);
        } else {
          m_leftLeadMotor.set(0);
          m_rightLeadMotor.set(0);
        }

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

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    /* get gamepad stick values */
    double forw = -1 * js_Driver.getRawAxis(1);/* positive is forward */
    double turn = -1 * js_Driver.getRawAxis(4);/* positive is right */

    /* deadband gamepad 10% */
    if (Math.abs(forw) < 0.10) {
      forw = 0;
    }
    if (Math.abs(turn) < 0.10) {
      turn = 0;
    }

    /* drive robot */
    m_robotDrive.arcadeDrive(forw, turn);

  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    timer.reset();
    timer.start();
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    if (timer.get() < redTimer) {
      led_Color = Constants.LEDConstants.RED;
      m_blinkin.set(led_Color);
      SmartDashboard.putNumber("LED Color", led_Color);
    } else if (timer.get() > whiteTimerStart && timer.get() < whiteTimerEnd) {
      led_Color = Constants.LEDConstants.WHITE;
      m_blinkin.set(led_Color);
      SmartDashboard.putNumber("LED Color", led_Color);
    } else if (timer.get() > blueTimerStart && timer.get() < blueTimerEnd) {
      led_Color = Constants.LEDConstants.DARK_BLUE;
      m_blinkin.set(led_Color);
      SmartDashboard.putNumber("LED Color", led_Color);
    } else {
      led_Color = Constants.LEDConstants.RED;
      m_blinkin.set(led_Color);
      SmartDashboard.putNumber("LED Color", led_Color);
      timer.reset();
    }
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {

    timer.reset();
    timer.start();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    // //region Test Wheel Direction
    // if (js_Driver.getRawButton(1)) {
    // // XBox A Button
    // m_leftLeadMotor.set(0.5);
    // } else {
    // m_leftLeadMotor.set(0);
    // }
    // if (js_Driver.getRawButton(2)) {
    // // XBox B Button
    // m_leftFollowMotor.set(0.5);
    // } else {
    // m_leftFollowMotor.set(0);
    // }
    // if (js_Driver.getRawButton(3)) {
    // // Xbox X Button
    // m_rightLeadMotor.set(0.5);
    // } else {
    // m_rightLeadMotor.set(0);
    // }
    // if (js_Driver.getRawButton(4)) {
    // // XBox Y Button
    // m_rightFollowMotor.set(0.5);
    // } else {
    // m_rightFollowMotor.set(0);
    // }
    // //endregion Test Wheel Direction

    // region Test LED

    if (js_Driver.getRawButton(2)) {
      // XBox B Button
      // Logitech A Button
      // m_blinkin.set(Constants.LEDConstants.STROBE_RED); // Blinking Red

      led_Color = Constants.LEDConstants.DARK_RED;

      m_blinkin.set(led_Color); // Dark Red

      SmartDashboard.putNumber("LED Color", led_Color);

    } else if (js_Driver.getRawButton(1)) {
      // XBox A Buton
      led_Color = Constants.LEDConstants.WHITE;
      m_blinkin.set(led_Color);
      SmartDashboard.putNumber("LED Color", led_Color);
    } else if (js_Driver.getRawButton(3)) {
      // XBox X Buton
      led_Color = Constants.LEDConstants.DARK_BLUE;
      m_blinkin.set(led_Color);
      SmartDashboard.putNumber("LED Color", led_Color);
    } else if (js_Driver.getRawButton(4)) {
      // XBox Y Buton

      if (timer.get() < 15) {
        led_Color = Constants.LEDConstants.RED;
        m_blinkin.set(led_Color);
        SmartDashboard.putNumber("LED Color", led_Color);
      } else if (timer.get() > 15 && timer.get() < 30) {
        led_Color = Constants.LEDConstants.WHITE;
        m_blinkin.set(led_Color);
        SmartDashboard.putNumber("LED Color", led_Color);
      } else if (timer.get() > 30 && timer.get() < 45) {
        led_Color = Constants.LEDConstants.DARK_BLUE;
        m_blinkin.set(led_Color);
        SmartDashboard.putNumber("LED Color", led_Color);
      } else {
        led_Color = Constants.LEDConstants.RED;
        m_blinkin.set(led_Color);
        SmartDashboard.putNumber("LED Color", led_Color);
        timer.reset();
      }

    } else {
      led_Color = Constants.LEDConstants.DARK_BLUE;
      m_blinkin.set(led_Color);
      SmartDashboard.putNumber("LED Color", led_Color);
      // m_blinkin.allianceColor();

    }
    // endregion Test LED
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
