// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//region WPI Library
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.DigitalInput;
//endregion WPI Library

//region CTRE
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;
//endregion CTRE

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
  private DifferentialDrive m_myRobot;
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private final XboxController js1Xbox = new XboxController(1);
  private final Joystick js1 = new Joystick(0);

  private final WPI_VictorSPX m_leftFrontMotor = new WPI_VictorSPX(7);
  private final WPI_VictorSPX m_leftRearMotor = new WPI_VictorSPX(9);
  private final WPI_VictorSPX m_rightFrontMotor = new WPI_VictorSPX(11);
  private final WPI_VictorSPX m_rightRearMotor = new WPI_VictorSPX(12);

  private final WPI_VictorSPX roller = new WPI_VictorSPX(8);

  private final DigitalInput ls_StopDriving = new DigitalInput(1);

  // region Robot
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

    m_leftFrontMotor.configFactoryDefault();
    m_leftRearMotor.configFactoryDefault();
    m_rightFrontMotor.configFactoryDefault();
    m_rightRearMotor.configFactoryDefault();
    roller.configFactoryDefault();

    m_leftFrontMotor.setInverted(false);
    m_leftRearMotor.setInverted(false);
    m_rightFrontMotor.setInverted(true);
    m_rightRearMotor.setInverted(true);
    roller.setInverted(true);

    // m_leftRearMotor.follow(m_leftFrontMotor);
    // m_rightRearMotor.follow(m_rightFrontMotor);

    m_myRobot = new DifferentialDrive(m_leftFrontMotor, m_rightFrontMotor);

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and
   * test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {

  }

  // endregion Robot

  // region Auton
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
        break;
      case kDefaultAuto:
      default:

        break;
    }
  }
  // endregion Auton

  // region Teleop
  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    // region Drive Unless Limit Switch Is Pressed - Tank
    // double leftPower = js1.getRawAxis(1);
    // double rightPower = js1.getRawAxis(3);

    // if (leftPower < 0 && rightPower < 0) {
    //   if (ls_StopDriving.get()) {
    //     m_myRobot.tankDrive(leftPower, rightPower);
    //   } else {
    //     // m_myRobot.stopMotor();
    //     m_myRobot.tankDrive(0, 0);
    //   }
    // } else {
    //   m_myRobot.tankDrive(leftPower, rightPower);
    // }
    // endregion Drive Unless Limit Switch Is Pressed - Tank

    // region Drive Unless Limit Switch Is Pressed - Arcade
    double forwardPower = js1.getRawAxis(1);
    double turningPower = js1.getRawAxis(2);

    if (forwardPower < 0) {
      if (ls_StopDriving.get()) {
        m_myRobot.stopMotor();
      } else {
        m_myRobot.arcadeDrive(forwardPower, turningPower);
      }
    } else {
      m_myRobot.arcadeDrive(forwardPower, turningPower);
    }
    // endregion Drive Unless Limit Switch Is Pressed - Arcade

  }
  // endregion Teleop

  // region Disabled
  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {

  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {

  }
  // endregion Disabled

  // region Test
  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    super.testInit();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {

    // region Logitech Controller
    // if (js1.getRawButtonPressed(1)) { // X
    // m_leftFrontMotor.set(0.4);
    // }
    // if (js1.getRawButtonReleased(1)) {
    // m_leftFrontMotor.set(0);
    // }
    // if (js1.getRawButtonPressed(2)) { // A
    // m_leftRearMotor.set(0.4);
    // }
    // if (js1.getRawButtonReleased(2)) {
    // m_leftRearMotor.set(0);
    // }
    // if (js1.getRawButtonPressed(3)) { // B
    // m_rightFrontMotor.set(0.4);
    // }
    // if (js1.getRawButtonReleased(3)) {
    // m_rightFrontMotor.set(0);
    // }
    // if (js1.getRawButtonPressed(4)) { // Y
    // m_rightRearMotor.set(0.4);
    // }
    // if (js1.getRawButtonReleased(4)) {
    // m_rightRearMotor.set(0);
    // }
    // endregion Logitech Controller

    // region XBOX Controller - Accessories
    // if (js1Xbox.getXButtonPressed()) {
    // roller.set(0.4);
    // }
    // if (js1Xbox.getXButtonReleased()) {
    // roller.set(0);
    // }
    // endregion XBOX Controller - Accessories

    // region XBOX Controller - Test Driving Motors
    // if (js1Xbox.getXButtonPressed()) {
    // m_leftFrontMotor.set(0.4);
    // }
    // if (js1Xbox.getXButtonReleased()) {
    // m_leftFrontMotor.set(0);
    // }

    // if (js1Xbox.getYButtonPressed()) {
    // m_leftRearMotor.set(0.4);
    // }
    // if (js1Xbox.getYButtonReleased()) {
    // m_leftRearMotor.set(0);
    // }

    // if (js1Xbox.getAButtonPressed()) {
    // m_rightFrontMotor.set(0.4);
    // }
    // if (js1Xbox.getAButtonReleased()) {
    // m_rightFrontMotor.set(0);
    // }

    // if (js1Xbox.getBButtonPressed()) {
    // m_rightRearMotor.set(0.4);
    // }
    // if (js1Xbox.getBButtonReleased()) {
    // m_rightRearMotor.set(0);
    // }
    // endregion XBOX Controller - Test Driving Motors

  }
  // endregion Test
}
