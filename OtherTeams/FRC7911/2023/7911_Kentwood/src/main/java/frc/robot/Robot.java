// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.MecanumDrive;

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

  private final DoubleSolenoid p_Boom = new DoubleSolenoid(
      PneumaticsModuleType.CTREPCM,
      0,
      1);

  private final DoubleSolenoid p_Arm = new DoubleSolenoid(
      PneumaticsModuleType.CTREPCM,
      2,
      3);

  private final DoubleSolenoid p_Hand = new DoubleSolenoid(
      PneumaticsModuleType.CTREPCM,
      4,
      5);

  private MecanumDrive m_robotDrive;
  XboxController js_Driver = new XboxController(0);
  XboxController js_Operator = new XboxController(1);

  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

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

    WPI_VictorSPX frontLeft = new WPI_VictorSPX(5);
    WPI_VictorSPX rearLeft = new WPI_VictorSPX(7);
    WPI_VictorSPX frontRight = new WPI_VictorSPX(8);
    WPI_VictorSPX rearRight = new WPI_VictorSPX(6);

    // Invert the right side motors.
    // You may need to change or remove this to match your robot.
    frontRight.setInverted(true);
    rearRight.setInverted(true);

    m_robotDrive = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);

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
    // Use the joystick Y axis for forward movement, X axis for lateral
    // movement, and Z axis for rotation.
    m_robotDrive.driveCartesian(
        js_Driver.getRawAxis(1),
        -js_Driver.getRawAxis(0),
        -js_Driver.getRawAxis(4));

    if (js_Operator.getRawButton(3)) {
      // XBox X Button
      p_Boom.set(DoubleSolenoid.Value.kForward);
    } else if (js_Operator.getRawButton(1)) {
      // XBox A Button
      p_Boom.set(DoubleSolenoid.Value.kReverse);
    }

    if (js_Operator.getRawButton(4)) {
      // XBox Y Button
      p_Arm.set(DoubleSolenoid.Value.kForward);
    } else if (js_Operator.getRawButton(2)) {
      // XBox B
      p_Arm.set(DoubleSolenoid.Value.kReverse);
    }

    if (js_Operator.getRawButton(5)) {
      // XBox LB Button
      p_Hand.set(DoubleSolenoid.Value.kForward);
    } else if (js_Operator.getRawButton(6)) {
      // XBox RB Button

      p_Hand.set(DoubleSolenoid.Value.kReverse);
    }

  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
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
