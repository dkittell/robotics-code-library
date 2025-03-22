// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// Rev Robotics - https://software-metadata.revrobotics.com/REVLib-2023.json
// CTRE - https://maven.ctr-electronics.com/release/com/ctre/phoenix/Phoenix-frc2022-latest.json

import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants;

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

  private PowerDistribution m_pdp = new PowerDistribution();
  private static final String kDefaultAuto = "Do Nothing";
  private static final String kCustomAuto = "Initiation Drive";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private final PWMSparkMax brake = new PWMSparkMax(1);
  private final PWMSparkMax claw = new PWMSparkMax(0);
  private final WPI_VictorSPX armLeft = new WPI_VictorSPX(4);
  private final WPI_VictorSPX armRight = new WPI_VictorSPX(5);
  private final WPI_VictorSPX m_leftDrive = new WPI_VictorSPX(1);
  private final WPI_VictorSPX m_rightDrive = new WPI_VictorSPX(2);
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(
    m_leftDrive,
    m_rightDrive
  );
  private final XboxController m_controller = new XboxController(0);
  private final Joystick m_stick = new Joystick(1);
  private final Timer m_timer = new Timer();

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption(kDefaultAuto, kDefaultAuto);
    m_chooser.addOption(kCustomAuto, kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    CameraServer.startAutomaticCapture();
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightDrive.setInverted(true);
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
    m_timer.reset();
    m_timer.start();

    SmartDashboard.putBoolean("Operator Controller", m_stick.isConnected());

    SmartDashboard.putBoolean("Enabled", isEnabled());
    SmartDashboard.putNumber("Voltage", m_pdp.getVoltage());

    boolean bBattery;
    if (m_pdp.getVoltage() >= 12) {
      bBattery = true;
    } else {
      bBattery = false;
    }
    SmartDashboard.putBoolean("Battery", bBattery);

    SmartDashboard.putNumber("Temperature", m_pdp.getTemperature());
    SmartDashboard.putNumber("PDP 0", m_pdp.getCurrent(0));
    SmartDashboard.putNumber("PDP 1", m_pdp.getCurrent(1));
    SmartDashboard.putNumber("PDP 2", m_pdp.getCurrent(2));
    SmartDashboard.putNumber("PDP 3", m_pdp.getCurrent(3));
    SmartDashboard.putNumber("PDP 4", m_pdp.getCurrent(4));
    SmartDashboard.putNumber("PDP 5", m_pdp.getCurrent(5));
    SmartDashboard.putNumber("PDP 6", m_pdp.getCurrent(6));
    SmartDashboard.putNumber("PDP 7", m_pdp.getCurrent(7));
    SmartDashboard.putNumber("PDP 8", m_pdp.getCurrent(8));
    SmartDashboard.putNumber("PDP 9", m_pdp.getCurrent(9));
    SmartDashboard.putNumber("PDP 10", m_pdp.getCurrent(10));
    SmartDashboard.putNumber("PDP 11", m_pdp.getCurrent(11));
    SmartDashboard.putNumber("PDP 12", m_pdp.getCurrent(12));
    SmartDashboard.putNumber("PDP 13", m_pdp.getCurrent(13));
    SmartDashboard.putNumber("PDP 14", m_pdp.getCurrent(14));
    SmartDashboard.putNumber("PDP 15", m_pdp.getCurrent(15));
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
        // Drive for 2 seconds
        if (m_timer.get() < 2) {
          // Drive forwards half speed, make sure to turn input squaring off
          m_robotDrive.arcadeDrive(-0.55, 0.0, false);
          if (m_timer.get() < 0.5) {
            claw.set(-1.0);
            if (m_timer.get() < 2) {
              armLeft.set(-0.15);
              armRight.set(-0.15);
            }
          }
        } else {
          m_robotDrive.stopMotor(); // stop robot
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
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    m_robotDrive.arcadeDrive(
      -m_controller.getLeftY() * 0.925,
      -m_controller.getRightX() * 0.925,
      true
    );

    if (m_stick.isConnected()) {
      // region Operator Controller
      if (m_stick.getRawButton(4)) {
        armLeft.set(0.2);
        armRight.set(0.2);
      } else if (m_stick.getRawButton(3)) {
        armLeft.set(-0.25);
        armRight.set(-0.25);
      } else {
        armLeft.set(0);
        armRight.set(0);
      }

      if (m_stick.getRawButton(2)) {
        claw.set(1.0);
      } else if (m_stick.getRawButton(1)) {
        claw.set(-1.0);
      } else {
        claw.set(0);
      }

      if (m_stick.getRawButton(11)) {
        brake.set(1.0);
      } else if (m_stick.getRawButton(12)) {
        brake.set(-1.0);
      } else {
        brake.set(0);
      }
      // endregion Operator Controller

    } else {
      // region Driver Controller
      // region Move Arm
      // Y - Move Arm Down
      // X - Move Arm Up
      if (m_controller.getRawButton(constants.btnXB_Y)) {
        armLeft.set(0.2);
        armRight.set(0.2);
      } else if (m_controller.getRawButton(constants.btnXB_X)) {
        armLeft.set(-0.25);
        armRight.set(-0.25);
      } else {
        armLeft.set(0);
        armRight.set(0);
      }
      // endregion Move Arm

      // region Open/Close Claw
      // B - Open Clow
      // A - Close Claw
      if (m_controller.getRawButton(constants.btnXB_B)) {
        claw.set(1.0);
      } else if (m_controller.getRawButton(constants.btnXB_A)) {
        claw.set(-1.0);
      } else {
        claw.set(0);
      }
      // endregion Open/Close Claw

      // region Engage/Disengage Brake
      // LB - Disengage Brake
      // RB - Engage Brake
      if (m_controller.getRawButton(constants.btnXB_LB)) {
        brake.set(1.0);
      } else if (m_controller.getRawButton(constants.btnXB_RB)) {
        brake.set(-1.0);
      } else {
        brake.set(0);
      }
      // endregion Engage/Disengage Brake

      // endregion Driver Controller
    }
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

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
