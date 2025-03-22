// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Vendor Libraries: 
// CTRE - https://maven.ctr-electronics.com/release/com/ctre/phoenix/Phoenix-frc2022-latest.json

package frc.robot;

import javax.lang.model.util.ElementScanner6;

// region Motor Control
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
// endregion Motor Control

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;

/**
 * This is a demo program showing the use of the DifferentialDrive class,
 * specifically it contains
 * the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {

  public static Timer rTimer = new Timer();

  PowerDistribution m_pdp = new PowerDistribution();

  // region Controller Assignments
  // Joystick js1;
  // Joystick js2;
  XboxController js1;
  XboxController js2;
  // endregion Controller Assignments

  // region Motor Assignments
  // https://docs.wpilib.org/en/latest/docs/software/hardware-apis/motors/wpi-drive-classes.html#multi-motor-differentialdrive-with-motorcontrollergroups
  WPI_VictorSPX m_LeftMaster = new WPI_VictorSPX(6);
  WPI_VictorSPX m_LeftSlave = new WPI_VictorSPX(7);
  MotorControllerGroup m_Left = new MotorControllerGroup(m_LeftMaster, m_LeftSlave);

  WPI_VictorSPX m_RightMaster = new WPI_VictorSPX(4);
  WPI_VictorSPX m_RightSlave = new WPI_VictorSPX(5);
  MotorControllerGroup m_Right = new MotorControllerGroup(m_RightMaster, m_RightSlave);

  WPI_VictorSPX m_RightAddison = new WPI_VictorSPX(0);
  WPI_VictorSPX m_LeftAddison = new WPI_VictorSPX(1);
  WPI_VictorSPX m_Kicker = new WPI_VictorSPX(3);
  WPI_VictorSPX m_Spinner = new WPI_VictorSPX(2);

  private DifferentialDrive m_Drive = new DifferentialDrive(m_Left, m_Right);
  // endregion Motor Assignments

  // region XBox Controller Layout
  public static final int btnXB_X = 3;
  public static final int btnXB_A = 1;
  public static final int btnXB_B = 2;
  public static final int btnXB_Y = 4;
  public static final int btnXB_LB = 5;
  public static final int btnXB_RB = 6;
  public static final int btnXB_Back = 7;
  public static final int btnXB_Start = 8;
  public static final int btnXB_LToggle = 9;
  public static final int btnXB_RToggle = 10;

  public static final int axisXB_lLeftRight = 0;
  public static final int axisXB_lUpDown = 1;
  public static final int axisXB_LTrigger = 2;
  public static final int axisXB_RTrigger = 3;
  public static final int axisXB_rLeftRight = 4;
  public static final int axisXB_rUpDown = 5;
  // endregion XBox Controller Layout

  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_LeftMaster.setInverted(true);
    m_LeftSlave.setInverted(true);
    m_RightMaster.setInverted(false);
    m_RightSlave.setInverted(false);
    m_RightAddison.setInverted(false);
    m_LeftAddison.setInverted(false);
    m_Kicker.setInverted(false);

    m_Drive = new DifferentialDrive(m_LeftMaster, m_RightMaster);
    // js1 = new Joystick(0);
    // js2 = new Joystick(1);
    js1 = new XboxController(0);
    js2 = new XboxController(1);

    // region Camera Test
    CameraServer camera01 = CameraServer.getInstance();
    camera01.startAutomaticCapture(0);
    // endregion Camera Test

  }

  @Override
  public void robotPeriodic() {

  }

  @Override
  public void autonomousInit() {
    rTimer.reset();
    rTimer.start();
  }

  @Override
  public void autonomousPeriodic() {

    do {
      m_Drive.tankDrive(0.6, 0.6);
    } while (rTimer.get() < 10);
    m_Drive.tankDrive(0.0, 0.0);
    rTimer.stop();
    rTimer.reset();

    rTimer.start();
    do {
      m_Drive.tankDrive(0.6, -0.6);
    } while (rTimer.get() < 10);
    m_Drive.tankDrive(0.0, 0.0);
    rTimer.stop();
    rTimer.reset();

    rTimer.start();
    do {
      m_Drive.tankDrive(0.6, 0.6);
    } while (rTimer.get() < 10);
    m_Drive.tankDrive(0.0, 0.0);
    rTimer.stop();
    rTimer.reset();

    m_Kicker.set(0.4);

    // region Test Motor Direction
    // This area of code should only be used to make sure the motors are all going
    // the correct direction
    // Plug only one motor in at a time and then run auton mode to verify direction.
    // m_LeftMaster.set(0.5);
    // m_LeftSlave.set(0.5);
    // m_frontRight.set(0.5);
    // m_rearRight.set(0.5);
    // endregion Test Motor Direction

    // region Simple Auton Drive Example
    // Only uncomment and run after testing direction of wheels
    // Once you feel good about where the robot is going you can chagne the speed
    // m_Drive.tankDrive(0.4, 0.4);

    // endregion Simple Auton Drive Example

  }

  @Override
  public void teleopInit() {

  }

  @Override
  public void teleopPeriodic() {
    // region Drive Robot - Tank Drive
    m_Drive.tankDrive(-js1.getLeftY(), -js1.getRightY());
    // endregion Drive Robot - Tank Drive

    /*
     * Operator Functions:
     * RA-LB/RB
     * LA-
     * Climber -
     * Boot -
     */

    // region Joystick Commands For Arm
    // region Extend Arm
    // m_ExtendArm.set(js2.getRawAxis(axisXB_RTrigger));
    if (js2.getRawButtonPressed(btnXB_LB)) {
      m_RightAddison.set(0.4);
      m_LeftAddison.set(0.4);
    }
    if (js2.getRawButtonReleased(btnXB_LB)) {
      m_RightAddison.set(0.0);
      m_LeftAddison.set(0.0);
    }
    // endregion Extend Arm

    // region Bring Arm In
    // m_ExtendArm.set(-js2.getRawAxis(axisXB_LTrigger));
    if (js2.getRawButtonPressed(btnXB_RB)) {
      m_RightAddison.set(-0.4);
      m_LeftAddison.set(-0.4);
    }
    if (js2.getRawButtonReleased(btnXB_RB)) {
      m_RightAddison.set(0.0);
      m_LeftAddison.set(0.0);
    }

    // endregion Bring Arm In

    // Region kicker out
    if (js2.getRawButtonPressed(btnXB_X)) {
      m_Kicker.set(0.4);
    }
    if (js2.getRawButtonReleased(btnXB_X)) {
      m_Kicker.set(0.0);
    }
    if (js2.getRawButtonPressed(btnXB_Y)) {
      m_Kicker.set(-0.4);
    }
    if (js2.getRawButtonReleased(btnXB_Y)) {
      m_Kicker.set(0.0);
    }
    // EndRegion kicker out

    // Region kicker out
    if (js2.getRawButtonPressed(btnXB_A)) {
      m_Spinner.set(0.4);
    }
    if (js2.getRawButtonReleased(btnXB_A)) {
      m_Spinner.set(0.0);
    }
    if (js2.getRawButtonPressed(btnXB_B)) {
      m_Spinner.set(-0.4);
    }
    if (js2.getRawButtonReleased(btnXB_B)) {
      m_Spinner.set(0.0);
    }
    // EndRegion kicker out

  }

}
