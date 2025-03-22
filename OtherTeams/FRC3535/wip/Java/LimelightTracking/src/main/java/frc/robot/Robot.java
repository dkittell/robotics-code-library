/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

//region Imports
import edu.wpi.first.wpilibj.Joystick;
// import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.MotorSafety;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.Timer;

import frc.robot.subsystems.*;
import frc.robot.constants.*;

//region Venom Code Imports
import com.playingwithfusion.CANVenom;
import com.playingwithfusion.CANVenom.ControlMode;
//endregion Venom Code Imports
//endregion Imports

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private String c_SelectedAuton;
  private String c_SelectedDriveMode;
  private final SendableChooser<String> c_Auton = new SendableChooser<>();
  private final SendableChooser<String> c_Auton_Delay = new SendableChooser<>();
  private final SendableChooser<String> c_DriveMode = new SendableChooser<>();

  // region Venom
  public com.playingwithfusion.CANVenom m_frontLeft = new CANVenom(2);
  public com.playingwithfusion.CANVenom m_rearLeft = new CANVenom(1);
  public com.playingwithfusion.CANVenom m_frontRight = new CANVenom(4);
  public com.playingwithfusion.CANVenom m_rearRight = new CANVenom(3);
  public com.playingwithfusion.CANVenom m_Shooter = new CANVenom(5);

  // SpeedControllerGroup m_left = new SpeedControllerGroup(m_frontLeft,
  // m_rearLeft);
  // SpeedControllerGroup m_right = new SpeedControllerGroup(m_frontRight,
  // m_rearRight);
  // DifferentialDrive m_drive = new DifferentialDrive(m_left, m_right);
  DifferentialDrive m_drive = new DifferentialDrive(m_frontLeft, m_frontRight);
  // endregion Venom

  // region Controller Definition
  private final Joystick js1 = new Joystick(0);
  private final Joystick js2 = new Joystick(1);

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

  // JS1 Left Deadband
  double js1_L_Deadband_Low = 0.15;
  double js1_L_Deadband_High = 0.90;
  double js1_L_Deadband_Max = 1.00;
  // JS2 Left Deadband
  double js2_L_Deadband_Low = 0.15;
  double js2_L_Deadband_High = 0.90;
  double js2_L_Deadband_Max = 1.00;

  // JS1 Right Deadband
  double js1_R_Deadband_Low = 0.15;
  double js1_R_Deadband_High = 0.60;
  double js1_R_Deadband_Max = 1.00;
  // JS2 Right Deadband
  double js2_R_Deadband_Low = 0.15;
  double js2_R_Deadband_High = 0.50;
  double js2_R_Deadband_Max = 1.00;
  // endregion Dead Band Variables

  // region Controller Button Variables
  // region PXN
  int btnPXND_X = 3;
  int btnPXND_A = 1;
  int btnPXND_B = 2;
  int btnPXND_Y = 4;
  int btnPXND_LB = 5;
  int btnPXND_RB = 6;
  int btnPXND_L3 = 9;
  int btnPXND_R3 = 10;
  int btnPXND_Share = 7;
  int btnPXND_Options = 8;
  // endregion PXN

  // region Logitech Gamepad F310 Controller Layout - D Switch
  int btnD_X = 1;
  int btnD_A = 2;
  int btnD_B = 3;
  int btnD_Y = 4;
  int btnD_LB = 5;
  int btnD_RB = 6;
  int btnD_LT = 7;
  int btnD_RT = 8;
  int btnD_Back = 9;
  int btnD_Start = 10;
  int btnD_LToggle = 11;
  int btnD_RToggle = 12;
  int axisD_lUpDown = 1;
  int axisD_lLeftRight = 0;
  int axisD_rUpDown = 3;
  int axisD_rLeftRight = 2;
  // endregion Logitech Gamepad F310 Controller Layout - D Switch

  // region Logitech Gamepad F310 Controller Layout - X Switch
  int btnX_X = 3;
  int btnX_A = 1;
  int btnX_B = 2;
  int btnX_Y = 4;
  int btnX_LB = 5;
  int btnX_RB = 6;
  int btnX_Back = 7;
  int btnX_Start = 8;
  int btnX_LToggle = 9;
  int btnX_RToggle = 10;
  int axisX_lUpDown = 1;
  int axisX_lLeftRight = 0;
  int axisX_rUpDown = 5;
  int axisX_rLeftRight = 4;
  // endregion Logitech Gamepad F310 Controller Layout - X Switch

  // region Logitech Attack 3 J - UJ18
  int btnA_Trigger = 1;
  int btnA_2 = 2;
  int btnA_3 = 3;
  int btnA_4 = 4;
  int btnA_5 = 5;
  int btnA_6 = 6;
  int btnA_7 = 7;
  int btnA_8 = 8;
  int btnA_9 = 9;
  int btnA_10 = 10;
  int btnA_11 = 11;
  int axisX_UpDown = 0;
  int axisX_LeftRight = 1;
  // endregion Logitech Attack 3 J - UJ18
  // endregion Controller Button Variables

  // endregion Controller Definition

  boolean bLimeLightConnected = true;
  double left_command = 0.0;
  double right_command = 0.0;

  double dTargetHeight = 8.5; // Height in feet
  double dLimeLightHeight = 4.0; // Height in feet
  boolean m_LimelightHasValidTarget = false;
  double m_LimelightDriveCommand = 0.0;
  double m_LimelightSteerCommand = 0.0;

  // region 2337 LimeLight Basic
  double targetDistance = 1.5; // Target Area (NEED TO DO Target Area to Inches)
  double driveScaleDown = 1;
  double turnScaleDown = 0.05;
  double tx = 0.0;
  double ta = 0.0;
  double m_speed = 0.0;
  double output = 0.0;
  boolean turnInPlace = false;

  public void LimelightBasic() {
    tx = Vision.getTX();
    ta = Vision.getTA();
    if (Math.abs(tx) > 0) {
      m_drive.arcadeDrive((getDriveSpeed() * driveScaleDown), (tx * turnScaleDown));
    } else {
      m_drive.arcadeDrive(0.0, 0.0);
    }
  }

  public double getDriveSpeed() {
    double ta = Vision.getTA();
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
  // endregion 2337 LimeLight Basic

  @Override
  public void robotInit() {

    // System.out.println("Robot.robotInit()");

    c_Auton.setDefaultOption("Default Auton", "Default");
    c_Auton.addOption("Left", "Left");
    c_Auton.addOption("Center", "Center");
    c_Auton.addOption("Right", "Right");
    SmartDashboard.putData("Auton choices", c_Auton);

    c_Auton_Delay.setDefaultOption("None", "None");
    c_Auton_Delay.addOption("1 second", "1 second");
    c_Auton_Delay.addOption("2 second", "2 second");
    c_Auton_Delay.addOption("3 second", "3 second");
    c_Auton_Delay.addOption("4 second", "4 second");
    c_Auton_Delay.addOption("5 second", "5 second");
    c_Auton_Delay.addOption("6 second", "6 second");
    SmartDashboard.putData("Auto Delay", c_Auton_Delay);

    c_DriveMode.setDefaultOption("Arcade", "Arcade");
    c_DriveMode.addOption("Tank", "Tank");
    SmartDashboard.putData("Drive Mode", c_DriveMode);

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
    // System.out.println("Robot.robotPeriodic()");
    // region SmartDashboard Data
    try {
      Vision.setLEDMode(0);
      SmartDashboard.putNumber("limelight_X", Vision.getTX());
      SmartDashboard.putNumber("limelight_Y", Vision.getTY());
      SmartDashboard.putNumber("limelight_Area", Vision.getTA());
      SmartDashboard.putNumber("limelight_Latency", Vision.getTL());
      SmartDashboard.putNumber("limelight_Valid_Target", Vision.getTV());
      SmartDashboard.putNumber("limelight_Skew", Vision.getTS());
      SmartDashboard.putNumber("limelight_Steering_Adjust", Vision.getSteeringAdjust());
      left_command = 0.0;
      right_command = 0.0;
      left_command += Vision.getSteeringAdjust();
      right_command -= Vision.getSteeringAdjust();
      SmartDashboard.putNumber("left_command", left_command);
      SmartDashboard.putNumber("right_command", right_command);
    } catch (Exception name) {
      // System.out.println("Exception: " + name);
      // System.out.println("Error 272 - Unable to connect to the LimeLight");
      bLimeLightConnected = false;
    }

    SmartDashboard.putNumber("Battery_Voltage", edu.wpi.first.wpilibj.RobotController.getBatteryVoltage());
    SmartDashboard.putString("RoboRio", GetMacAddress.getRIOMAC());

    SmartDashboard.putNumber("Venom_FrontLeft", m_frontLeft.getSpeed());
    SmartDashboard.putNumber("Venom_FrontRight", m_frontRight.getSpeed());
    SmartDashboard.putNumber("Venom_RearLeft", m_rearLeft.getSpeed());
    SmartDashboard.putNumber("Venom_RearRight", m_rearRight.getSpeed());
    // endregion SmartDashboard Data

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
    // System.out.println("Robot.autonomousInit()");
    c_SelectedAuton = c_Auton.getSelected();
    // c_SelectedAuton = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    // System.out.println("Auto selected: " + c_SelectedAuton);

    System.out.println("Front Left: " + m_frontLeft.getPosition());
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    // System.out.println("Robot.autonomousPeriodic()");
    m_frontLeft.enable();
    m_frontRight.enable();

    m_rearLeft.follow(m_frontLeft);
    m_rearRight.follow(m_frontRight);

    switch (c_SelectedAuton) {
    case "Left":
      // Left Auton

      break;
    case "Center":
      // Center Auton

      break;
    case "Right":
      // Right Auton
      if (m_frontLeft.getPosition() > 317) {
        m_drive.arcadeDrive(-0.5, 0.0);
        System.out.println("Front Left: " + m_frontLeft.getPosition());
      }
      break;
    case "Default": // DEFAULT
    default:

      if (m_frontLeft.getPosition() < 338.77) {
        m_drive.arcadeDrive(0.5, 0.0);
        System.out.println("Front Left: " + m_frontLeft.getPosition());
      }

      break;
    }
  }

  @Override
  public void teleopInit() {
    // System.out.println("Robot.teleopInit()");

  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    // System.out.println("Robot.teleopPeriodic()");
    m_frontLeft.enable();
    m_frontRight.enable();
    
    System.out.println("Front Left: " + m_frontLeft.getPosition());
    m_rearLeft.follow(m_frontLeft);
    m_rearRight.follow(m_frontRight);

    // m_drive.setSafetyEnabled(true); // motor safety back on.

    if (bLimeLightConnected) {
      Vision.setLEDMode(0);
    }

    c_SelectedDriveMode = c_DriveMode.getSelected();
    // System.out.println("Drive Mode: " + c_SelectedDriveMode);
    switch (c_SelectedDriveMode) {
    case "Tank":
      // js1.getRawAxis(1); // Speed controls up & down (Controller D)
      // js1.getRawAxis(2) // Speed controls left & right (Controller D)
      m_drive.tankDrive(-js1.getRawAxis(1), -js1.getRawAxis(3)); // Tank Drive
      break;
    case "Arcade": // DEFAULT
    default:
      m_drive.arcadeDrive(-js1.getRawAxis(1), js1.getRawAxis(2)); // Arcade Drive
      break;
    }

    // region button config

    if (js1.getRawButton(btnD_X)) {
      left_command = 0.0;
      right_command = 0.0;
      double dSteer = 0.0;
      if (bLimeLightConnected) {
        dSteer = Vision.getSteeringAdjust();

        left_command += dSteer;
        right_command -= dSteer;
      }

      m_drive.arcadeDrive(0.0, dSteer);
      // m_drive.tankDrive(left_command, -right_command);
    }

    if (js1.getRawButton(btnD_A)) {
      double output = 0;
      output = Vision.getTX() * -0.04;
      output *= 0.5;
      m_SteerAdjust(-output, output);
    }

    if (js1.getRawButton(btnD_B)) {
      LimelightBasic();
    }

    if (js1.getRawButton(btnD_Y)) {

      final double STEER_K = 0.03; // how hard to turn toward˓→the target
      final double DRIVE_K = 0.26; // how hard to drive f˓→toward the target
      final double DESIRED_TARGET_AREA = 13.0; // Area of the target when˓→the robot reaches the wall
      final double MAX_DRIVE = 0.7; // Simple speed limit so we˓→don’t drive too fast
      double tv = Vision.getTV();
      double tx = Vision.getTX();
      double ty = Vision.getTY();
      double ta = Vision.getTA();
      if (tv < 1.0) {
        m_LimelightHasValidTarget = false;
        m_LimelightDriveCommand = 0.0;
        m_LimelightSteerCommand = 0.0;
        return;
      } else {
        m_LimelightHasValidTarget = true;
        // Start with proportional steering
        double steer_cmd = tx * STEER_K;
        m_LimelightSteerCommand = steer_cmd;
        // try to drive forward until the target area reaches our desired area
        double drive_cmd = (DESIRED_TARGET_AREA - ta) * DRIVE_K;
        // don’t let the robot drive too fast into the goal
        if (drive_cmd > MAX_DRIVE) {
          drive_cmd = MAX_DRIVE;
        }
        m_LimelightDriveCommand = drive_cmd;

        m_SteerAdjust(-m_LimelightDriveCommand, m_LimelightDriveCommand);

      }

    }
    // endregion button config

  }

  public void m_SteerAdjust(double left_speed, double right_speed) {
    m_frontLeft.set(left_speed * 0.5);
    m_frontRight.set(right_speed * 0.5);
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    // System.out.println("Robot.testPeriodic()");

  }

  @Override
  public void disabledInit() {

    // System.out.println("Robot.disabledInit()");

  }

  @Override
  public void disabledPeriodic() {
    // System.out.println("Robot.disabledPeriodic()");
    try {
      Vision.setLEDMode(1);
    } catch (Exception name) {
      // System.out.println("Exception: " + name);
      // System.out.println("Error 396 - Unable to connect to the LimeLight");
    }
  }

}