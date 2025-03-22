/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;

import java.lang.annotation.Target;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.drive.MecanumDrive;

public class Robot extends TimedRobot {

  // gyro calibration constant, may need to be adjusted;
  // gyro value of 360 is set to correspond to one full revolution
  private static final double kVoltsPerDegreePerSecond = 0.0128;
  private static final int kGyroPort = 0;
  private final AnalogGyro m_gyro = new AnalogGyro(kGyroPort);

  private static final int kJoystickPort = 0;
  private final Joystick m_joystick = new Joystick(kJoystickPort);

  // region Motors
  private static final int kFrontLeftChannel = 0;
  private static final int kRearLeftChannel = 1;
  private static final int kFrontRightChannel = 2;
  private static final int kRearRightChannel = 3;
  private MecanumDrive m_robotDrive;
  // endregion Motors

  // region Controller Definition
  // region Dead Band Variables JS1
  double js1lSpeed = 0; // Speed controls up & down
  double js1rSpeed = 0; // Speed controls up & down
  double js1lRotate = 0; // Rotate controls left & right
  double js1rRotate = 0; // Rotate controls left & right
  double js1LTSpeed = 0;
  double js1RTSpeed = 0;

  // JS1 Left Deadband
  double js1lDeadband_1 = 0.15;
  double js1lDeadband_2 = 0.90;

  // JS1 Right Deadband
  double js1rDeadband_1 = 0.15;
  double js1rDeadband_2 = 0.60;
  // endregion Dead Band Variables JS1

  // region Dead Band Variables JS2
  double js2lSpeed = 0; // Speed controls up & down
  double js2rSpeed = 0; // Speed controls up & down
  double js2lRotate = 0; // Rotate controls left & right
  double js2rRotate = 0; // Rotate controls left & right
  double js2LTSpeed = 0;
  double js2RTSpeed = 0;

  // JS2 Left Deadband
  double js2lDeadband_1 = 0.15;
  double js2lDeadband_2 = 0.90;

  // JS2 Right Deadband
  double js2rDeadband_1 = 0.15;
  double js2rDeadband_2 = 0.50;
  // endregion Dead Band Variables JS2

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

  private static final String kDefaultAuto = "Default";
  private static final String kLeftAuto = "Left";
  private static final String kCenterAuto = "Center";
  private static final String kRightAuto = "Right";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("Left", kLeftAuto);
    m_chooser.addOption("Center", kCenterAuto);
    m_chooser.addOption("Right", kRightAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    PWMVictorSPX frontLeft = new PWMVictorSPX(kFrontLeftChannel);
    PWMVictorSPX rearLeft = new PWMVictorSPX(kRearLeftChannel);
    PWMVictorSPX frontRight = new PWMVictorSPX(kFrontRightChannel);
    PWMVictorSPX rearRight = new PWMVictorSPX(kRearRightChannel);

    // Invert the left side motors.
    // You may need to change or remove this to match your robot.
    frontLeft.setInverted(true);
    rearLeft.setInverted(true);

    m_robotDrive = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);

    m_gyro.setSensitivity(kVoltsPerDegreePerSecond);
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
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);

  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    // Auton Mode
    m_autoSelected = m_chooser.getSelected();
    System.out.println("Auto selected: " + m_autoSelected);

    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);

    switch (m_autoSelected) {
    case kLeftAuto:
      // Put custom auto code here

      // If is has been less than 2 seconds since autonomous started, drive forwards
      // if (myTimer.get() < 2.0) {
      // m_robotDrive.driveCartesian(0, 0, -.4, 0); // Drive Forward
      m_robotDrive.driveCartesian(0, 0, .4, 0); // Drive Backward
      // }

      // If more than 2 seconds have elapsed, stop driving and turn off the timer
      // else {
      // m_robotDrive.driveCartesian(0, 0, 0, 0);
      // myTimer.stop();
      // }

      // 1st is Strife
      // 2nd is Turn
      // 3rd is forward/reverse
      // 4th
      break;
    case kCenterAuto:
      // Put custom auto code here
      m_robotDrive.driveCartesian(0, 0, -.4, 0); // Drive Forward
      // m_robotDrive.driveCartesian(0, 0, .4, 0); // Drive Backward
      // 1st is Strife
      // 2nd is Turn
      // 3rd is forward/reverse
      // 4th
      break;
    case kRightAuto:
      // Put custom auto code here
      // m_robotDrive.driveCartesian(0, 0, -.4, 0); // Drive Forward
      // m_robotDrive.driveCartesian(0, 0, .4, 0); // Drive Backward
      // m_robotDrive.driveCartesian(0.4, 0, 0, 0); // Strife Right
      m_robotDrive.driveCartesian(-0.4, 0, 0, 0); // Strife Left
      // 1st is Strife
      // 2nd is Turn
      // 3rd is forward/reverse
      // 4th
      break;
    case kDefaultAuto:
      break;
    default:
      // Put default auto code here
      // m_robotDrive.driveCartesian(0, 0, -.4, 0); // Drive Forward
      m_robotDrive.driveCartesian(0, 0, .4, 0); // Drive Backward
      // 1st is Strife
      // 2nd is Turn
      // 3rd is forward/reverse
      // 4th
      break;
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {

    // Teleop Mode
    // Limelight Data Start
    // get the default instance of NetworkTables
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    // get a reference to the subtable called "datatable"
    NetworkTable table = inst.getTable("limelight");

    inst.startClientTeam(3535);

    inst.startDSClient(); // recommended if running on DS computer; this gets the robot IP from the DS

    // NetworkTableEntry TeamEntry = table.getEntry("tx");
    NetworkTableEntry xEntry = table.getEntry("tx");
    NetworkTableEntry yEntry = table.getEntry("ty");
    NetworkTableEntry aEntry = table.getEntry("ta");
    NetworkTableEntry lEntry = table.getEntry("tl");
    NetworkTableEntry vEntry = table.getEntry("tv");
    NetworkTableEntry sEntry = table.getEntry("ts");

    NetworkTableEntry tshortEntry = table.getEntry("tshort");
    NetworkTableEntry tlongEntry = table.getEntry("tlong");
    NetworkTableEntry thorEntry = table.getEntry("thor");
    NetworkTableEntry tvertEntry = table.getEntry("tvert");
    NetworkTableEntry getpipeEntry = table.getEntry("getpipe");
    NetworkTableEntry camtranEntry = table.getEntry("camtran");
    NetworkTableEntry ledModeEntry = table.getEntry("ledMode");

    // double tx = xEntry.getDouble(0.0);
    double tx = xEntry.getDouble(0.0); // Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees)
    double ty = yEntry.getDouble(0.0); // Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees)
    double ta = aEntry.getDouble(0.0); // Target Area (0% of image to 100% of image)
    double tl = lEntry.getDouble(0.0); // The pipeline’s latency contribution (ms) Add at least 11ms for image capture
                                       // latency.
    double tv = vEntry.getDouble(0.0); // Whether the limelight has any valid targets (0 or 1)
    double ts = sEntry.getDouble(0.0); // Skew or rotation (-90 degrees to 0 degrees)

    // double tshort = tshortEntry.getString(); // Sidelength of shortest side of
    // the fitted bounding box (pixels)
    // double tlong = tlong // Sidelength of longest side of the fitted bounding box
    // (pixels)
    // double thor = thor // Horizontal sidelength of the rough bounding box (0 -
    // 320 pixels)
    // double tvert = tvert // Vertical sidelength of the rough bounding box (0 -
    // 320 pixels)
    // double getpipe = getpipe // True active pipeline index of the camera (0 .. 9)
    // double camtran = camtran // Results of a 3D position solution, 6 numbers:
    // Translation (x,y,y) Rotation(pitch,yaw,roll)

    // ledModeEntry.setNumber(0); // use the LED Mode set in the current pipeline
    // ledModeEntry.setNumber(1); // force off
    // ledModeEntry.setNumber(2); // force blink
    // ledModeEntry.setNumber(3); // force on

    // System.out.println("X: " + tx);
    // System.out.println("Y: " + ty);
    // System.out.println("A: " + ta);
    // System.out.println("L: " + tl);
    // System.out.println("V: " + tv);
    // System.out.println("S: " + tv);

    // post to smart dashboard periodically
    SmartDashboard.putNumber("Limelight X", tx);
    SmartDashboard.putNumber("Limelight Y", ty);
    SmartDashboard.putNumber("Limelight Area", ta);
    SmartDashboard.putNumber("Limelight Latency", tl);
    SmartDashboard.putNumber("Limelight Valid Target", tv);
    SmartDashboard.putNumber("Limelight Skew", ts);

    boolean bValidTarget = false;
    if (tv == 1.0f) {
      bValidTarget = true;
    }
    SmartDashboard.putBoolean("Valid Target", bValidTarget);

    // Limelight Data End

    // double j_xAxis = m_joystick.getX() / 2;
    // double j_yAxis = m_joystick.getY() / 2;
    // double j_zAxis = m_joystick.getZ() / 2;
    double j_xAxis = m_joystick.getX(); // Left Toggle Left/Right
    double j_yAxis = m_joystick.getY(); // Left Toggle Up/Down
    double j_zAxis = m_joystick.getZ(); // Right Toggle Left/Right
    double r_gyro = m_gyro.getAngle();

    // System.out.println("j_xAxis: " + j_xAxis);
    // System.out.println("j_yAxis: " + j_yAxis);
    // System.out.println("j_zAxis: " + j_zAxis);

    // m_robotDrive.driveCartesian(m_joystick.getX(), m_joystick.getY(),
    // m_joystick.getZ(), m_gyro.getAngle());
    m_robotDrive.driveCartesian(j_xAxis, -j_zAxis, -j_yAxis, r_gyro);
    // 1st is Strife
    // 2nd is Turn
    // 3rd is forward/reverse
    // 4th

    double left_command = 0.0;
    double right_command = 0.0;
    double drivingAdjust = 0.0f;
    double steeringAdjust = 0.0f;

    if (m_joystick.getRawButton(btnD_X)) {
      double Kp = -0.1f;
      double min_command = 0.05f;
      double heading_error = tx;
      double steering_adjust = 0.0f;
      double KpAim = -0.1f;
      double KpDist = 0.0f; // 0.09;
      double AimMinCmd = 0.095f;
      // double KpDistance = -0.1f;
      // double min_aim_command = 0.05f;

      // System.out.println(" ");

      j_xAxis *= 0.70; // steer
      j_yAxis *= 0.70; // drive

      if (tx > 1.0) {
        steering_adjust = Kp * heading_error - min_command;
      } else if (tx < 1.0) {
        steering_adjust = Kp * heading_error + min_command;
      }
      left_command += steering_adjust;
      right_command -= steering_adjust;

      System.out.println("left_command: " + left_command);
      System.out.println("right_command: " + right_command);

      // Aim error and distance error based on calibrated limelight cross-hair
      double aim_error = tx;
      double dist_error = tv;

      // Steering adjust with a 0.2 degree deadband (close enough at 0.2deg)
      steeringAdjust = KpAim * aim_error;
      if (aim_error > .2f)
        steeringAdjust += AimMinCmd;
      else if (aim_error < -.2f)
        steeringAdjust -= AimMinCmd;

      // Distance adjust, drive to the correct distance from the goal
      drivingAdjust = KpDist * dist_error;

      m_robotDrive.driveCartesian(0.0, steering_adjust, drivingAdjust, 0.0);
      // 1st is Strife
      // 2nd is Turn
      // 3rd is forward/reverse
      // 4th

    }

    if (m_joystick.getRawButtonPressed(btnD_A)) // Aim using pipeline 0
    {
      left_command += drivingAdjust - steeringAdjust;
      right_command += drivingAdjust + steeringAdjust;

      System.out.println("left_command: " + left_command);
      System.out.println("right_command: " + right_command);
    }

  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
