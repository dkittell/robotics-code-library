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
import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.drive.MecanumDrive;

import frc.robot.Climber;
import frc.robot.ControlWheel;
import frc.robot.DriveTrain;
import frc.robot.Intake;
import frc.robot.Shooter;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
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

  Timer myTimer;

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

    // Reset timer to 0sec
    myTimer.reset();

    // Start timer
    myTimer.start();

  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    // Auton Mode
    switch (m_autoSelected) {
    case kLeftAuto:
      // Put custom auto code here

      // If is has been less than 2 seconds since autonomous started, drive forwards
      if (myTimer.get() < 2.0) {
        m_robotDrive.driveCartesian(0, 0, .4, 0);
      }

      // If more than 2 seconds have elapsed, stop driving and turn off the timer
      else {
        m_robotDrive.driveCartesian(0, 0, 0, 0);
        myTimer.stop();
      }

      // 1st is Strife
      // 2nd is Turn
      // 3rd is forward/reverse
      // 4th
      break;
    case kCenterAuto:
      // Put custom auto code here
      m_robotDrive.driveCartesian(0, 0, .4, 0);
      // 1st is Strife
      // 2nd is Turn
      // 3rd is forward/reverse
      // 4th
      break;
    case kRightAuto:
      // Put custom auto code here
      m_robotDrive.driveCartesian(0, 0, .4, 0);
      // 1st is Strife
      // 2nd is Turn
      // 3rd is forward/reverse
      // 4th
      break;
    case kDefaultAuto:
    default:
      // Put default auto code here
      m_robotDrive.driveCartesian(0, 0, .4, 0);
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

    // double j_xAxis = m_joystick.getX() / 2;
    // double j_yAxis = m_joystick.getY() / 2;
    // double j_zAxis = m_joystick.getZ() / 2;
    double j_xAxis = m_joystick.getX(); // Left Toggle Left/Right
    double j_yAxis = m_joystick.getY(); // Left Toggle Up/Down
    double j_zAxis = m_joystick.getZ(); // Right Toggle Left/Right
    double r_gyro = m_gyro.getAngle();

    // m_robotDrive.driveCartesian(m_joystick.getX(), m_joystick.getY(),
    // m_joystick.getZ(), m_gyro.getAngle());
    m_robotDrive.driveCartesian(j_xAxis, -j_zAxis, j_yAxis, r_gyro);
    // 1st is Strife
    // 2nd is Turn
    // 3rd is forward/reverse
    // 4th

  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
