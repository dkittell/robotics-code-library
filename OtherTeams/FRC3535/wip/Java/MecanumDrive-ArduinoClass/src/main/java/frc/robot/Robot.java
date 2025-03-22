/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.MecanumDrive;

import edu.wpi.first.wpilibj.DriverStation;

import edu.wpi.first.wpilibj.smartdashboard.*;

/**
 * This is a demo program showing how to use Mecanum control with the RobotDrive
 * class.
 */
public class Robot extends TimedRobot {

  Timer mTimer;

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

  private static final int kJoystickPort = 0;
  private final Joystick m_joystick = new Joystick(kJoystickPort);
  // region Motors
  private static final int kFrontLeftChannel = 0;
  private static final int kRearLeftChannel = 1;
  private static final int kFrontRightChannel = 2;
  private static final int kRearRightChannel = 3;
  // endregion Motors

  private MecanumDrive m_robotDrive;

  @Override
  public void robotInit() {
    PWMVictorSPX frontLeft = new PWMVictorSPX(kFrontLeftChannel);
    PWMVictorSPX rearLeft = new PWMVictorSPX(kRearLeftChannel);
    PWMVictorSPX frontRight = new PWMVictorSPX(kFrontRightChannel);
    PWMVictorSPX rearRight = new PWMVictorSPX(kRearRightChannel);

    // CameraServer camera01 = CameraServer.getInstance();
    // camera01.startAutomaticCapture(0);

    // Invert the left side motors.
    // You may need to change or remove this to match your robot.
    frontLeft.setInverted(true);
    rearLeft.setInverted(true);

    // region Dead Band Variables JS1
    js1lSpeed = 0; // Speed controls up & down
    js1rSpeed = 0; // Speed controls up & down
    js1lRotate = 0; // Rotate controls left & right
    js1rRotate = 0; // Rotate controls left & right
    js1LTSpeed = 0;
    js1RTSpeed = 0;
    // endregion Dead Band Variables JS1

    // region Dead Band Variables JS2
    js2lSpeed = 0; // Speed controls up & down
    js2rSpeed = 0; // Speed controls up & down
    js2lRotate = 0; // Rotate controls left & right
    js2rRotate = 0; // Rotate controls left & right
    js2LTSpeed = 0;
    js2RTSpeed = 0;
    // endregion Dead Band Variables JS2

    m_robotDrive = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);

  }

  @Override
  public void robotPeriodic() {
    // TODO Auto-generated method stub
    super.robotPeriodic();
    SmartDashboard.putNumber("js1lSpeed", js1lSpeed);
    SmartDashboard.putNumber("js1lRotate", js1lRotate);
    SmartDashboard.putNumber("js1rRotate", js1rRotate);

    SmartDashboard.putNumber("js1lS", m_joystick.getY());
    SmartDashboard.putNumber("js1lR", m_joystick.getX());
    SmartDashboard.putNumber("js1rR", m_joystick.getZ());    
  }

  @Override
  public void autonomousInit() {
    // TODO Auto-generated method stub
    super.autonomousInit();

  }

  @Override
  public void autonomousPeriodic() {
    // TODO Auto-generated method stub
    super.autonomousPeriodic();


  }

  @Override
  public void teleopPeriodic() {

    // double j_xAxis = m_joystick.getX(); // Left Toggle Left/Right
    // double j_yAxis = m_joystick.getY(); // Left Toggle Up/Down
    // double j_zAxis = m_joystick.getZ(); // Right Toggle Left/Right

    // double j_xAxis = m_joystick.getX() / 2; // Left Toggle Left/Right
    // double j_yAxis = -m_joystick.getY() / 2; // Left Toggle Up/Down
    // double j_zAxis = m_joystick.getZ() / 2; // Right Toggle Left/Right

    // js1lSpeed = -m_joystick.getY(); // Left Toggle Up/Down
    // js1lRotate = m_joystick.getX(); // Left Toggle Left/Right
    // js1rRotate = -m_joystick.getZ(); // Right Toggle Left/Right

    js1lSpeed = -m_joystick.getY() / 2; // Left Toggle Up/Down
    js1lRotate = m_joystick.getX() / 2; // Left Toggle Left/Right
    js1rRotate = -m_joystick.getZ() / 2; // Right Toggle Left/Right

    // movement, and Z axis for rotation.
    // m_robotDrive.driveCartesian(m_joystick.getX(), m_joystick.getY(),
    m_robotDrive.driveCartesian(js1lRotate, js1rRotate, js1lSpeed, 0.0);

  }
}
