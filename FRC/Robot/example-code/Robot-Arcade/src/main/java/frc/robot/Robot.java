// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// region Imports
// region FRC8767
import frc.robot.Constants;
// endregion FRC8767

// region CTRE
// https://maven.ctr-electronics.com/release/com/ctre/phoenix/Phoenix-frc2022-latest.json
import com.ctre.phoenix.motorcontrol.can.*;
// endregion CTRE

// region Rev Robotics
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
// endregion Rev Robotics

// region WPILib Imports
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// endregion WPILib

// endregion Imports

/**
 * This is a demo program showing the use of the DifferentialDrive class. Runs
 * the motors with
 * arcade steering.
 */
public class Robot extends TimedRobot {

  // region Drive Train
  // region Drive Motor Assignment
  private static final int m_frontLeft_id = Constants.m_frontLeft;
  private static final int m_frontRight_id = Constants.m_frontRight;
  private static final int m_rearLeft_id = Constants.m_rearLeft;
  private static final int m_rearRight_id = Constants.m_rearRight;

  private CANSparkMax m_frontLeft;
  private CANSparkMax m_frontRight;
  private CANSparkMax m_rearLeft;
  private CANSparkMax m_rearRight;
  // endregion Drive Motor Assignment

  private DifferentialDrive m_DriveRobotRobot;
  private DifferentialDrive m_climb;
  // endregion Drive Train

  // region Climber Motor Assignment
  Servo s_RachetPull = new Servo(Constants.s_RatchetPull);
  Servo s_IntakePush = new Servo(Constants.s_IntakePush);

  WPI_TalonSRX m_Climber01 = new WPI_TalonSRX(Constants.m_Climber01);
  WPI_TalonSRX m_Climber02 = new WPI_TalonSRX(Constants.m_Climber02);
  WPI_TalonSRX m_Intake = new WPI_TalonSRX(Constants.m_Intake);
  // endregion Climber Motor Assignment

  // region Joysticks
  private final Joystick js1 = new Joystick(Constants.js1);
  private final Joystick js2 = new Joystick(Constants.js2);
  // endregion Joysticks

  public static Timer rTimer = new Timer();

  // region Dead Band Variables
  double js1_L_UpDown = 0; // Speed controls up & down
  double js1_R_UpDown = 0; // Speed controls up & down
  double js1_L_LeftRight = 0; // Rotate controls left & right
  double js1_R_LeftRight = 0; // Rotate controls left & right
  double js1_L_TSpeed = 0;
  double js1_R_TSpeed = 0;
  double js2_L_UpDown = 0; // Speed controls up & down
  double js2_R_UpDown = 0; // Speed controls up & down
  double js2_L_LeftRight = 0; // Rotate controls left & right
  double js2_R_LeftRight = 0; // Rotate controls left & right
  double js2_L_TSpeed = 0;
  double js2_R_TSpeed = 0;
  // endregion Dead Band Variables

  private PowerDistribution m_pdp = new PowerDistribution();

  @Override
  public void robotInit() {

    // region Initialize the drive train
    // region Initialize the Motors
    m_frontLeft = new CANSparkMax(m_frontLeft_id, MotorType.kBrushed);
    m_frontRight = new CANSparkMax(m_frontRight_id, MotorType.kBrushed);
    m_rearLeft = new CANSparkMax(m_rearLeft_id, MotorType.kBrushed);
    m_rearRight = new CANSparkMax(m_rearRight_id, MotorType.kBrushed);
    // private final MotorController m_frontLeft = new PWMSparkMax(0);
    // private final MotorController m_frontRight = new PWMSparkMax(1);

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_frontLeft.setInverted(true);
    m_frontRight.setInverted(false);

    /**
     * The RestoreFactoryDefaults method can be used to reset the configuration
     * parameters
     * in the SPARK MAX to their factory default state. If no argument is passed,
     * these
     * parameters will not persist between power cycles
     */
    m_frontLeft.restoreFactoryDefaults();
    m_frontRight.restoreFactoryDefaults();
    m_rearLeft.restoreFactoryDefaults();
    m_rearRight.restoreFactoryDefaults();

    /**
     * In CAN mode, one SPARK MAX can be configured to follow another. This is done
     * by calling
     * the follow() method on the SPARK MAX you want to configure as a follower, and
     * by passing
     * as a parameter the SPARK MAX you want to configure as a leader.
     */
    m_rearLeft.follow(m_frontLeft);
    m_rearRight.follow(m_frontRight);

    // endregion Initialize the Motors

    m_DriveRobotRobot = new DifferentialDrive(m_frontLeft, m_frontRight);
    // endregion Initialize the drive train

    m_climb = new DifferentialDrive(m_Climber01, m_Climber02);

  }

  @Override
  public void robotPeriodic() {

    // region diagnostic
    /*
     * Get the current going through channel 7, in Amperes. The PDP returns the
     * current in increments of 0.125A. At low currents
     * the current readings tend to be less accurate.
     */

    // SmartDashboard.putNumber("Front Right Motor", m_pdp.getCurrent(0));
    // SmartDashboard.putNumber("Rear Right Motor", m_pdp.getCurrent(14));
    // SmartDashboard.putNumber("Rear Left Motor", m_pdp.getCurrent(12));
    // SmartDashboard.putNumber("Front Left Motor", m_pdp.getCurrent(3));

    /*
     * Get the voltage going into the PDP, in Volts.
     * The PDP returns the voltage in increments of 0.05 Volts.
     */
    // SmartDashboard.putNumber("Voltage", m_pdp.getVoltage());

    /*
     * Retrieves the temperature of the PDP, in degrees Celsius.
     */
    // SmartDashboard.putNumber("Temperature", m_pdp.getTemperature());
    // endregion diagnostic

  }

  // region Auton
  @Override
  public void autonomousInit() {
    rTimer.reset();
    rTimer.start();
  }

  @Override
  public void autonomousPeriodic() {
    do {
      m_DriveRobotRobot.tankDrive(0.6, 0.6);
    } while (rTimer.get() < 10);
    m_DriveRobotRobot.tankDrive(0.0, 0.0);
    rTimer.stop();
    rTimer.reset();

    rTimer.start();
    do {
      m_DriveRobotRobot.tankDrive(0.6, -0.6);
    } while (rTimer.get() < 10);
    m_DriveRobotRobot.tankDrive(0.0, 0.0);
    rTimer.stop();
    rTimer.reset();

    rTimer.start();
    do {
      m_DriveRobotRobot.tankDrive(0.6, 0.6);
    } while (rTimer.get() < 10);
    m_DriveRobotRobot.tankDrive(0.0, 0.0);
    rTimer.stop();
    rTimer.reset();
  }
  // endregion Auton

  @Override
  public void teleopInit() {
    /* factory default values */
    m_Climber01.configFactoryDefault();
    m_Climber02.configFactoryDefault();

    /* flip values so robot moves forward when stick-forward/LEDs-green */
    m_Climber01.setInverted(false); // <<<<<< Adjust this
    m_Climber02.setInverted(false); // <<<<<< Adjust this

  }

  @Override
  public void teleopPeriodic() {

    // region Deadbands
    // region get raw values

        js1_L_UpDown = js1.getRawAxis(Constants.axisD_lUpDown); // Y Axis - Up and Down
        js1_L_LeftRight = js1.getRawAxis(Constants.axisD_lLeftRight); // X Axis - Left and Right
        js1_R_UpDown = -js1.getRawAxis(Constants.axisD_rUpDown); // Z Rotate - Up and Down
        js1_R_LeftRight = js1.getRawAxis(Constants.axisD_rLeftRight); // Z Axis - Left and Right
    

    js2_L_UpDown = js2.getRawAxis(Constants.axisD_lUpDown); // Y Axis - Up and Down
    js2_L_LeftRight = js2.getRawAxis(Constants.axisD_lLeftRight); // X Axis - Left and Right
    js2_R_UpDown = -js2.getRawAxis(Constants.axisD_rUpDown); // Z Rotate - Up and Down
    js2_R_LeftRight = js2.getRawAxis(Constants.axisD_rLeftRight); // Z Axis - Left and Right
    // endregion get raw values


    double dL_UD_Speed = Double.parseDouble("0.8");
    double dR_UD_Speed = Double.parseDouble("0.8");
    double dR_LR_Speed = Double.parseDouble("0.6");

    // double dL_UD_Speed = 0.8;
    // double dR_UD_Speed = 0.8;
    // double dR_LR_Speed = 0.6;

    // region Driver Controller
    if ((js1_L_UpDown > Constants.js1_L_UpDown_Deadband_Low)) {
      js1_L_UpDown = ((js1_L_UpDown - Constants.js1_L_UpDown_Deadband_Low) * dL_UD_Speed);
    } else if ((js1_L_UpDown < -Constants.js1_L_UpDown_Deadband_Low)) {
      js1_L_UpDown = ((js1_L_UpDown + Constants.js1_L_UpDown_Deadband_Low) * dL_UD_Speed);
    } else {
      js1_L_UpDown = 0; // If between boundaries. Do nothing.
    }

    if ((js1_R_UpDown > Constants.js1_R_UpDown_Deadband_Low)) {
      js1_R_UpDown = ((js1_R_UpDown - Constants.js1_R_UpDown_Deadband_Low) * dR_UD_Speed);
    } else if ((js1_R_UpDown < -Constants.js1_R_UpDown_Deadband_Low)) {
      js1_R_UpDown = ((js1_R_UpDown + Constants.js1_R_UpDown_Deadband_Low) * dR_UD_Speed);
    } else {
      js1_R_UpDown = 0; // If between boundaries. Do nothing.
    }

    if ((js1_R_LeftRight > Constants.js1_R_LeftRight_Deadband_Low)) {
      js1_R_LeftRight = ((js1_R_LeftRight - Constants.js1_R_LeftRight_Deadband_Low) * dR_LR_Speed);
    } else if ((js1_R_LeftRight < -Constants.js1_R_LeftRight_Deadband_Low)) {
      js1_R_LeftRight = ((js1_R_LeftRight + Constants.js1_R_LeftRight_Deadband_Low) * dR_LR_Speed);
    } else {
      js1_R_LeftRight = 0; // If between boundaries. Do nothing.
    }
    // endregion Driver Controller

    // region Operator Controller
    if ((js2_L_UpDown > Constants.js2_L_UpDown_Deadband_Low)) {
      js2_L_UpDown = ((js2_L_UpDown - Constants.js2_L_UpDown_Deadband_Low) * Constants.js2_L_UpDown_Deadband_High);
    } else if ((js2_L_UpDown < -Constants.js2_L_UpDown_Deadband_Low)) {
      js2_L_UpDown = ((js2_L_UpDown + Constants.js2_L_UpDown_Deadband_Low) * Constants.js2_L_UpDown_Deadband_High);
    } else {
      js2_L_UpDown = 0; // If between boundaries. Do nothing.
    }
    if ((js2_R_UpDown > Constants.js2_R_UpDown_Deadband_Low)) {
      js2_R_UpDown = ((js2_R_UpDown - Constants.js2_R_UpDown_Deadband_Low) * Constants.js2_R_UpDown_Deadband_High);
    } else if ((js2_R_UpDown < -Constants.js2_R_UpDown_Deadband_Low)) {
      js2_R_UpDown = ((js2_R_UpDown + Constants.js2_R_UpDown_Deadband_Low) * Constants.js2_R_UpDown_Deadband_High);
    } else {
      js2_R_UpDown = 0; // If between boundaries. Do nothing.
    }
    // endregion Operator Controller
    // endregion Deadbands

    // Drive with arcade drive.
    // That means that the Y axis drives forward
    // and backward, and the X turns left and right.
    // m_DriveRobotRobot.arcadeDrive(-js1.getZ(), js1.getY());
    m_DriveRobotRobot.arcadeDrive(-js1_R_LeftRight,js1_L_UpDown); // regular teleop driving
    // m_DriveRobotRobot.tankDrive(-js1_L_UpDown, js1_R_UpDown); // regular teleop
    // driving

    double xSpeed = js2.getRawAxis(1) * -1; // make forward stick positive
    double zRotation = js2.getRawAxis(2); // WPI Drivetrain uses positive=> right

    m_climb.arcadeDrive(xSpeed, 0);
    // _drive.arcadeDrive(xSpeed, zRotation);

    if (js2.getRawButtonPressed(Constants.btnD_X)) {
      s_RachetPull.set(1.0);
    }
    if (js2.getRawButtonReleased(Constants.btnD_X)) {
      s_RachetPull.set(0.0);
    }

    if (js2.getRawButtonPressed(Constants.btnD_A)) {
      s_IntakePush.set(1.0);
    }
    if (js2.getRawButtonReleased(Constants.btnD_A)) {
      s_IntakePush.set(0.0);
    }

    if (js2.getRawButtonPressed(Constants.btnD_LB)) {
      m_Intake.set(1.0);
    }
    if (js2.getRawButtonReleased(Constants.btnD_LB)) {
      m_Intake.set(0.0);
    }

    if (js2.getRawButtonPressed(Constants.btnD_LT)) {
      m_Intake.set(-1.0);
    }
    if (js2.getRawButtonReleased(Constants.btnD_LT)) {
      m_Intake.set(0.0);
    }

    /* hold down btn1 to print stick values */
    // if (js2.getRawButton(1)) {
    // System.out.println("xSpeed:" + xSpeed + " zRotation:" + zRotation);
    // }

  }
}
