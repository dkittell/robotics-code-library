
package frc.robot;

// region Imports
// region FRC3535 Imports
import frc.robot.constants.*;
import frc.robot.subsystems.GetMacAddress;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.setSmartDashboard;
import frc.robot.frc3535_Variables;
// endregion FRC3535 Imports

//region CTRE Imports
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
//endregion CTRE Imports

// region FRC WPILib Imports
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.TimedRobot;

// region Color Sensor
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.WPILibVersion;
// endregion Color Sensor
// endregion FRC WPILib Imports

// region RevRobotics Color Sensor
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
// endregion RevRobotics Color Sensor

// region Java Imports
import java.lang.Object;
import java.net.InetAddress;
import java.net.NetworkInterface;
import java.net.SocketException;
import java.net.UnknownHostException;
// endregion Java Imports
//endregion Imports

public class Robot extends TimedRobot {

  // region CTRE
  static com.ctre.phoenix.motorcontrol.can.TalonSRX m_ControlPanel = new com.ctre.phoenix.motorcontrol.can.TalonSRX(
      frc3535_Variables.m_ControlPanel);
  // static com.ctre.phoenix.motorcontrol.can.TalonSRX m_Climber01 = new
  // com.ctre.phoenix.motorcontrol.can.TalonSRX(frc3535_Variables.m_Climber01);
  // static com.ctre.phoenix.motorcontrol.can.TalonSRX m_Climber02 = new
  // com.ctre.phoenix.motorcontrol.can.TalonSRX(frc3535_Variables.m_Climber02);
  // static com.ctre.phoenix.motorcontrol.can.TalonSRX m_ClimberPosition = new
  // com.ctre.phoenix.motorcontrol.can.TalonSRX(frc3535_Variables.m_ClimberPosition);
  static com.ctre.phoenix.motorcontrol.can.TalonSRX m_Intake = new com.ctre.phoenix.motorcontrol.can.TalonSRX(
      frc3535_Variables.m_Intake);
  static com.ctre.phoenix.motorcontrol.can.TalonSRX m_Uptake = new com.ctre.phoenix.motorcontrol.can.TalonSRX(
      frc3535_Variables.m_Uptake);
  // endregion CTRE

  private final PowerDistributionPanel m_pdp = new PowerDistributionPanel(frc3535_Variables.m_pdp);

  // region Venom
  public com.playingwithfusion.CANVenom m_frontLeft = new com.playingwithfusion.CANVenom(frc3535_Variables.m_frontLeft);
  public com.playingwithfusion.CANVenom m_rearLeft = new com.playingwithfusion.CANVenom(frc3535_Variables.m_rearLeft);
  public com.playingwithfusion.CANVenom m_frontRight = new com.playingwithfusion.CANVenom(frc3535_Variables.m_frontRight);
  public com.playingwithfusion.CANVenom m_rearRight = new com.playingwithfusion.CANVenom(frc3535_Variables.m_rearRight);
  public com.playingwithfusion.CANVenom m_Shooter = new com.playingwithfusion.CANVenom(frc3535_Variables.m_Shooter);

  public void resetEncoders() {
    m_frontLeft.setPosition(0);
    m_frontRight.setPosition(0);
    m_rearLeft.setPosition(0);
    m_rearRight.setPosition(0);
    m_frontLeft.resetPosition();
    m_frontRight.resetPosition();
    m_rearLeft.resetPosition();
    m_rearRight.resetPosition();
  }
  // endregion Venom

  public void m_SteerAdjust(double left_speed, double right_speed) {
    m_frontLeft.set(left_speed * 0.5);
    m_frontRight.set(right_speed * 0.5);
  }

  DifferentialDrive m_drive = new DifferentialDrive(m_frontLeft, m_frontRight);

  // region Controller Definition
  private final Joystick js1 = new Joystick(frc3535_Variables.js1);
  private final Joystick js2 = new Joystick(frc3535_Variables.js2);
  // endregion Controller Definition

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

  private String c_SelectedAuton;
  private String c_SelectedDriveMode;
  private final SendableChooser<String> c_Auton = new SendableChooser<>();
  private final SendableChooser<String> c_Auton_Delay = new SendableChooser<>();
  private final SendableChooser<String> c_DriveMode = new SendableChooser<>();

  boolean bLimeLightConnected = true;

  // region LimeLight Basic
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
  // endregion LimeLight Basic

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

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();
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

    } catch (Exception name) {
      // System.out.println("Exception: " + name);
      // System.out.println("Error 272 - Unable to connect to the LimeLight");
      bLimeLightConnected = false;
    }
    SmartDashboard.putNumber("Shooter_Speed_Graph", m_Shooter.getSpeed());
    SmartDashboard.putNumber("Shooter_Speed", m_Shooter.getSpeed());
    SmartDashboard.putNumber("Battery_Voltage", edu.wpi.first.wpilibj.RobotController.getBatteryVoltage());
    SmartDashboard.putString("RoboRio", GetMacAddress.getRIOMAC());
    SmartDashboard.putString("Shooter_Temperature", m_Shooter.getTemperature() + " C");
    SmartDashboard.putNumber("Shooter_Output_Current ", m_Shooter.getOutputCurrent());
    SmartDashboard.putNumber("Shooter_Output_Voltage ", m_Shooter.getOutputVoltage());
    SmartDashboard.putNumber("Venom_FrontLeft", m_frontLeft.getSpeed());
    SmartDashboard.putNumber("Venom_FrontRight", m_frontRight.getSpeed());
    SmartDashboard.putNumber("Venom_RearLeft", m_rearLeft.getSpeed());
    SmartDashboard.putNumber("Venom_RearRight", m_rearRight.getSpeed());
    // endregion SmartDashboard Data

    // region Deadbands

    js1_L_Speed = js1.getRawAxis(1); // Speed controls up & down
    // js1_L_TSpeed = js1.getRawAxis(2); // L Trigger Axis (Controller X)
    // js1_R_TSpeed = js1.getRawAxis(3); // R Trigger Axis (Controller X)
    // js1_R_Speed = js1.getRawAxis(4); // Speed controls left & right (Controller
    // X)
    js1_R_Speed = -js1.getRawAxis(2); // Speed controls left & right (Controller D)
    js1_L_Rotate = js1.getRawAxis(0); // Rotate controls left & right
    js1_R_Rotate = js1.getRawAxis(2); // Rotate controls left & right

    js2_L_Speed = js2.getRawAxis(1); // Speed controls up & down
    // js2_L_TSpeed = js2.getRawAxis(2); // L Trigger Axis (Controller X)
    // js2_R_TSpeed = js2.getRawAxis(3); // R Trigger Axis (Controller X)
    // js2_R_Speed = js2.getRawAxis(4); // Speed controls left & right (Controller
    // X)
    js2_R_Speed = -js2.getRawAxis(3); // Speed controls Up & Down (Controller D)
    js2_L_Rotate = js2.getRawAxis(0); // Rotate controls left & right
    js2_R_Rotate = js2.getRawAxis(2); // Rotate controls left & right
    // if (js1_L_Deadband_High != js1_L_Deadband_Max) {
    // js1_L_Deadband_High = js1_L_Deadband_Max;
    // }

    // if (js1_R_Deadband_High != js1_R_Deadband_Max) {
    // js1_R_Deadband_High = js1_R_Deadband_Max;
    // }

    // if (js2_L_Deadband_High != js2_L_Deadband_Max) {
    // js2_L_Deadband_High = js2_L_Deadband_Max;
    // }

    // if (js2_R_Deadband_High != js2_R_Deadband_Max) {
    // js2_R_Deadband_High = js2_R_Deadband_Max;
    // }

    if ((js1_L_Speed > js1_L_Deadband_Low)) {
      js1_L_Speed = ((js1_L_Speed - js1_L_Deadband_Low) * js1_L_Deadband_High);
    } else if ((js1_L_Speed < -js1_L_Deadband_Low)) {
      js1_L_Speed = ((js1_L_Speed + js1_L_Deadband_Low) * js1_L_Deadband_High);
    } else {
      js1_L_Speed = 0; // If between boundaries. Do nothing.
    }
    if ((js1_R_Speed > js1_R_Deadband_Low)) {
      js1_R_Speed = ((js1_R_Speed - js1_R_Deadband_Low) * js1_R_Deadband_High);
    } else if ((js1_R_Speed < -js1_R_Deadband_Low)) {
      js1_R_Speed = ((js1_R_Speed + js1_R_Deadband_Low) * js1_R_Deadband_High);
    } else {
      js1_R_Speed = 0; // If between boundaries. Do nothing.
    }

    if ((js2_L_Speed > js2_L_Deadband_Low)) {
      js2_L_Speed = ((js2_L_Speed - js2_L_Deadband_Low) * js2_L_Deadband_High);
    } else if ((js2_L_Speed < -js2_L_Deadband_Low)) {
      js2_L_Speed = ((js2_L_Speed + js2_L_Deadband_Low) * js2_L_Deadband_High);
    } else {
      js2_L_Speed = 0; // If between boundaries. Do nothing.
    }
    if ((js2_R_Speed > js2_R_Deadband_Low)) {
      js2_R_Speed = ((js2_R_Speed - js2_R_Deadband_Low) * js2_R_Deadband_High);
    } else if ((js2_R_Speed < -js2_R_Deadband_Low)) {
      js2_R_Speed = ((js2_R_Speed + js2_R_Deadband_Low) * js2_R_Deadband_High);
    } else {
      js2_R_Speed = 0; // If between boundaries. Do nothing.
    }

    // endregion Deadbands

  }

  @Override
  public void autonomousInit() {
    c_SelectedAuton = c_Auton.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    // System.out.println("Auto selected: " + c_SelectedAuton);
    resetEncoders();
  }

  @Override
  public void autonomousPeriodic() {
    // System.out.println("Robot.autonomousPeriodic()");

    Shuffleboard.startRecording();

    m_frontLeft.enable();
    m_frontRight.enable();
    m_rearLeft.enable();
    m_rearRight.enable();
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
    super.teleopInit();
    // System.out.println("Robot.teleopInit()");

    m_Shooter.setMaxAcceleration(frc3535_Variables.pidMaxAcceleration); // Set max acceleration to 20,000 RPM/s
    m_Shooter.setMaxJerk(frc3535_Variables.pidMaxJerk); // Set max jerk to 31,250 RPM/s^2
    m_Shooter.setPID(frc3535_Variables.pidKp, frc3535_Variables.pidKi, frc3535_Variables.pidKd, frc3535_Variables.pidKf,
        frc3535_Variables.pidKb); // Configure PID gains
    // Kp, Ki, Kd, Kf, b

  }

  @Override
  public void teleopPeriodic() {
    // System.out.println("Robot.teleopPeriodic()");

    Shuffleboard.startRecording();

    m_frontLeft.enable();
    m_frontRight.enable();
    m_rearLeft.enable();
    m_rearRight.enable();

    // System.out.println("Front Left: " + m_frontLeft.getPosition());
    m_rearLeft.follow(m_frontLeft);
    m_rearRight.follow(m_frontRight);

    if (bLimeLightConnected) {
      Vision.setLEDMode(0);
    }

    // region Driver Controller JS1
    // region Driving
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
        // m_drive.arcadeDrive(-js1.getRawAxis(1), js1.getRawAxis(2)); // Arcade Drive
        m_drive.arcadeDrive(-js1_L_Speed, -js1_R_Speed); // Arcade Drive
        break;
    }
    // endregion Driving
    // region Limelight Tracking
    if (js1.getRawButton(frc3535_Variables.btnD_LT)) {
      LimelightBasic();
    }
    // endregion Limelight Tracking
    // region PID Shooter
    if (js1.getRawButtonPressed(frc3535_Variables.btnD_RT)) {
      m_Shooter.enable();
      m_Shooter.setCommand(com.playingwithfusion.CANVenom.ControlMode.SpeedControl, 1500); // Spin the motor at x RPM.
    }
    if (js1.getRawButtonReleased(frc3535_Variables.btnD_RT)) {
      m_Shooter.stopMotor();
    }
    // endregion PID Shooter
    // endregion Driver Controller JS1

    // region Operator Controller JS2

    // region Climber - Up and Down
    // if (js2.getRawButton(frc3535_Variables.btnPXND_R3)) {
    // double xSpeed = 0.0; // make forward stick positive
    // // double xSpeed = js1.getRawAxis(1) * -1; // make forward stick positive
    // // double xSpeed = js1.getRawAxis(1) ; // make forward stick positive

    // xSpeed = -frc3535_Joystick.getDeadBand(2, 1, 1) * -1; // make forward stick
    // positive
    // // CTRE_MotorControls.Climber_Climb(xSpeed);
    // // double xSpeed = js1.getRawAxis(1) ; // make forward stick positive
    // /* update motor controller */

    // }
    // endregion Climber - Up and Down

    // region Climber Position - Left and Right
    // CTRE_MotorControls.Climber_Position(frc3535_Joystick.getDeadBand(2, 1, 2));
    // endregion Climber Position - Left and Right

    // region Intake
    if (js2.getRawButtonPressed(frc3535_Variables.btnPXND_X)) {
      // Intake In
      // CTRE_MotorControls.Intake(frc3535_Variables.intake_in);
      m_Intake.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, frc3535_Variables.intake_in);
    }
    if (js2.getRawButtonReleased(frc3535_Variables.btnPXND_X)) {
      // Intake In
      // CTRE_MotorControls.Intake(0.0);
      m_Intake.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, 0.0);
    }
    if (js2.getRawButtonPressed(frc3535_Variables.btnPXND_A)) {
      // Intake Out
      // CTRE_MotorControls.Intake(frc3535_Variables.intake_out);
      m_Intake.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, frc3535_Variables.intake_out);
    }
    if (js2.getRawButtonReleased(frc3535_Variables.btnPXND_A)) {
      // Intake Out
      // CTRE_MotorControls.Intake(0.0);
      m_Intake.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, 0.0);
    }
    // endregion Intake

    // region Uptake
    if (js2.getRawButtonPressed(frc3535_Variables.btnPXND_Y)) {
      // Uptake Up
      // CTRE_MotorControls.Uptake(frc3535_Variables.uptake_up);
      m_Uptake.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, frc3535_Variables.uptake_up);
    }
    if (js2.getRawButtonReleased(frc3535_Variables.btnPXND_Y)) {
      // Uptake Up
      // CTRE_MotorControls.Uptake(0.0);
      m_Uptake.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, 0.0);
    }
    if (js2.getRawButtonPressed(frc3535_Variables.btnPXND_B)) {
      // Uptake Down
      // CTRE_MotorControls.Uptake(frc3535_Variables.uptake_down);

      m_Uptake.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, frc3535_Variables.uptake_down);
    }
    if (js2.getRawButtonReleased(frc3535_Variables.btnPXND_B)) {
      // Uptake Down
      // CTRE_MotorControls.Uptake(0.0);
      m_Uptake.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, 0.0);
    }
    // endregion Uptake

    // region PID Shooter
    if (js2.getRawButtonPressed(frc3535_Variables.btnPXND_RB)) {
      m_Shooter.enable();
      m_Shooter.setCommand(com.playingwithfusion.CANVenom.ControlMode.SpeedControl, 1500); // Spin the motor at x RPM.
    }
    if (js2.getRawButtonReleased(frc3535_Variables.btnPXND_RB)) {
      m_Shooter.stopMotor();
    }
    // endregion PID Shooter

    // region Limelight Tracking
    if (js2.getRawAxis(frc3535_Variables.axisPXND_R2RT) > 0) {
      LimelightBasic();
    }
    // endregion Limelight Tracking

    // endregion Operator Controller JS2
  }

  @Override
  public void testInit() {
    System.out.println("testInit");
  }

  @Override
  public void testPeriodic() {
    System.out.println("testPeriodic");
  }

  @Override
  public void disabledInit() {
    // System.out.println("Robot.disabledInit()");
    super.disabledInit();
  }

  @Override
  public void disabledPeriodic() {
    // System.out.println("Robot.disabledPeriodic()");
    super.disabledPeriodic();
    Shuffleboard.stopRecording();
    resetEncoders();
    try {
      Vision.setLEDMode(1);
    } catch (Exception name) {
      // System.out.println("Exception: " + name);
      // System.out.println("Error 396 - Unable to connect to the LimeLight");
    }
  }
}