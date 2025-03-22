// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
// CTRE - https://maven.ctr-electronics.com/release/com/ctre/phoenix/Phoenix-frc2024-latest.json
// Rev Robotics - https://software-metadata.revrobotics.com/REVLib-2024.json
// Venom - https://www.playingwithfusion.com/frc/playingwithfusion2024.json
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private static final String kDefaultAuto = "Default";
  private static final String kLineDriveAuto = "Drive Past Init Line";
  private String m_autoSelected;
  private final SendableChooser<String> c_Auton = new SendableChooser<>();
  private final SendableChooser<String> c_Auton_Delay = new SendableChooser<>();
  private final SendableChooser<String> c_DriveYSpeed = new SendableChooser<>();
  private final SendableChooser<String> c_DriveZASpeed = new SendableChooser<>();
  private final SendableChooser<String> c_DriveXSpeed = new SendableChooser<>();

  private DifferentialDrive m_robotDrive;
  private Joystick js_Driver;
  private Joystick js_Operator;

  private final CANSparkMax m_leftMotor = new CANSparkMax(
    constants.can_id_frontLeft,
    MotorType.kBrushed
  );
  private final CANSparkMax m_rightMotor = new CANSparkMax(
    constants.can_id_frontRight,
    MotorType.kBrushed
  );

  private final CANSparkMax m_Intake = new CANSparkMax(
    constants.can_id_intake,
    MotorType.kBrushed
  );
  private final CANSparkMax m_Shooter1 = new CANSparkMax(
    constants.can_id_Shooter1,
    MotorType.kBrushed
  );
  private final CANSparkMax m_Shooter2 = new CANSparkMax(
    constants.can_id_Shooter2,
    MotorType.kBrushed
  );

  public CANSparkMax m_Conveyor = new CANSparkMax(
    constants.can_id_Conveyor,
    MotorType.kBrushless
  );
  PowerDistribution m_pdh = new PowerDistribution(
    constants.can_id_pdh,
    ModuleType.kRev
  );

  private String c_SelectedDriveYSpeed;
  private String c_SelectedDriveZASpeed;
  private String c_SelectedDriveXSpeed;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // region SmartDashboard

    c_Auton.setDefaultOption(kDefaultAuto, kDefaultAuto);
    c_Auton.addOption(kLineDriveAuto, kLineDriveAuto);

    c_Auton_Delay.setDefaultOption("None", "None");
    c_Auton_Delay.addOption("1 second", "1");
    c_Auton_Delay.addOption("2 second", "2");
    c_Auton_Delay.addOption("3 second", "3");
    c_Auton_Delay.addOption("4 second", "4");
    c_Auton_Delay.addOption("5 second", "5");
    c_Auton_Delay.addOption("6 second", "6");

    c_DriveYSpeed.addOption(".40", ".40");
    c_DriveYSpeed.addOption(".45", ".45");
    c_DriveYSpeed.addOption(".50", ".50");
    c_DriveYSpeed.addOption(".55", ".55");
    c_DriveYSpeed.addOption(".60", ".60");
    c_DriveYSpeed.addOption(".65", ".65");
    c_DriveYSpeed.addOption(".70", ".70");
    c_DriveYSpeed.addOption(".75", ".75");
    c_DriveYSpeed.setDefaultOption(".80", ".80");
    c_DriveYSpeed.addOption(".85", ".85");
    c_DriveYSpeed.addOption(".90", ".90");

    c_DriveZASpeed.addOption(".40", ".40");
    c_DriveZASpeed.addOption(".45", ".45");
    c_DriveZASpeed.addOption(".50", ".50");
    c_DriveZASpeed.addOption(".55", ".55");
    c_DriveZASpeed.setDefaultOption(".60", ".60");
    c_DriveZASpeed.addOption(".65", ".65");
    c_DriveZASpeed.addOption(".70", ".70");
    c_DriveZASpeed.addOption(".75", ".75");
    c_DriveZASpeed.addOption(".80", ".80");
    c_DriveZASpeed.addOption(".85", ".85");
    c_DriveZASpeed.addOption(".90", ".90");

    c_DriveXSpeed.addOption(".40", ".40");
    c_DriveXSpeed.addOption(".45", ".45");
    c_DriveXSpeed.addOption(".50", ".50");
    c_DriveXSpeed.addOption(".55", ".55");
    c_DriveXSpeed.setDefaultOption(".60", ".60");
    c_DriveXSpeed.addOption(".65", ".65");
    c_DriveXSpeed.addOption(".70", ".70");
    c_DriveXSpeed.addOption(".75", ".75");
    c_DriveXSpeed.addOption(".80", ".80");
    c_DriveXSpeed.addOption(".85", ".85");
    c_DriveXSpeed.addOption(".90", ".90");

    SmartDashboard.putData("Auto choices", c_Auton);

    SendableRegistry.addChild(m_robotDrive, m_leftMotor);
    SendableRegistry.addChild(m_robotDrive, m_rightMotor);

    // endregion SmartDashboard

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotor.setInverted(true);

    m_robotDrive = new DifferentialDrive(m_leftMotor::set, m_rightMotor::set);
    js_Driver = new Joystick(constants.js_Driver);
    js_Operator = new Joystick(constants.js_Operator);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    SmartDashboard.putData("Auton choices", c_Auton);
    SmartDashboard.putData("Auto Delay", c_Auton_Delay);
    SmartDashboard.putData("Drive Speed", c_DriveYSpeed);
    SmartDashboard.putData("Turning Speed", c_DriveZASpeed);
    SmartDashboard.putData("Strafing Speed", c_DriveXSpeed);

    boolean bBattery;
    if (m_pdh.getVoltage() >= 12) {
      bBattery = true;
    } else {
      bBattery = false;
    }
    SmartDashboard.putBoolean("Battery", bBattery);
    SmartDashboard.putNumber("Voltage", m_pdh.getVoltage());
    SmartDashboard.putBoolean("Enabled", isEnabled());

    c_SelectedDriveYSpeed = c_DriveYSpeed.getSelected();
    c_SelectedDriveZASpeed = c_DriveZASpeed.getSelected();
    c_SelectedDriveXSpeed = c_DriveXSpeed.getSelected();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = c_Auton.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kLineDriveAuto:
        // Drive robot past initialization Line

        break;
      case kDefaultAuto:
      default:
        // Do Nothing, Sit Still and look pretty0
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // Get joystick values and scale
    double speed_lUpDown =
      js_Driver.getRawAxis(constants.axisXB_lUpDown) *
      Double.parseDouble(c_SelectedDriveYSpeed);
    double speed_lLeftRight =
      js_Driver.getRawAxis(constants.axisXB_lLeftRight) *
      Double.parseDouble(c_SelectedDriveXSpeed);
    double speed_rLeftRight =
      js_Driver.getRawAxis(constants.axisXB_rLeftRight) *
      Double.parseDouble(c_SelectedDriveZASpeed);

    // m_robotDrive.tankDrive(js_Driver.getRawAxis(1), js_Driver.getRawAxis(2));
    m_robotDrive.arcadeDrive(speed_lUpDown, speed_rLeftRight);

    if (js_Operator.getRawButton(constants.btnD_RB)) {
      // Bring note to conveyor
      m_Intake.set(0.7);
    } else if (js_Operator.getRawButton(constants.btnD_RT)) {
      // Kick the note out of robot
      m_Intake.set(-0.7);
    } else {
      m_Intake.set(0);
    }

    if (js_Operator.getRawButton(constants.btnD_A)) {
      // Kick the note back to the conveyor
      m_Shooter1.set(1);
      m_Shooter2.set(1);
    } else if (js_Operator.getRawButton(constants.btnD_X)) {
      // Shoot the note
      m_Shooter1.set(-1);
      m_Shooter2.set(-1);
    } else {
      m_Shooter1.set(0);
      m_Shooter2.set(0);
    }

    if (js_Operator.getRawButton(constants.btnD_LB)) {
      // Bring note to shooter
      m_Conveyor.set(1);
    } else if (js_Operator.getRawButton(constants.btnD_LT)) {
      // Kick the note back to intake
      m_Conveyor.set(-1);
    } else {
      m_Conveyor.set(0);
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
