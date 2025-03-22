// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private final PWMSparkMax m_leftBack = new PWMSparkMax(1);
  private final PWMSparkMax m_leftFront = new PWMSparkMax(3);
  private final PWMSparkMax m_rightBack = new PWMSparkMax(4);
  private final PWMSparkMax m_rightFront = new PWMSparkMax(2);

  private final DifferentialDrive m_robotDrive = new DifferentialDrive(
    m_leftFront::set,
    m_rightFront::set
  );
  private final Joystick js_Driver = new Joystick(0);

  // Solenoid corresponds to a single solenoid.
  // In this case, it's connected to channel 0 of a PH with the default CAN ID.
  private final Solenoid m_solenoid = new Solenoid(
    PneumaticsModuleType.REVPH,
    0
  );

  // DoubleSolenoid corresponds to a double solenoid.
  // In this case, it's connected to channels 1 and 2 of a PH with the default CAN ID.
  private final DoubleSolenoid m_doubleSolenoid = new DoubleSolenoid(
    PneumaticsModuleType.REVPH,
    1,
    2
  );

  // Compressor connected to a PH with a default CAN ID (1)
  private final Compressor m_compressor = new Compressor(
    PneumaticsModuleType.REVPH
  );

  static final int kSolenoidButton = 1;
  static final int kDoubleSolenoidForwardButton = 2;
  static final int kDoubleSolenoidReverseButton = 3;
  static final int kCompressorButton = 4;

  public Robot() {
    SendableRegistry.addChild(m_robotDrive, m_leftFront);
    SendableRegistry.addChild(m_robotDrive, m_rightFront);
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    m_leftBack.addFollower(m_leftFront);
    m_rightBack.addFollower(m_rightFront);

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightFront.setInverted(true);

    // Publish elements to shuffleboard.
    ShuffleboardTab tab = Shuffleboard.getTab("Pneumatics");
    tab.add("Single Solenoid", m_solenoid);
    tab.add("Double Solenoid", m_doubleSolenoid);
    tab.add("Compressor", m_compressor);

    // Also publish some raw data
    // Get the pressure (in PSI) from the analog sensor connected to the PH.
    // This function is supported only on the PH!
    // On a PCM, this function will return 0.
    tab.addDouble("PH Pressure [PSI]", m_compressor::getPressure);
    // Get compressor current draw.
    tab.addDouble("Compressor Current", m_compressor::getCurrent);
    // Get whether the compressor is active.
    tab.addBoolean("Compressor Active", m_compressor::isEnabled);
    // Get the digital pressure switch connected to the PCM/PH.
    // The switch is open when the pressure is over ~120 PSI.
    tab.addBoolean("Pressure Switch", m_compressor::getPressureSwitchValue);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

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
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @SuppressWarnings("PMD.UnconditionalIfStatement")
  @Override
  public void teleopPeriodic() {
    // Drive with arcade drive.
    // That means that the Y axis drives forward
    // and backward, and the X turns left and right.
    m_robotDrive.arcadeDrive(-js_Driver.getY(), -js_Driver.getX());

    /*
     * The output of GetRawButton is true/false depending on whether
     * the button is pressed; Set takes a boolean for whether
     * to retract the solenoid (false) or extend it (true).
     */
    m_solenoid.set(js_Driver.getRawButton(kSolenoidButton));

    /*
     * GetRawButtonPressed will only return true once per press.
     * If a button is pressed, set the solenoid to the respective channel.
     */
    if (js_Driver.getRawButtonPressed(kDoubleSolenoidForwardButton)) {
      m_doubleSolenoid.set(DoubleSolenoid.Value.kForward);
    } else if (js_Driver.getRawButtonPressed(kDoubleSolenoidReverseButton)) {
      m_doubleSolenoid.set(DoubleSolenoid.Value.kReverse);
    }

    // On button press, toggle the compressor.
    if (js_Driver.getRawButtonPressed(kCompressorButton)) {
      // Check whether the compressor is currently enabled.
      boolean isCompressorEnabled = m_compressor.isEnabled();
      if (isCompressorEnabled) {
        // Disable closed-loop mode on the compressor.
        m_compressor.disable();
      } else {
        // Change the if statements to select the closed-loop you want to use:
        if (false) {
          // Enable closed-loop mode based on the digital pressure switch connected to the PCM/PH.
          // The switch is open when the pressure is over ~120 PSI.
          m_compressor.enableDigital();
        }
        if (true) {
          // Enable closed-loop mode based on the analog pressure sensor connected to the PH.
          // The compressor will run while the pressure reported by the sensor is in the
          // specified range ([70 PSI, 120 PSI] in this example).
          // Analog mode exists only on the PH! On the PCM, this enables digital control.
          m_compressor.enableAnalog(70, 120);
        }
        if (false) {
          // Enable closed-loop mode based on both the digital pressure switch AND the analog
          // pressure sensor connected to the PH.
          // The compressor will run while the pressure reported by the analog sensor is in the
          // specified range ([70 PSI, 120 PSI] in this example) AND the digital switch reports
          // that the system is not full.
          // Hybrid mode exists only on the PH! On the PCM, this enables digital control.
          m_compressor.enableHybrid(70, 120);
        }
      }
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
