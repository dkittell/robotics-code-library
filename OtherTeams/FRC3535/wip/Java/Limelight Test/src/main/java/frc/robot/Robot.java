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
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
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

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {

    // Limelight Data Start
    // get the default instance of NetworkTables
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    // get a reference to the subtable called "datatable"
    NetworkTable table = inst.getTable("limelight");

    // inst.startClientTeam();

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

    //ledModeEntry.setNumber(0); // use the LED Mode set in the current pipeline
    ledModeEntry.setNumber(1); // force off
    //ledModeEntry.setNumber(2); // force blink
    //ledModeEntry.setNumber(3); // force on

    System.out.println("X: " + tx);
    System.out.println("Y: " + ty);
    System.out.println("A: " + ta);
    System.out.println("L: " + tl);
    System.out.println("V: " + tv);
    System.out.println("S: " + tv);

    // post to smart dashboard periodically
    SmartDashboard.putNumber("Limelight X", tx);
    SmartDashboard.putNumber("Limelight Y", ty);
    SmartDashboard.putNumber("Limelight Area", ta);
    SmartDashboard.putNumber("Limelight Latency", tl);
    SmartDashboard.putNumber("Limelight Valid Target", tv);
    SmartDashboard.putNumber("Limelight Skew", ts);

    // Limelight Data End

  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
