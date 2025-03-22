/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

//region Imports
import frc.robot.frc3535_Variables;
import edu.wpi.first.wpilibj.Joystick;
// import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.MotorSafety;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.net.InetAddress;
import java.net.NetworkInterface;
import java.net.SocketException;
import java.net.UnknownHostException;

// region Limelight
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
// endregion Limelight

//region Venom Code Imports
import com.playingwithfusion.CANVenom;
import com.playingwithfusion.CANVenom.ControlMode;
//endregion Venom Code Imports
//endregion Imports

/**
 * This is a demo program showing the use of the DifferentialDrive class. Runs
 * the motors with arcade steering.
 */
public class Robot extends TimedRobot {
  // private final PWMVictorSPX m_leftMotor = new PWMVictorSPX(0);
  // private final PWMVictorSPX m_rightMotor = new PWMVictorSPX(1);
  // m_rearLeft.Follow(m_frontLeft);
  // m_rearRight.Follow(m_frontRight);

  private static final int kPDPId = 0;

  private final PowerDistributionPanel m_pdp = new PowerDistributionPanel(kPDPId);

  // region Venom

  public com.playingwithfusion.CANVenom m_frontLeft = new CANVenom(frc3535_Variables.m_frontLeft);
  public com.playingwithfusion.CANVenom m_rearLeft = new CANVenom(frc3535_Variables.m_rearLeft);
  public com.playingwithfusion.CANVenom m_frontRight = new CANVenom(frc3535_Variables.m_frontRight);
  public com.playingwithfusion.CANVenom m_rearRight = new CANVenom(frc3535_Variables.m_rearRight);
  public com.playingwithfusion.CANVenom m_Shooter = new CANVenom(frc3535_Variables.m_Shooter);

  /**
   * Reset the drive encoders to 0
   */
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

  DifferentialDrive m_drive = new DifferentialDrive(m_frontLeft, m_frontRight);

  // region Controller Definition
  private final Joystick js1 = new Joystick(frc3535_Variables.js1);
  private final Joystick js2 = new Joystick(frc3535_Variables.js2);
  // endregion Controller Definition

  double tx = 0.0;
  double ty = 0.0;
  double ta = 0.0;
  double tl = 0.0;
  double tv = 0.0;
  double ts = 0.0;
  double left_command = 0.0;
  double right_command = 0.0;
  double drivingAdjust = 0.0f;
  double steeringAdjust = 0.0f;

  double Kp = -0.1f;
  double min_command = 0.05f;
  double heading_error = tx;
  double steering_adjust = 0.0f;
  double KpAim = -0.1f;
  double KpDist = 0.0f; // 0.09;
  double AimMinCmd = 0.095f;
  double ll_steer = 0.0;
  double ll_drive = 0.0;
  // double KpDistance = -0.1f;
  // double min_aim_command = 0.05f;
  boolean bValidTarget = false;

  public String mac;
  public static boolean isComp = false;

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

  @Override
  public void robotInit() {

    m_rearLeft.follow(m_frontLeft);
    m_rearRight.follow(m_frontRight);

    mac = "xx:xx:xx:xx:xx:xx";
    // Attempt to get the MAC address of the robot
    try {
      NetworkInterface network = NetworkInterface.getByInetAddress(InetAddress.getLocalHost());

      byte[] address = network.getHardwareAddress();

      StringBuilder sb = new StringBuilder();
      for (int i = 0; i < address.length; i++) {
        sb.append(String.format("%02X%s", address[i], (i < address.length - 1) ? ":" : ""));
      }
      mac = sb.toString();
      // System.out.println(mac);
    } catch (UnknownHostException e) {
      System.out.println("Unknown Host Exception - " + e);
    } catch (SocketException e) {
      System.out.println("Socket Exception - " + e);
    }
    /// Determines what robot we are using

    if (mac.equals("00:80:2F:17:BD:5F")) {
      System.out.println("2020 Competition " + mac);
      isComp = true;
    } else {

      System.out.println("Practice " + mac);
      isComp = false;
    }

    isComp = true;
  }

  @Override
  public void robotPeriodic() {
    // TODO Auto-generated method stub
    super.robotPeriodic();

    SmartDashboard.putNumber("Shooter_Speed_Graph", m_Shooter.getSpeed());
    SmartDashboard.putNumber("Shooter_Speed", m_Shooter.getSpeed());

    SmartDashboard.putNumber("m_frontLeft", m_frontLeft.getPosition());
    SmartDashboard.putNumber("m_frontRight", m_frontRight.getPosition());
    SmartDashboard.putNumber("m_rearLeft", m_rearLeft.getPosition());
    SmartDashboard.putNumber("m_rearRight", m_rearRight.getPosition());

    // region PDP Stats
    // /*
    // * Get the current going through channel 7, in Amperes. The PDP returns the
    // * current in increments of 0.125A. At low currents the current readings tend
    // to
    // * be less accurate.
    // */
    // SmartDashboard.putNumber("PDP Current Channel 7", m_pdp.getCurrent(7));

    // /*
    // * Get the voltage going into the PDP, in Volts. The PDP returns the voltage
    // in
    // * increments of 0.05 Volts.
    // */
    // SmartDashboard.putNumber("PDP Voltage", m_pdp.getVoltage());

    // /*
    // * Retrieves the temperature of the PDP, in degrees Celsius.
    // */
    // SmartDashboard.putNumber("PDP Temperature", m_pdp.getTemperature());
    // endregion PDP Stats

    // region Limelight Data Start
    // // get the default instance of NetworkTables
    // NetworkTableInstance inst = NetworkTableInstance.getDefault();
    // // get a reference to the subtable called "datatable"
    // NetworkTable table = inst.getTable("limelight");

    // inst.startClientTeam(3535);

    // inst.startDSClient(); // recommended if running on DS computer; this gets the
    // robot IP from the DS

    // // NetworkTableEntry TeamEntry = table.getEntry("tx");
    // NetworkTableEntry xEntry = table.getEntry("tx");
    // NetworkTableEntry yEntry = table.getEntry("ty");
    // NetworkTableEntry aEntry = table.getEntry("ta");
    // NetworkTableEntry lEntry = table.getEntry("tl");
    // NetworkTableEntry vEntry = table.getEntry("tv");
    // NetworkTableEntry sEntry = table.getEntry("ts");

    // // NetworkTableEntry tshortEntry = table.getEntry("tshort");
    // // NetworkTableEntry tlongEntry = table.getEntry("tlong");
    // // NetworkTableEntry thorEntry = table.getEntry("thor");
    // // NetworkTableEntry tvertEntry = table.getEntry("tvert");
    // // NetworkTableEntry getpipeEntry = table.getEntry("getpipe");
    // // NetworkTableEntry camtranEntry = table.getEntry("camtran");
    // NetworkTableEntry ledModeEntry = table.getEntry("ledMode");

    // // double tx = xEntry.getDouble(0.0);
    // tx = xEntry.getDouble(0.0); // Horizontal Offset From Crosshair To Target
    // (-27 degrees to 27 degrees)
    // ty = yEntry.getDouble(0.0); // Vertical Offset From Crosshair To Target
    // (-20.5 degrees to 20.5 degrees)
    // ta = aEntry.getDouble(0.0); // Target Area (0% of image to 100% of image)
    // tl = lEntry.getDouble(0.0); // The pipeline’s latency contribution (ms) Add
    // at least 11ms for image capture
    // // latency.
    // tv = vEntry.getDouble(0.0); // Whether the limelight has any valid targets (0
    // or 1)
    // ts = sEntry.getDouble(0.0); // Skew or rotation (-90 degrees to 0 degrees)

    // // double tshort = tshortEntry.getString(); // Sidelength of shortest side of
    // // the fitted bounding box (pixels)
    // // double tlong = tlong // Sidelength of longest side of the fitted bounding
    // box
    // // (pixels)
    // // double thor = thor // Horizontal sidelength of the rough bounding box (0 -
    // // 320 pixels)
    // // double tvert = tvert // Vertical sidelength of the rough bounding box (0 -
    // // 320 pixels)
    // // double getpipe = getpipe // True active pipeline index of the camera (0 ..
    // 9)
    // // double camtran = camtran // Results of a 3D position solution, 6 numbers:
    // // Translation (x,y,y) Rotation(pitch,yaw,roll)

    // ledModeEntry.setNumber(0); // use the LED Mode set in the current pipeline
    // // ledModeEntry.setNumber(1); // force off
    // // ledModeEntry.setNumber(2); // force blink
    // // ledModeEntry.setNumber(3); // force on

    // // System.out.println("X: " + tx);
    // // System.out.println("Y: " + ty);
    // // System.out.println("A: " + ta);
    // // System.out.println("L: " + tl);
    // // System.out.println("V: " + tv);
    // // System.out.println("S: " + tv);

    // if (tv == 1.0f) {
    // bValidTarget = true;
    // }

    // endregion Limelight Data Start

    // // double j_xAxis = js1.getX(); // Left Toggle Left/Right
    // // double j_yAxis = js1.getY(); // Left Toggle Up/Down
    // // double j_zAxis = js1.getZ(); // Right Toggle Left/Right

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

    SmartDashboard.putNumber("js1_L_Actual", js1.getY());
    SmartDashboard.putNumber("js1_R_Actual", js1.getZ());

    SmartDashboard.putNumber("js1_L_Speed", js1_L_Speed);
    SmartDashboard.putNumber("js1_R_Speed", js1_R_Speed);

    // region SmartDashboard Data
    SmartDashboard.putNumber("Battery_Voltage", edu.wpi.first.wpilibj.RobotController.getBatteryVoltage());

    SmartDashboard.putString("Shooter_Temperature", m_Shooter.getTemperature() + " C");
    SmartDashboard.putNumber("Shooter_Output_Current ", m_Shooter.getOutputCurrent());
    SmartDashboard.putNumber("Shooter_Output_Voltage ", m_Shooter.getOutputVoltage());

    // endregion SmartDashboard Data

  }

  @Override
  public void teleopInit() {
    m_Shooter.setMaxAcceleration(20000); // Set max acceleration to 20,000 RPM/s
    m_Shooter.setMaxJerk(31250); // Set max jerk to 31,250 RPM/s^2
    m_Shooter.setPID(0.195, 0.010, 0.0, 0.184, 0.0); // Configure PID gains
    // m_Shooter.setPID(0.215, 0.010, 0.055, 0.184, 0.0); // Configure PID gains
    // Kp, Ki, Kd, Kf, b
    // Kp lower shoots lower

    m_frontLeft.setPosition(0);
    m_frontRight.setPosition(0);
    m_rearLeft.setPosition(0);
    m_rearRight.setPosition(0);
    m_frontLeft.resetPosition();
    m_frontRight.resetPosition();
    m_rearLeft.resetPosition();
    m_rearRight.resetPosition();

  }

  @Override
  public void teleopPeriodic() {

    // region Drive with arcade drive.
    // That means that the Y axis drives forward
    // and backward, and the X turns left and right.

    m_drive.arcadeDrive(-js1_L_Speed, -js1_R_Speed); // Regular
                                                     // Controller
    // m_drive.arcadeDrive(js2.getRawAxis(1), js2.getRawAxis(0)); // PXN Controller

    // // endregion Drive with arcade drive.

    // region PID Shooter
    if (js1.getRawButtonPressed(frc3535_Variables.btnD_A)) {
      m_Shooter.enable();
      m_Shooter.setCommand(com.playingwithfusion.CANVenom.ControlMode.SpeedControl, 1500); // Spin the motor at x RPM.
      SmartDashboard.putNumber("Shooter_Speed_Graph", m_Shooter.getSpeed());
      SmartDashboard.putNumber("Shooter_Speed", m_Shooter.getSpeed());
    }
    if (js1.getRawButtonReleased(frc3535_Variables.btnD_A)) {
      m_Shooter.stopMotor();
    }
    // endegion PID Shooter

    // if (js1.getRawButtonPressed(btnD_X)) {
    // // m_frontLeft.setPosition(0);
    // // m_frontRight.setPosition(0);
    // // m_rearLeft.setPosition(0);
    // // m_rearRight.setPosition(0);

    // m_frontLeft.resetPosition();
    // m_frontRight.resetPosition();
    // m_rearLeft.resetPosition();
    // m_rearRight.resetPosition();
    // }

  }

  @Override
  public void testInit() {

  }

  @Override
  public void testPeriodic() {
    System.out.println("testPeriodic");

  }

  @Override
  public void disabledInit() {

  }

  @Override
  public void disabledPeriodic() {

    m_frontLeft.resetPosition();
    m_frontRight.resetPosition();
    m_rearLeft.resetPosition();
    m_rearRight.resetPosition();

    m_frontLeft.setPosition(0);
    m_frontRight.setPosition(0);
    m_rearLeft.setPosition(0);
    m_rearRight.setPosition(0);
  }

}
