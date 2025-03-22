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
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.MotorSafety;
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
 * This is a sample program to demonstrate how to use a gyro sensor to make a
 * robot drive straight. This program uses a joystick to drive forwards and
 * backwards while the gyro is used for direction keeping.
 */
public class Robot extends TimedRobot {

   // region Venom
   public com.playingwithfusion.CANVenom m_frontLeft = new CANVenom(2);
   public com.playingwithfusion.CANVenom m_rearLeft = new CANVenom(1);
   public com.playingwithfusion.CANVenom m_frontRight = new CANVenom(4);
   public com.playingwithfusion.CANVenom m_rearRight = new CANVenom(3);
   // endregion Venom
 
   SpeedControllerGroup m_left = new SpeedControllerGroup(m_frontLeft, m_rearLeft);
   SpeedControllerGroup m_right = new SpeedControllerGroup(m_frontRight, m_rearRight);
   DifferentialDrive m_drive = new DifferentialDrive(m_left, m_right);
 
   // region Controller Definition
   private final Joystick js1 = new Joystick(0);
  //  private final Joystick js2 = new Joystick(1);

  // region Gyro
  private static final int kGyroPort = 0;

  private final AnalogGyro m_gyro = new AnalogGyro(kGyroPort);
  private static final double kAngleSetpoint = 0.0;
  private static final double kP = 0.005; // propotional turning constant

  // gyro calibration constant, may need to be adjusted;
  // gyro value of 360 is set to correspond to one full revolution
  private static final double kVoltsPerDegreePerSecond = 0.0128;
    // endregion Gyro

  public String mac;
  public static boolean isComp = false;

  

  @Override
  public void robotInit() {
    m_gyro.setSensitivity(kVoltsPerDegreePerSecond);

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

  /**
   * The motor speed is set from the joystick while the RobotDrive turning
   * value is assigned from the error between the setpoint and the gyro angle.
   */
  @Override
  public void teleopPeriodic() {

    double turningValue = (kAngleSetpoint - m_gyro.getAngle()) * kP;
    // Invert the direction of the turn if we are going backwards
    turningValue = Math.copySign(turningValue, js1.getY());
   
    if (js1.getRawButton(1))
    {
      m_drive.arcadeDrive(js1.getY(), turningValue);
    }
   else 
    {
      m_drive.arcadeDrive(js1.getY(), js1.getZ());
    }


    // System.out.println("turningValue " + turningValue);
    SmartDashboard.putNumber("Gyro Turning Point", turningValue);

  }
}
