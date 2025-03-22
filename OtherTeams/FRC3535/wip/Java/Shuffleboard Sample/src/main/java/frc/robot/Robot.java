/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.MotorSafety;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
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


public class Robot extends TimedRobot {
  // private final DifferentialDrive m_tankDrive = new DifferentialDrive(new
  // PWMVictorSPX(0),
  // new PWMVictorSPX(1));
  // private final Encoder m_leftEncoder = new Encoder(0, 1);
  // private final Encoder m_rightEncoder = new Encoder(2, 3);

  // private final PWMVictorSPX m_elevatorMotor = new PWMVictorSPX(2);

  private final Joystick js1 = new Joystick(0);

  // region Venom
  public com.playingwithfusion.CANVenom m_frontLeft = new CANVenom(2);
  public com.playingwithfusion.CANVenom m_rearLeft = new CANVenom(1);
  public com.playingwithfusion.CANVenom m_frontRight = new CANVenom(4);
  public com.playingwithfusion.CANVenom m_rearRight = new CANVenom(3);
  public com.playingwithfusion.CANVenom m_Shooter = new CANVenom(5);
  // endregion Venom

  SpeedControllerGroup m_left = new SpeedControllerGroup(m_frontLeft, m_rearLeft);
  SpeedControllerGroup m_right = new SpeedControllerGroup(m_frontRight, m_rearRight);
  DifferentialDrive m_drive = new DifferentialDrive(m_left, m_right);

  private final AnalogPotentiometer m_elevatorPot = new AnalogPotentiometer(0);
  private NetworkTableEntry m_maxSpeed;

    // public double j_xAxis = js1.getX() / 2; // Left Toggle Left/Right
    public  double j_yAxis = 0.0; // Left Toggle Up/Down
    public  double j_zAxis = 0.0; // Right Toggle Left/Right

  @Override
  public void robotInit() {

    // Add a 'max speed' widget to a tab named 'Configuration', using a number
    // slider
    // The widget will be placed in the second column and row and will be TWO
    // columns wide
    m_maxSpeed = Shuffleboard.getTab("Configuration").add("Max Speed", 1).withWidget("Number Slider").withPosition(1, 1)
        .withSize(2, 1).getEntry();

    // Add the tank drive and encoders to a 'Drivebase' tab
    ShuffleboardTab driveBaseTab = Shuffleboard.getTab("Configuration");
    driveBaseTab.add("Drive", m_drive);
    // Put both encoders in a list layout
    ShuffleboardLayout encoders = driveBaseTab.getLayout("List Layout", "Encoders").withPosition(0, 0).withSize(2, 2);
    encoders.add("Left Encoder", m_frontLeft.getPosition());
    encoders.add("Right Encoder", m_frontRight.getPosition());

  }

  @Override
  public void robotPeriodic() {
    // TODO Auto-generated method stub
    super.robotPeriodic();

 //   m_drive.setMaxOutput(m_maxSpeed.getDouble(1.0));
  }

  @Override
  public void autonomousInit() {
    // Read the value of the 'max speed' widget from the dashboard

  }

  @Override
  public void teleopPeriodic() {
    // m_drive.arcadeDrive(m_maxSpeed.getDouble(1.0), m_maxSpeed.getDouble(1.0));
    System.out.println("Max Speed: " + m_maxSpeed.getDouble(1.0));
  
    if (js1.getY() > m_maxSpeed.getDouble(1.0)) {
      j_yAxis = m_maxSpeed.getDouble(1.0); // Left Toggle Up/Down
    } else {
      j_yAxis = js1.getY(); // Left Toggle Up/Down
    }

    if (js1.getZ() > m_maxSpeed.getDouble(1.0)) {
      j_zAxis = m_maxSpeed.getDouble(1.0); // Right Toggle Left/Right
    } else {
      j_zAxis = js1.getZ(); // Right Toggle Left/Right
    }


    j_yAxis = -j_yAxis;
    j_zAxis = -j_zAxis;

    m_drive.arcadeDrive(j_yAxis, j_zAxis);

  }

}
