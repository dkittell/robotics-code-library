package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//region Venom Code Imports
import com.playingwithfusion.CANVenom;
import com.playingwithfusion.CANVenom.ControlMode;
//endregion Venom Code Imports

public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  // region Venom
  public com.playingwithfusion.CANVenom m_frontLeft = new CANVenom(2);
  public com.playingwithfusion.CANVenom m_rearLeft = new CANVenom(1);
  public com.playingwithfusion.CANVenom m_frontRight = new CANVenom(4);
  public com.playingwithfusion.CANVenom m_rearRight = new CANVenom(3);
  public com.playingwithfusion.CANVenom m_Shooter = new CANVenom(5);
  // endregion Venom

   

  DifferentialDrive m_drive = new DifferentialDrive(m_frontLeft, m_frontRight);

  // region Controller Definition
  private final Joystick js1 = new Joystick(0);

  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    // CameraServer camera01 = CameraServer.getInstance();
    // camera01.startAutomaticCapture(0);

    m_rearLeft.follow(m_frontLeft);
    m_rearRight.follow(m_frontRight);
  }

  @Override
  public void robotPeriodic() {
    
  }

  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);

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

  @Override
  public void teleopInit() {
    super.teleopInit();
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
    // js1.getRawAxis(1); // Left control up & down
    // js1.getRawAxis(2); // Right control left & right
    m_drive.arcadeDrive(-js1.getRawAxis(1), js1.getRawAxis(2));

    if (js1.getRawButtonPressed(1)) {
      m_frontLeft.setPosition(0);
      m_frontRight.setPosition(0);
      m_rearLeft.setPosition(0);
      m_rearRight.setPosition(0);
      
      // System.out.println(m_frontLeft.getDescription());
    }
    if (js1.getRawButtonPressed(2)) {
      m_frontLeft.resetPosition();
      m_frontRight.resetPosition();
      m_rearLeft.resetPosition();
      m_rearRight.resetPosition();
    }

    // System.out.println("m_frontLeft: " + m_frontLeft.getPosition());
    // System.out.println("m_frontRight: " + m_frontRight.getPosition());
    // System.out.println("m_rearLeft: " + m_rearLeft.getPosition());
    // System.out.println("m_rearRight: " + m_rearRight.getPosition());

    SmartDashboard.putNumber("m_frontLeft", m_frontLeft.getPosition());
    SmartDashboard.putNumber("m_frontRight", m_frontRight.getPosition());
    SmartDashboard.putNumber("m_rearLeft", m_rearLeft.getPosition());
    SmartDashboard.putNumber("m_rearRight", m_rearRight.getPosition());
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void disabledInit() {
    m_frontLeft.resetPosition();
    m_frontRight.resetPosition();
    m_rearLeft.resetPosition();
    m_rearRight.resetPosition();
  }
}
