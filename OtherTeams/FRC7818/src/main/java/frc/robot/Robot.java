package frc.robot;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class Robot extends TimedRobot {

  private final Joystick m_stick = new Joystick(0);
  private CANSparkMax m_leftMotor1;
  private CANSparkMax m_leftMotor2;
  private CANSparkMax m_rightMotor1;
  private CANSparkMax m_rightMotor2;
  private CANSparkMax m_launchy1;
  private CANSparkMax m_launchy2;
  private DifferentialDrive m_robotDrive1;
  private DifferentialDrive m_robotDrive2;
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  //private DigitalInput topLimitSwitch = new DigitalInput(0);
  //private DigitalInput bottomLimitSwitch = new DigitalInput(1);
  @Override
  public void robotInit() {
    //drive motors
    m_leftMotor1 = new CANSparkMax(1, MotorType.kBrushed);
    m_leftMotor2 = new CANSparkMax(2, MotorType.kBrushed);
    m_rightMotor1 = new CANSparkMax(4, MotorType.kBrushed);
    m_rightMotor2 = new CANSparkMax(3, MotorType.kBrushed);
    m_robotDrive1 = new DifferentialDrive(m_leftMotor1, m_rightMotor1);
    m_robotDrive2 = new DifferentialDrive(m_leftMotor2, m_rightMotor2);
    //arm motor
    m_launchy1 = new CANSparkMax(6, MotorType.kBrushed);
    m_launchy2 = new CANSparkMax(5, MotorType.kBrushed);
    //camera (gay)
    CameraServer.startAutomaticCapture();
    //anonymous
    m_chooser.setDefaultOption("Drive By", "Drive By");
    m_chooser.addOption("Code 2", "Code 2");
    m_chooser.addOption("Nothing", "Nothing");
    SmartDashboard.putData("Autonomous mode choices", m_chooser);
  }

  public void shootThingy() {
    drive(0, 0); //make sure robot is parked
    m_launchy1.set(-Double.MAX_VALUE);
    Timer.delay(0.6);
    m_launchy2.set(-Double.MAX_VALUE);
    Timer.delay(0.6);
    m_launchy1.set(0);
    m_launchy2.set(0);
  }

  public void intimidate() {
    m_launchy1.set(-Double.MAX_VALUE);
    Timer.delay(0.12);
    m_launchy1.set(-0.7);
    Timer.delay(0.12);
  }

  //buttons
  /* 1: a
   * 2: b
   * 3: x
   * 4: y
   * 5: LB
   * 6: RB
   */
  @Override
  public void teleopPeriodic() {
    // Drive with arcade drive.
    // That means that the Y axis drives forward
    // and backward, and the X turns left and right.
    drive(m_stick.getRawAxis(1), m_stick.getRawAxis(4));
    if (m_stick.getRawAxis(2) - m_stick.getRawAxis(3) < -0.1) {
      shootThingy();
    } else if (m_stick.getRawAxis(2) - m_stick.getRawAxis(3) > 0.1) {
      m_launchy1.set(.65);
      m_launchy2.set(.65);
    } else if (m_stick.getRawButton(6)) {
      m_launchy1.set(-.65);
      m_launchy2.set(-.65);
    } else if (m_stick.getRawButton(5)) {
      //intimidate();
    } else {
      m_launchy1.set(0);
      m_launchy2.set(0);
    }

    if (m_stick.getRawButton(1)) {
      drive(-0.5, 0);
    } else if (m_stick.getRawButton(2)) {
      drive(0, 0.5);
    } else if (m_stick.getRawButton(3)) {
      drive(0, -0.5);
    } else if (m_stick.getRawButton(4)) {
      drive(0.5, 0);
    }
  }

  public void drive(double a, double b) {
    m_robotDrive1.arcadeDrive(a, b);
    m_robotDrive2.arcadeDrive(a, b);
  }

  @Override
  public void autonomousInit() {
    m_robotDrive1.setSafetyEnabled(false);
    m_robotDrive2.setSafetyEnabled(false);
    m_autoSelected = m_chooser.getSelected();
    System.out.println("Auto selected: " + m_autoSelected);
    switch (m_autoSelected) {
      case "Drive By":
        shootThingy();
        drive(0.5, 0);
        Timer.delay(3);
        drive(0, 0);
        break;
      case "Code 2":
        drive(0.25, 0);
        break;
      default:
        //don't do shit
        break;
    }
  }

  @Override
  public void autonomousPeriodic() {
    //no
  }
}
