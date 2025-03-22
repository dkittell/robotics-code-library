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

import edu.wpi.first.wpilibj.Joystick;
import com.playingwithfusion.CANVenom;
import com.playingwithfusion.CANVenom.ControlMode;
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
  private Joystick m_joystick;
  private CANVenom m_motor;
  private double m_commandedSpeed; 

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    // Create CANVenom instance to control motor ID 1
    m_motor = new CANVenom(1);
    m_joystick = new Joystick(0);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
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
    /** 
     * Place the robot into teleop mode and enable the driverstation.
     * Press the X, Y, A & B buttons to adjsut the motor speed.   Y and A
     * command 4000 RPM in forward and reverse.  X and  B command the 
     * motor to stop.
     */
    if (m_joystick.getRawButtonPressed(4)) { // Y button
      m_commandedSpeed = 4000;
    }
    else if (m_joystick.getRawButtonPressed(1)) { // A button
      m_commandedSpeed = -4000;
    }
    else if (m_joystick.getRawButtonPressed(2) || m_joystick.getRawButtonPressed(3)) {  // X or B button
      m_commandedSpeed = 0;
    }

    // Place Venom into speed control mode and command m_commandedSpeed (in RPM)
    m_motor.setCommand(ControlMode.SpeedControl, m_commandedSpeed);
  }

  @Override
  public void teleopInit() {

    /**
     * Set PID gains and Feed foreward term.  These values work well for an
     * unloaded Venom motor.   P and I gains will likely need to increase 
     * depending on the motor load
     */
    m_motor.setPID(0.195, 0.010, 0.0, 0.184, 0.0);
    
    /**
     * Set max acceleration and jerk limits when responding to step changes
     * in the motor speed command.   This allows Venom to calculate a true
     * s-curve internally
     */
    m_motor.setMaxAcceleration(15000);  // RPM per second
    m_motor.setMaxJerk(15000);          // RPM per second squared

    /**
     * Set the commanded motor speed to 0 RPM each time we enable the driver
     * station
     */
    m_commandedSpeed = 0;
    m_motor.setCommand(ControlMode.SpeedControl, m_commandedSpeed);
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
