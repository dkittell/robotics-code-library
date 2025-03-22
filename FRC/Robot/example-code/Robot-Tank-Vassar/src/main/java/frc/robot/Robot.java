// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Vendor Libraries: 
// CTRE - https://maven.ctr-electronics.com/release/com/ctre/phoenix/Phoenix-frc2022-latest.json

package frc.robot;

//import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This is a demo program showing the use of the DifferentialDrive class,
 * specifically it contains
 * the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  PowerDistribution m_pdp = new PowerDistribution();

  // region Controller Assignments
  // Joystick js1;
  // Joystick js2;
  XboxController js1;
  XboxController js2;
  // endregion Controller Assignments

  // region Motor Assignments
  // https://docs.wpilib.org/en/latest/docs/software/hardware-apis/motors/wpi-drive-classes.html#multi-motor-differentialdrive-with-motorcontrollergroups
  MotorController m_frontLeft = new PWMTalonSRX(1);
  MotorController m_rearLeft = new PWMTalonSRX(2);
  MotorControllerGroup m_left = new MotorControllerGroup(m_frontLeft, m_rearLeft);

  MotorController m_frontRight = new PWMTalonSRX(3);
  MotorController m_rearRight = new PWMTalonSRX(4);
  MotorControllerGroup m_right = new MotorControllerGroup(m_frontLeft, m_rearLeft);

  private DifferentialDrive m_Drive = new DifferentialDrive(m_left, m_right);
  // endregion Motor Assignments

  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_frontLeft.setInverted(false);
    m_rearLeft.setInverted(false);
    m_frontRight.setInverted(true);
    m_rearRight.setInverted(true);

    m_Drive = new DifferentialDrive(m_frontLeft, m_frontRight);
    // js1 = new Joystick(0);
    // js2 = new Joystick(1);
    js1 = new XboxController(0);
    js2 = new XboxController(1);
  }

  @Override
  public void robotPeriodic() {

    // region diagnostic
    /*
     * Get the current going through channel 7, in Amperes. The PDP returns the
     * current in increments of 0.125A. At low currents
     * the current readings tend to be less accurate.
     */
    SmartDashboard.putNumber("Front Right Motor", m_pdp.getCurrent(1));
    SmartDashboard.putNumber("Rear Right Motor", m_pdp.getCurrent(0));
    SmartDashboard.putNumber("Rear Left Motor", m_pdp.getCurrent(15));
    SmartDashboard.putNumber("Front Left Motor", m_pdp.getCurrent(14));

    /*
     * Get the voltage going into the PDP, in Volts.
     * The PDP returns the voltage in increments of 0.05 Volts.
     */
    SmartDashboard.putNumber("Voltage", m_pdp.getVoltage());

    /*
     * Retrieves the temperature of the PDP, in degrees Celsius.
     */
    SmartDashboard.putNumber("Temperature", m_pdp.getTemperature());
    // endregion diagnostic

  }

  @Override
  public void autonomousPeriodic() {

    // region Test Motor Direction
    // This area of code should only be used to make sure the motors are all going
    // the correct direction
    // Plug only one motor in at a time and then run auton mode to verify direction.
    m_frontLeft.set(0.5);
    // m_rearLeft.set(0.5);
    // m_frontRight.set(0.5);
    // m_rearRight.set(0.5);
    // endregion Test Motor Direction

    // region Simple Auton Drive Example
    // Only uncomment and run after testing direction of wheels
    // Once you feel good about where the robot is going you can chagne the speed
    // m_Drive.tankDrive(0.4, 0.4);

    // endregion Simple Auton Drive Example

  }

  @Override
  public void teleopPeriodic() {
    // Drive Robot - Tank Drive
    m_Drive.tankDrive(-js1.getLeftY(), -js1.getRightY());
  }
}
