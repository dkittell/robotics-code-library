// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

/**
 * This is a demo program showing the use of the DifferentialDrive class. Runs the motors with
 * arcade steering.
 */
public class Robot extends TimedRobot {
  private final PWMSparkMax m_FrontLeftMotor = new PWMSparkMax(2);
  private final PWMSparkMax m_FrontRightMotor = new PWMSparkMax(4);
  private final PWMSparkMax m_RearLefttMotor = new PWMSparkMax(5);
  private final PWMSparkMax m_RearRightMotor = new PWMSparkMax(6);
  private final PWMSparkMax m_chomp = new PWMSparkMax(0);

  MotorControllerGroup m_left = new MotorControllerGroup(m_FrontLeftMotor, m_RearLefttMotor);
  MotorControllerGroup m_right = new MotorControllerGroup(m_FrontRightMotor, m_RearRightMotor);

  private DifferentialDrive m_Drive = new DifferentialDrive(m_left, m_right);

  private final Joystick js_Driver = new Joystick(0);

  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_FrontRightMotor.setInverted(false);
    m_RearRightMotor.setInverted(false);
    m_FrontLeftMotor.setInverted(true);
    m_RearLefttMotor.setInverted(true);
  }

  @Override
  public void teleopPeriodic() {
    // Drive with arcade drive.
    // That means that the Y axis drives forward
    // and backward, and the X turns left and right.
    m_Drive.arcadeDrive(-js_Driver.getRawAxis(1), js_Driver.getRawAxis(4));

    if (js_Driver.getRawButtonPressed(1)) {
      m_chomp.set(0.2);
    }
  }
}
