// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

// Rev Robotics - https://software-metadata.revrobotics.com/REVLib-2023.json
// CTRE - https://maven.ctr-electronics.com/release/com/ctre/phoenix/Phoenix-frc2022-latest.json
// Venom - https://www.playingwithfusion.com/frc/playingwithfusion2023.json

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;

public class Robot extends TimedRobot {

  private DifferentialDrive m_myRobot;
  private final Joystick js1 = new Joystick(0);
  // private final VictorSP m_leftFrontMotor = new VictorSP(7);
  // private final VictorSP m_leftRearMotor = new VictorSP(9);
  // private final VictorSP m_rightFrontMotor = new VictorSP(11);
  // private final VictorSP m_rightRearMotor = new VictorSP(12);

  private final WPI_VictorSPX m_frontLeft = new WPI_VictorSPX(7);
  private final WPI_VictorSPX m_rearLeft = new WPI_VictorSPX(9);
  private final WPI_VictorSPX m_frontRight = new WPI_VictorSPX(11);
  private final WPI_VictorSPX m_rearRight = new WPI_VictorSPX(12);

  private final MotorControllerGroup m_left = new MotorControllerGroup(
    m_frontLeft,
    m_rearLeft
  );
  private final MotorControllerGroup m_right = new MotorControllerGroup(
    m_frontRight,
    m_rearRight
  );

  @Override
  public void robotInit() {
    m_frontLeft.setInverted(false);
    m_rearLeft.setInverted(false);
    m_frontRight.setInverted(false);
    m_rearRight.setInverted(false);

    m_myRobot = new DifferentialDrive(m_left, m_right);
  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {}

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    m_myRobot.arcadeDrive(-js1.getRawAxis(1), js1.getRawAxis(2));
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
