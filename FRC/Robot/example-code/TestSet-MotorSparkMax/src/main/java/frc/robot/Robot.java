// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Robot extends TimedRobot {
  private final CANSparkMax m_leftMotor = new CANSparkMax(2, MotorType.kBrushed);

  private final Joystick m_stick = new Joystick(0);

  @Override
  public void robotInit() {

  }

  @Override
  public void teleopPeriodic() {

    m_leftMotor.set(.5);
  }
}
