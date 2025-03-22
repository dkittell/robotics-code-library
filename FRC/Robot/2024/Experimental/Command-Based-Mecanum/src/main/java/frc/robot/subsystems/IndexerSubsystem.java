// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IndexerSubsystem extends SubsystemBase {

  private final CANSparkMax m_indexer = new CANSparkMax(
    Constants.ShooterConstants.can_id_indexer,
    com.revrobotics.CANSparkLowLevel.MotorType.kBrushless
  );

  // endregion Shooting System

  public IndexerSubsystem() {
    m_indexer.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  public void setIndexerMotor(double Speed) {
    m_indexer.set(Speed);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
