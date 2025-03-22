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

public class PivotSubsystem extends SubsystemBase {

  private final CANSparkMax m_pivot = new CANSparkMax(
    Constants.ShooterConstants.can_id_Pivot,
    com.revrobotics.CANSparkLowLevel.MotorType.kBrushless
  );

  RelativeEncoder m_pivotEncoder;

  private final double kEncoderTick2Meter = 1.0 / 4096.0 * 0.1 * Math.PI;

  public double getEncoderMeters() {
    return m_pivotEncoder.getPosition() * kEncoderTick2Meter;
  }

  public PivotSubsystem() {
    m_pivot.setIdleMode(IdleMode.kBrake);
    m_pivot.setInverted(true);
    m_pivotEncoder = m_pivot.getEncoder();
    m_pivotEncoder.setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber(
      "Pivot encoder value",
      m_pivotEncoder.getPosition()
    );
  }

  public void setPivotMotor(double Speed) {
    m_pivot.set(Speed);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
