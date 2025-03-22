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

public class ShooterSubsystem extends SubsystemBase {

  private final CANSparkMax m_shooter1 = new CANSparkMax(
    Constants.ShooterConstants.can_id_Shooter1,
    com.revrobotics.CANSparkLowLevel.MotorType.kBrushless
  );
  private final CANSparkMax m_shooter2 = new CANSparkMax(
    Constants.ShooterConstants.can_id_Shooter2,
    com.revrobotics.CANSparkLowLevel.MotorType.kBrushless
  );

  public ShooterSubsystem() {
    m_shooter1.setInverted(true);
    m_shooter2.setInverted(true);
  }

  @Override
  public void periodic() {}

  public void setShooter1Speed(double Speed) {
    m_shooter1.set(Speed);
  }

  public void setShooter2Speed(double Speed) {
    m_shooter2.set(Speed);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
