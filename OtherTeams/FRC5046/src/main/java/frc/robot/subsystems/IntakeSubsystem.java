// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.InTakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */

  private final static CANSparkMax IntakeMotor = new CANSparkMax(InTakeConstants.IntakeMotor, MotorType.kBrushless);
  private final static CANSparkMax Shooterintake = new CANSparkMax(InTakeConstants.shooterintakeMotor,
      MotorType.kBrushless);

  // private final CANSparkMax m_CanSparkMax = new
  // CANSparkMax(DriveConstants.kLeftMotor2Port,MotorType.kBrushless);
  public IntakeSubsystem() {
    IntakeMotor.restoreFactoryDefaults();
    Shooterintake.restoreFactoryDefaults();
    IntakeMotor.setInverted(false);
    Shooterintake.setInverted(true);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  public Command IntakeCommand(double speed) {
    return this.startEnd(() -> intakeManual(speed), () -> intakeManual(0));

  }

  public void intakeManual(double speed) {
    IntakeMotor.set(-speed);
    Shooterintake.set(speed);
  }

  public void intakeManualrev(double speed) {
    IntakeMotor.set(speed);
    Shooterintake.set(-speed);
  }
}
