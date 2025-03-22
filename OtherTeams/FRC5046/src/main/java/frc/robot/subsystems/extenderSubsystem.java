// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkLimitSwitch.Type;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LiftConstants;
import frc.robot.Constants.extenderConstants;

public class extenderSubsystem extends SubsystemBase {
  /** Creates a new extenderSubsystem. */
  private final static CANSparkMax inOutMotor = new CANSparkMax(extenderConstants.inOutMotor, MotorType.kBrushed);
  // private final static SparkAbsoluteEncoder inoutencoder = inOutMotor
  // .getAbsoluteEncoder(SparkRelativeEncoder.Type.kQuadrature, 4096);
  private RelativeEncoder inoutencoder = inOutMotor.getEncoder(SparkRelativeEncoder.Type.kQuadrature, 4096);
  private final static SparkPIDController inoutPIDController = inOutMotor.getPIDController();
  private final static SparkLimitSwitch m_forwardLimit = inOutMotor.getForwardLimitSwitch(Type.kNormallyOpen);
  private final static SparkLimitSwitch m_reverseLimit = inOutMotor.getReverseLimitSwitch(Type.kNormallyOpen);

  public extenderSubsystem() {

    inOutMotor.restoreFactoryDefaults();
    m_forwardLimit.enableLimitSwitch(true);
    m_reverseLimit.enableLimitSwitch(true);
    inOutMotor.setIdleMode(LiftConstants.ShooterLiftMotorIdleMode);

    inoutPIDController.setFeedbackDevice(inoutencoder);
    inoutencoder.setInverted(false);
    inoutPIDController.setP(extenderConstants.inoutP);
    inoutPIDController.setI(extenderConstants.inoutI);
    inoutPIDController.setD(extenderConstants.inoutD);
    inoutPIDController.setFF(extenderConstants.inoutFF);
    inoutPIDController.setOutputRange(extenderConstants.inoutMinOutput,
        extenderConstants.inoutMaxOutput);

    inOutMotor.setInverted(false);
    inOutMotor.burnFlash();
    inoutencoder.setPosition(0);

  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("LiftPosition", inoutencoder.getPosition());
    // This method will be called once per scheduler run
  }

  public void extenderSetPoints(double rotation) {
    double m_rotation = rotation;
    inoutPIDController.setReference(m_rotation, CANSparkMax.ControlType.kPosition);
  }

  public void ManualinOutMotor(double speed) {

    inOutMotor.set(speed);
  }

  public Command powerlift(double speed) {
    return startEnd(() -> ManualinOutMotor(speed), () -> ManualinOutMotor(0))
        .until(() -> speed > 0 ? m_forwardLimit.isPressed() : m_reverseLimit.isPressed());
  }
}