// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkLimitSwitch.Type;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.LiftConstants;
import frc.robot.commands.LiftPid;
import frc.robot.commands.LiftPid.LiftPidPositions;

public class LiftMotorSubsysystem extends SubsystemBase {
  /** Creates a new LiftMotorSubsysystem. */
  private final static CANSparkMax m_shooterLiftMotor = new CANSparkMax(LiftConstants.ShooterLiftMotor,
      MotorType.kBrushless);
  private final static SparkAbsoluteEncoder liftencoder = m_shooterLiftMotor
      .getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
  private final static SparkPIDController liftPIDController = m_shooterLiftMotor.getPIDController();
  private final static SparkLimitSwitch m_forwardLimit = m_shooterLiftMotor.getForwardLimitSwitch(Type.kNormallyOpen);
  private final static SparkLimitSwitch m_reverseLimit = m_shooterLiftMotor.getReverseLimitSwitch(Type.kNormallyOpen);

  public LiftMotorSubsysystem() {
    m_shooterLiftMotor.restoreFactoryDefaults();

    // inOutMotor.restoreFactoryDefaults();
    m_forwardLimit.enableLimitSwitch(true);
    m_reverseLimit.enableLimitSwitch(true);
    m_shooterLiftMotor.setIdleMode(LiftConstants.ShooterLiftMotorIdleMode);

    liftPIDController.setFeedbackDevice(liftencoder);
    liftencoder.setInverted(false);
    liftPIDController.setP(LiftConstants.LiftP);
    liftPIDController.setI(LiftConstants.LiftI);
    liftPIDController.setD(LiftConstants.LiftD);
    liftPIDController.setFF(LiftConstants.LiftFF);
    liftPIDController.setOutputRange(LiftConstants.LiftMinOutput,
        LiftConstants.LiftMaxOutput);

    m_shooterLiftMotor.setInverted(false);
    m_shooterLiftMotor.burnFlash();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("REncoder Position", getCurrentPosition());
  }

  LiftPidPositions targetPosition = LiftPidPositions.Up;

  public void liftSetPoints(double rotation) {
    double m_rotation = rotation;
    liftPIDController.setReference(m_rotation, CANSparkMax.ControlType.kPosition);
  }

  public void setPowerOutput(double percent) {
    m_shooterLiftMotor.set(percent);
  }

  public double getCurrentPosition() {
    return liftencoder.getPosition();
  }

  public LiftPidPositions getTargetPosition() {
    return targetPosition;
  }

  public Command setTargetPositionAndWaitUntilAtGoal(LiftPidPositions targetPosition) {
    return setTargetPosition(targetPosition).andThen(Commands.waitUntil(this::isAtGoalFromLiftPid));
  }

  public Command setTargetPosition(LiftPidPositions targetPosition) {
    return Commands.runOnce(() -> this.targetPosition = targetPosition);
  }

  public boolean isAtGoalFromLiftPid() {
    var currentCommand = getCurrentCommand();
    if (currentCommand instanceof LiftPid) {
      currentCommand = (LiftPid) currentCommand;
      return ((LiftPid) currentCommand).getController().atGoal();
    }
    return false;
  }

}