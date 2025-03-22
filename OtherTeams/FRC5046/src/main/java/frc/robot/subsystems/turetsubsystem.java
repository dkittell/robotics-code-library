// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;



import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.TuretConstants;

public class turetsubsystem extends SubsystemBase {
   private final static CANSparkMax TuretMotor = new CANSparkMax(TuretConstants.TuretMotor,MotorType.kBrushless);
   private final static  SparkAbsoluteEncoder  turetencoder  = TuretMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
   private final static SparkPIDController TuretPIDController = TuretMotor.getPIDController();
  /** Creates a new turetsubsystem. */
  
  public turetsubsystem() { 
    TuretMotor.restoreFactoryDefaults();

    TuretPIDController.setFeedbackDevice(turetencoder);
    TuretPIDController.setP(TuretConstants.TuretP);
   TuretPIDController.setI(TuretConstants.TuretI);
   TuretPIDController.setD(TuretConstants.TuretD);
   TuretPIDController.setFF(TuretConstants.TuretFF);
   TuretPIDController.setOutputRange(TuretConstants.TuretMinOutput,
      TuretConstants. TuretMaxOutput);
   TuretMotor.setIdleMode(TuretConstants.TuretMotorIdleMode);   
  }

  @Override
  public void periodic() {
   
    // This method will be called once per scheduler run
  }
   public void ManualTuret(double speed){
      TuretMotor.set(speed);
}

    public void TuretSetPoints(double rotation){
      double m_rotation = rotation;
      TuretPIDController.setReference(m_rotation,CANSparkMax.ControlType.kPosition);
    }
}
