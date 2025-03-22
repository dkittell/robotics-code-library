// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new Shooter. */

  private final static CANSparkFlex m_shooter1 = new CANSparkFlex(ShooterConstants.ShooterMotor1, MotorType.kBrushless);
  private final static CANSparkFlex m_shooter2 = new CANSparkFlex(ShooterConstants.ShooterMotor2, MotorType.kBrushless);

  /** Creates a new Lift. */
  public ShooterSubsystem() {
    m_shooter1.restoreFactoryDefaults();
    m_shooter2.restoreFactoryDefaults();
    m_shooter1.setInverted(true);
    m_shooter2.setInverted(true);

  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
  }

  public Command ShooterCommand(double speed) {
    // return this.runOnce(()->m_shooter1.set(speed));
    return startEnd(
        // Start a flywheel spinning at 50% power
        () -> this.SpeekerShoot(ShooterConstants.SpekerOnSpeed),
        // Stop the flywheel at the end of the command
        () -> this.SpeekerShoot(ShooterConstants.ShooterOff));
  }

  public Command ShooterCommand2(double speed) {
    return this.runOnce(() -> m_shooter2.set(speed));

  }

  public void Manual(double speed) {
    m_shooter1.set(-speed);
    m_shooter2.set(speed / 2);
  }

  public void SpeekerShoot(double speed) {
    m_shooter1.set(-speed);
    m_shooter2.set(speed);
  }

}
