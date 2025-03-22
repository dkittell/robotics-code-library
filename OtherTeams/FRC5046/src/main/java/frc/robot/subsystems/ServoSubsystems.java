// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ServoSubsystems extends SubsystemBase {
  /** Creates a new ServoSubsystems. */
  private final static Servo servo = new Servo(1);

  public ServoSubsystems() {

  }

  public enum ServoPosition {
    Up(0),
    Down(0.25);

    final double position;

    ServoPosition(double position) {
      this.position = position;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setServoPosition(ServoPosition targetPosition) {
    servo.set(targetPosition.position);
  }

  public double getservoPosition() {
    return servo.getPosition();
  }

  public Command servoPosition(ServoPosition targetPosition) {
    return run(() -> setServoPosition(targetPosition)).until(() -> getservoPosition() == targetPosition.position);
  }

}
