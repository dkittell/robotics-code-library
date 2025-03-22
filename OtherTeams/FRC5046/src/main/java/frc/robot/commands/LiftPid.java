// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

//import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
//mport edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.LiftMotorSubsysystem;

/** A command that will turn the robot to the specified angle. */
public class LiftPid extends ProfiledPIDCommand {
  public enum LiftPidPositions {
    Up(0.412),
    Down(0.3);

    final double position;

    LiftPidPositions(double position) {
      this.position = position;
    }
  }

  public LiftPid(LiftMotorSubsysystem liftMotorSubsysystem) {
    super(
        new ProfiledPIDController(2, 0, 0, // TODO change PID values and
            // velocity and accel
            new TrapezoidProfile.Constraints(0.25, 0.25)),
        // Close loop on heading
        () -> liftMotorSubsysystem.getCurrentPosition(),
        // Set reference to target
        () -> liftMotorSubsysystem.getTargetPosition().position,
        // Pipe output to turn robot
        (output, state) -> liftMotorSubsysystem.setPowerOutput(output),
        // Require the drive
        liftMotorSubsysystem);

    // Set the controller to be continuous (because it is an angle controller)

    // Set the controller tolerance - the delta tolerance ensures the robot is
    // stationary at the
    // setpoint before it is considered as having reached the reference
    getController()
        .setTolerance(.05, .05);
  }

  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    return getController().atGoal();
  }
}
