// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ServoSubsystems;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ServoSubsystems.ServoPosition;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/** Container for auto command factories. */
public final class Autos {
  /**
   * A simple auto routine that drives forward a specified distance, and then
   * stops.
   */
  public static Command simpleAuto(DriveSubsystem drive) {
    return new FunctionalCommand(
        // Reset encoders on command start
        drive::resetEncoders,
        // Drive forward while the command is executing
        () -> drive.arcadeDrive(AutoConstants.kAutoDriveSpeed, 0),
        // Stop driving at the end of the command
        interrupt -> drive.arcadeDrive(0, 0),
        // End the command when the robot's driven distance exceeds the desired value
        () -> drive.getAverageEncoderDistance() >= AutoConstants.kAutoDriveDistanceInches,
        // Require the drive subsystem
        drive);
  }

  public static Command Drive(DriveSubsystem DriveSubsystem, ShooterSubsystem m_ShooterSubsystem,
      IntakeSubsystem m_IntakeSubsystem, ServoSubsystems m_ServoSubsystem) {
    return Commands.sequence(
        // new DriveDistance(AutoConstants.kAutoDriveDistanceInches,
        // AutoConstants.kAutoDriveSpeed, DriveSubsystem),

        m_ServoSubsystem.servoPosition(ServoPosition.Down),

        // new TurnToAngle(AutoConstants.TurnToAngleleft, DriveSubsystem),
        // new shooterrun(m_ShooterSubsystem,m_intakeSubsystem),
        Commands.parallel(m_ShooterSubsystem.ShooterCommand(ShooterConstants.SpekerOnSpeed),
            Commands.sequence(Commands.waitSeconds(1.0), m_IntakeSubsystem.IntakeCommand(1)))
            .withTimeout(5.0),

        new DriveDistance(AutoConstants.kAutoDriveDistanceInches, -AutoConstants.kAutoDriveSpeed, DriveSubsystem));
  }

  /**
   * A complex auto routine that drives forward, drops a hatch, and then drives
   * backward.
   */
  public static Command complexAuto(DriveSubsystem driveSubsystem, ShooterSubsystem m_ShooterSubsystem) {
    return Commands.sequence(
        // Drive forward up to the front of the cargo ship
        new FunctionalCommand(
            // Reset encoders on command start
            driveSubsystem::resetEncoders,
            // Drive forward while the command is executing
            () -> driveSubsystem.arcadeDrive(AutoConstants.kAutoDriveSpeed, 0),
            // Stop driving at the end of the command
            interrupt -> driveSubsystem.arcadeDrive(0, 0),
            // End the command when the robot's driven distance exceeds the desired value
            () -> driveSubsystem.getAverageEncoderDistance() >= AutoConstants.kAutoDriveDistanceInches,
            // Require the drive subsystem
            driveSubsystem),

        // turnon turnoffShooter
        // m_ShooterSubsystem.ShooterCommand(ShooterConstants.ShooterRevOnSpeed2),
        new WaitCommand(2),
        // m_ShooterSubsystem.ShooterCommand(ShooterConstants.ShooterOff),
        new FunctionalCommand(
            // Reset encoders on command start
            driveSubsystem::resetEncoders,
            // Drive backward while the command is executing
            () -> driveSubsystem.arcadeDrive(-AutoConstants.kAutoDriveSpeed, 0),
            // Stop driving at the end of the command
            interrupt -> driveSubsystem.arcadeDrive(0, 0),
            // End the command when the robot's driven distance exceeds the desired value
            () -> driveSubsystem.getAverageEncoderDistance() <= AutoConstants.kAutoBackupDistanceInches,
            // Require the drive subsystem
            driveSubsystem));
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
