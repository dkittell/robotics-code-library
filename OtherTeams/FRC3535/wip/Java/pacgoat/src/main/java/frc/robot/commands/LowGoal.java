/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Pivot;

/**
 * Spit the ball out into the low goal assuming that the robot is in front of
 * it.
 */
public class LowGoal extends CommandGroup {
  /**
   * Create a new low goal command.
   */
  public LowGoal() {
    addSequential(new SetPivotSetpoint(Pivot.kLowGoal));
    addSequential(new SetCollectionSpeed(Collector.kReverse));
    addSequential(new ExtendShooter());
  }
}
