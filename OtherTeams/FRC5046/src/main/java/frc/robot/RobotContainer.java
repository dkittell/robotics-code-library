// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import frc.robot.Constants.InTakeConstants;
import frc.robot.Constants.LiftConstants;
import frc.robot.Constants.OIConstants;

import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TuretConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.LiftPid;
import frc.robot.commands.LiftPid.LiftPidPositions;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LiftMotorSubsysystem;
import frc.robot.subsystems.ServoSubsystems;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.extenderSubsystem;
import frc.robot.subsystems.turetsubsystem;
import frc.robot.subsystems.ServoSubsystems.ServoPosition;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  // private final HatchSubsystem m_hatchSubsystem = new HatchSubsystem();
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  // Retained command handles
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final turetsubsystem m_turetsubsystem = new turetsubsystem();
  private final ServoSubsystems m_servosubsystem = new ServoSubsystems();
  private final extenderSubsystem m_ExtenderSubsystem = new extenderSubsystem();
  private final LiftMotorSubsysystem m_LiftMotorSubsysystem = new LiftMotorSubsysystem();
  // The autonomous routines
  // A simple auto routine that drives forward a specified distance, and then
  // stops.
  private final Command m_simpleAuto = Autos.simpleAuto(m_robotDrive);
  // A complex auto routine that drives forward, drops a hatch, and then drives
  // backward.
  private final Command m_complexAuto = Autos.complexAuto(m_robotDrive, m_shooterSubsystem);
  private final Command m_drive = Autos.Drive(m_robotDrive, m_shooterSubsystem, m_intakeSubsystem, m_servosubsystem);
  // A chooser for autonomous commands
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  // The driver's controller

  // CommandJoystick m_driverController = new
  // CommandJoystick(OIConstants.kDriverControllerPort);

  CommandJoystick m_OparatorController = new CommandJoystick(OIConstants.OperatorControllerPort);
  Joystick m_driverController = new Joystick(OIConstants.kDriverControllerPort);

  // Joystick m_OparatorController = new Joystick(2);
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    // Set the default drive command to split-stick arcade drive
    m_robotDrive.setDefaultCommand(
        // A split-stick arcade command, with forward/backward controlled by the left
        // hand, and turning controlled by the right.
        new RunCommand(
            () -> m_robotDrive.arcadeDrive(
                -m_driverController.getRawAxis(1), -m_driverController.getRawAxis(4)),
            m_robotDrive));

    m_ExtenderSubsystem.setDefaultCommand(
        new RunCommand(() -> m_ExtenderSubsystem.ManualinOutMotor(
            MathUtil.applyDeadband(-m_OparatorController.getRawAxis(5), OIConstants.kOperatorDeadband)),
            m_ExtenderSubsystem));

    // m_ExtenderSubsystem.setDefaultCommand(
    // new RunCommand(() -> m_ExtenderSubsystem.ManualinOutMotor
    // (m_OparatorController.getRawAxis(1))));

    m_turetsubsystem.setDefaultCommand(Commands.run(() -> m_turetsubsystem.TuretSetPoints(TuretConstants.TuretCenter),
        m_turetsubsystem));
    m_LiftMotorSubsysystem
        .setDefaultCommand(new LiftPid(m_LiftMotorSubsysystem));

    // Add commands to the autonomous command chooser

    m_chooser.setDefaultOption("Simple Auto", m_simpleAuto);
    m_chooser.addOption("Complex Auto", m_complexAuto);
    m_chooser.addOption("drivforward", m_drive);
    // Put the chooser on the dashboard
    Shuffleboard.getTab("Autonomous").add(m_chooser);

    // Put subsystems to dashboard.
    Shuffleboard.getTab("Drivetrain").add(m_robotDrive);
    Shuffleboard.getTab("ShooterSubsystem").add(m_shooterSubsystem);

    // Set the scheduler to log Shuffleboard events for command initialize,
    // interrupt, finish
    CommandScheduler.getInstance()
        .onCommandInitialize(
            command -> Shuffleboard.addEventMarker(
                "Command initialized", command.getName(), EventImportance.kNormal));
    CommandScheduler.getInstance()
        .onCommandInterrupt(
            command -> Shuffleboard.addEventMarker(
                "Command interrupted", command.getName(), EventImportance.kNormal));
    CommandScheduler.getInstance()
        .onCommandFinish(
            command -> Shuffleboard.addEventMarker(
                "Command finished", command.getName(), EventImportance.kNormal));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link PS4Controller}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // m_driverController
    new JoystickButton(m_driverController, 5)
        .onFalse(Commands.runOnce(() -> m_robotDrive.setMaxOutput(0.5)))
        .onTrue(Commands.runOnce(() -> m_robotDrive.setMaxOutput(1)));

    m_OparatorController
        .button(OIConstants.kOperatorBottonA).toggleOnTrue(Commands.startEnd(
            // Start a flywheel spinning at 50% power
            () -> m_intakeSubsystem.intakeManual(InTakeConstants.InTakeOnSpeed),
            // Stop the flywheel at the end of the command
            () -> m_intakeSubsystem.intakeManual(InTakeConstants.InTakeOffSpeed),
            // Requires the shooter subsystem
            m_intakeSubsystem));// new code1/1/24

    new JoystickButton(m_driverController, OIConstants.kOperatorBottonA).toggleOnTrue(Commands.startEnd(
        // Start a flywheel spinning at 50% power
        () -> m_intakeSubsystem.intakeManual(InTakeConstants.InTakeOnSpeed),
        // Stop the flywheel at the end of the command
        () -> m_intakeSubsystem.intakeManual(InTakeConstants.InTakeOffSpeed),
        // Requires the shooter subsystem
        m_intakeSubsystem));
    m_OparatorController
        .button(4).toggleOnTrue(Commands.startEnd(
            // Start a flywheel spinning at 50% power
            () -> m_shooterSubsystem.Manual(ShooterConstants.AmpOnSpeed2),
            // Stop the flywheel at the end of the command
            () -> m_shooterSubsystem.Manual(ShooterConstants.ShooterOff),
            // Requires the shooter subsystem
            m_shooterSubsystem));

    m_OparatorController
        .button(OIConstants.kOperatorBottonB).toggleOnTrue(Commands.startEnd(
            // Start a flywheel spinning at 50% power
            () -> m_shooterSubsystem.Manual(ShooterConstants.SpekerOnSpeed),
            // Stop the flywheel at the end of the command
            () -> m_shooterSubsystem.Manual(ShooterConstants.ShooterOff),
            // Requires the shooter subsystem
            m_shooterSubsystem));

    m_OparatorController
        .button(OIConstants.kOperatorBottonLeftJoyStick).toggleOnTrue(Commands.deferredProxy(() -> {
          var speed = extended ? 0.5 : -.3;
          extended = !extended;
          return m_ExtenderSubsystem.powerlift(speed);
        }));

    m_OparatorController.button(OIConstants.kOperatorBottonX)
        .toggleOnTrue(new RunCommand(() -> m_turetsubsystem.TuretSetPoints(TuretConstants.Turetleft),
            m_turetsubsystem));

    m_OparatorController.button(OIConstants.kOperatorBottonleftButton)
        .toggleOnTrue(Commands.deferredProxy(
            () -> {
              var targetPosition = m_LiftMotorSubsysystem.getTargetPosition().equals(LiftPidPositions.Up)
                  ? LiftPidPositions.Down
                  : LiftPidPositions.Up;
              return m_LiftMotorSubsysystem.setTargetPosition(targetPosition);
            }));
    m_OparatorController.button(OIConstants.kOperatorBottonRightButton)
        .toggleOnTrue(Commands.deferredProxy(() -> {
          var targetPosition = servoUp ? ServoPosition.Down : ServoPosition.Up;
          servoUp = !servoUp;
          return m_servosubsystem.servoPosition(targetPosition);
        }));
  }

  static boolean extended = false;
  static boolean servoUp = true;

  // Commands.parallel(m_shooterSubsystem.ShooterCommand(ShooterConstants.SpekerOnSpeed)),
  // Commands.sequence(Command
  // waitSeconds(1.0),m_intakeSubsystem.IntakeCommand(1)).withTimeout?(5)
  // public Command ampitup( ShooterSubsystem m_shooterSubsystem,
  // IntakeSubsystem m_IntakeSubsystem){
  // Command.parallel(m_shooterSubsystem.ShooterCommand(ShooterConstants.SpekerOnSpeed),
  // Commands.sequence(Commands.waitSeconds(1.0),
  // m_IntakeSubsystem.IntakeCommand(1)))
  // .withTimeout(5.0);
  // }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }

}
