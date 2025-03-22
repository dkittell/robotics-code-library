// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.IdleMode;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    public static final int kLeftMotor1Port = 2;
    public static final int kLeftMotor2Port = 4;

    public static final int kRightMotor1Port = 1;
    public static final int kRightMotor2Port = 3;

    public static final int[] kLeftEncoderPorts = new int[] { 0, 1 };
    public static final int[] kRightEncoderPorts = new int[] { 2, 3 };
    public static final boolean kLeftEncoderReversed = true;
    public static final boolean kRightEncoderReversed = false;

    public static final int kEncoderCPR = 8192;
    public static final double kWheelDiameterInches = 6;
    public static final double kEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (kWheelDiameterInches * Math.PI) / (double) kEncoderCPR;

    public static final boolean kGyroReversed = false;

    public static final double kStabilizationP = 1;
    public static final double kStabilizationI = 0.;
    public static final double kStabilizationD = 0;

    public static final double kTurnP = 0.05;
    public static final double kTurnD = 0.000;

    public static final double kMaxTurnRateDegPerS = 100;
    public static final double kMaxTurnAccelerationDegPerSSquared = 300;

    public static final double kTurnToleranceDeg = 25;
    public static final double kTurnRateToleranceDegPerS = 50; // degrees per second

    public static final double rotationOffDriveMotor = 2.228 * .4489; // 1.11

    public static final int kDrivingMotorPinionTeeth = 14;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 6;// 0.1524
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 50) / (kDrivingMotorPinionTeeth * 19);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double LDrivingP = 0.1;
    public static final double LDrivingI = 0;
    public static final double LDrivingD = 0;
    public static final double LDrivingFF = 0;// kDriveWheelFreeSpeedRps;
    public static final double LDrivingMinOutput = -50;
    public static final double LDrivingMaxOutput = .50;

    public static final double RDrivingP = 0.1;
    public static final double RDrivingI = 0;

    public static final double RDrivingD = 0;
    public static final double RDrivingFF = 1;/// kDriveWheelFreeSpeedRps;
    public static final double RDrivingMinOutput = -.50;
    public static final double RDrivingMaxOutput = .50;

    public static final IdleMode DriveMotorIdleMode = IdleMode.kBrake;
  }

  public static final class InTakeConstants {
    public static final int IntakeMotor = 12;
    public static final int shooterintakeMotor = 10;
    public static final double InTakeOnSpeed = 1;
    public static final double InTakeOffSpeed = 0;
    public static final double InTakeRevSpeed = -1;

  }

  public static final class ShooterConstants {

    public static final int ShooterMotor1 = 40;
    public static final int ShooterMotor2 = 41;
    public static final double SpekerOnSpeed = -.8;
    public static final double AmpOnSpeed2 = -.30;
    public static final double ShooterOff = 0;
    public static final IdleMode ShooterMotorIdleMode = IdleMode.kBrake;
    public static final NeutralModeValue ShooterMotor1NeutralModeValue = NeutralModeValue.Brake;
    public static final NeutralModeValue ShooterMotor2NeutralModeValue = NeutralModeValue.Brake;
  }

  public static final class LiftConstants {
    public static final int ShooterLiftMotor = 14;
    public static final double ShooterLiftUpSpeed = 1;
    public static final double ShooterLiftDownSpeed = -1;
    public static final IdleMode ShooterLiftMotorIdleMode = IdleMode.kBrake;
    public static final double LiftP = 1;
    public static final double LiftI = 0;
    public static final double LiftD = 0;
    public static final double LiftFF = 0;
    public static final double LiftMinOutput = -1;
    public static final double LiftMaxOutput = 1;
    public static final double maxRPM = 5700;
    public static final double liftUp = 0.433;
    public static final double liftmid = .300;
    public static final double liftdown = .638;
  }

  public static final class TuretConstants {
    public static final int TuretMotor = 5;
    public static final IdleMode TuretMotorIdleMode = IdleMode.kBrake;
    public static final double TuretP = 1;
    public static final double TuretI = 0;
    public static final double TuretD = 0;
    public static final double TuretFF = 0;
    public static final double TuretMinOutput = -1;
    public static final double TuretMaxOutput = 1;
    public static final double maxRPM = 5700;
    public static final double TuretCenter = .75;
    public static final double Turetright = 0.0;
    public static final double Turetleft = .50;
  }

  public static final class extenderConstants {
    public static final int inOutMotor = 11;
    public static final double inoutmotorUpSpeed = 1;
    public static final double inoutmotorDownSpeed = -1;
    public static final IdleMode inoutmotorIdleMode = IdleMode.kBrake;
    public static final double inoutP = 1;
    public static final double inoutI = 0;
    public static final double inoutD = 0;
    public static final double inoutFF = 0;
    public static final double inoutMinOutput = -1;
    public static final double inoutMaxOutput = 1;
    public static final double maxRPM = 5700;
    public static final double out = 0.85;
    public static final double outmid = .300;
    public static final double in = .0;
  }

  public static final class AutoConstants {
    public static final double kAutoDriveDistanceInches = 10;
    public static final double kAutoBackupDistanceInches = 20;
    public static final double kAutoDriveSpeed = 0.5;
    public static final double TurnToAngleleft = 90;
    public static final double TurnToAngright = -90;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int OperatorControllerPort = 1;

    public static final double kDriveDeadband = 0.1;
    public static final int kDriverBottonA = 1;
    public static final int kDriverBottonB = 2;
    public static final int kDriverBottonX = 3;
    public static final int kDriverBottonY = 4;
    public static final int kDriverBottonleftButton = 5;
    public static final int kDriverBottonRightButton = 6;
    public static final int kDriverBottonBack = 7;
    public static final int kDriverBottonStart = 8;
    public static final int kDriverBottonLeftJoyStick = 9;
    public static final int kDriverBottonRightJoyStick = 10;

    public static final double kOperatorDeadband = 0.1;
    public static final int kOperatorBottonA = 1;
    public static final int kOperatorBottonB = 2;
    public static final int kOperatorBottonX = 3;
    public static final int kOperatorBottonY = 4;
    public static final int kOperatorBottonleftButton = 5;
    public static final int kOperatorBottonRightButton = 6;
    public static final int kOperatorBottonBack = 7;
    public static final int kOperatorBottonStart = 8;
    public static final int kOperatorBottonLeftJoyStick = 9;
    public static final int kOperatorBottonRightJoyStick = 10;
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }
}
