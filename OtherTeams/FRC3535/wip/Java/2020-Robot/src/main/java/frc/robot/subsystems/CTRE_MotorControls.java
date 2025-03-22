package frc.robot.subsystems;

// region Imports
// region FRC3535 Imports
import frc.robot.constants.*;
import frc.robot.subsystems.*;
import frc.robot.frc3535_Variables;
// endregion FRC3535 Imports

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.command.Subsystem;

//region CTRE Imports
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
//endregion CTRE Imports

//endregion Imports

public class CTRE_MotorControls extends Subsystem {

	// region CTRE
	static com.ctre.phoenix.motorcontrol.can.TalonSRX m_ControlPanel = new com.ctre.phoenix.motorcontrol.can.TalonSRX(frc3535_Variables.m_ControlPanel);
	// static com.ctre.phoenix.motorcontrol.can.TalonSRX m_Climber01 = new com.ctre.phoenix.motorcontrol.can.TalonSRX(frc3535_Variables.m_Climber01);
	// static com.ctre.phoenix.motorcontrol.can.TalonSRX m_Climber02 = new com.ctre.phoenix.motorcontrol.can.TalonSRX(frc3535_Variables.m_Climber02);
	// static com.ctre.phoenix.motorcontrol.can.TalonSRX m_ClimberPosition = new com.ctre.phoenix.motorcontrol.can.TalonSRX(frc3535_Variables.m_ClimberPosition);
	static com.ctre.phoenix.motorcontrol.can.TalonSRX m_Intake = new com.ctre.phoenix.motorcontrol.can.TalonSRX(frc3535_Variables.m_Intake);
	static com.ctre.phoenix.motorcontrol.can.TalonSRX m_Uptake = new com.ctre.phoenix.motorcontrol.can.TalonSRX(frc3535_Variables.m_Uptake);

	// endregion CTRE

	public CTRE_MotorControls() {
	}

	@Override
	public void initDefaultCommand() {
	}

	public void ControlPanel() {
		m_ControlPanel.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, 0.125);
	}

	// public static void ClimbInit() {
	// 	// region CTRE Climber
	// 	/* factory default values */
	// 	m_Climber01.configFactoryDefault();
	// 	m_Climber02.configFactoryDefault();

	// 	/*
	// 	 * choose whatever you want so "positive" values moves mechanism forward,
	// 	 * upwards, outward, etc.
	// 	 *
	// 	 * Note that you can set this to whatever you want, but this will not fix motor
	// 	 * output direction vs sensor direction.
	// 	 */
	// 	m_Climber01.setInverted(false);
	// 	m_Climber02.setInverted(false);

	// 	/*
	// 	 * flip value so that motor output and sensor velocity are the same polarity. Do
	// 	 * this before closed-looping
	// 	 */
	// 	m_Climber01.setSensorPhase(false); // <<<<<< Adjust this

	// 	// endregion CTRE Climber
	// }

	// public static void Climber_Climb(double xSpeed) {
	// 	m_Climber01.getSelectedSensorPosition();

	// 	m_Climber01.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, xSpeed);
	// 	m_Climber02.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, xSpeed);

	// 	/* hold down btn1 to print stick values */
	// 	// if (js1.getRawButton(1)) {
	// 	SmartDashboard.putNumber("Climber 01 Sensor Vel", m_Climber01.getSelectedSensorVelocity());
	// 	SmartDashboard.putNumber("Climber 01 Sensor Pos", m_Climber01.getSelectedSensorPosition());
	// 	SmartDashboard.putNumber("Climber 01 Out %", m_Climber01.getMotorOutputPercent());
	// 	SmartDashboard.putNumber("Climber 01 Out %", m_Climber02.getMotorOutputPercent());
	// }

	// public static void Climber_Position(double xSpeed) {
	// 	m_ClimberPosition.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput,
	// 			frc3535_Joystick.getDeadBand(2, 1, 2));
	// }

	public static void Intake(double xSpeed){
		m_Intake.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput,xSpeed);
	}

	public static void Uptake(double xSpeed){
		m_Uptake.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput,xSpeed);
	}

}