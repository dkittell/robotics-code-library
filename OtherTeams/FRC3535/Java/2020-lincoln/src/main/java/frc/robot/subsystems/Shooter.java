// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.frc3535_Constants;
import com.playingwithfusion.CANVenom;
import com.playingwithfusion.CANVenom.BrakeCoastMode;
import com.playingwithfusion.CANVenom.ControlMode;

/** Add your docs here. */
public class Shooter {

	public static com.playingwithfusion.CANVenom m_Shooter = new CANVenom(frc3535_Constants.m_Shooter); // shooter

	public Shooter() {
	m_Shooter.setBrakeCoastMode(BrakeCoastMode.Coast);
	}

}
