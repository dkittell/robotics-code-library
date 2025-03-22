package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//region Venom Code Imports
import com.playingwithfusion.CANVenom;
//endregion Venom Code Imports

public class VenomDrive extends Subsystem {

	public VenomDrive() {
	}

	@Override
	public void initDefaultCommand() {
	}

	// region Venom
	public  com.playingwithfusion.CANVenom m_frontLeft = new CANVenom(2);
	public  com.playingwithfusion.CANVenom m_rearLeft = new CANVenom(1);
	public  com.playingwithfusion.CANVenom m_frontRight = new CANVenom(4);
	public  com.playingwithfusion.CANVenom m_rearRight = new CANVenom(3);
	public com.playingwithfusion.CANVenom m_Shooter = new CANVenom(5);
	// SpeedControllerGroup m_left = new SpeedControllerGroup(m_frontLeft,
	// m_rearLeft);
	// SpeedControllerGroup m_right = new SpeedControllerGroup(m_frontRight,
	// m_rearRight);
	// DifferentialDrive m_drive = new DifferentialDrive(m_left, m_right);
	 DifferentialDrive m_drive = new DifferentialDrive(m_frontLeft, m_frontRight);
	// endregion Venom

	public  void Drive(double dDrive, double dSteer) {

		m_rearLeft.follow(m_frontLeft);
		m_rearRight.follow(m_frontRight);

		m_drive.arcadeDrive(dDrive, dSteer);
		SmartDashboard.putNumber("Venom_FrontLeft", m_frontLeft.getSpeed());
		SmartDashboard.putNumber("Venom_FrontRight", m_frontRight.getSpeed());
		SmartDashboard.putNumber("Venom_RearLeft", m_rearLeft.getSpeed());
		SmartDashboard.putNumber("Venom_RearRight", m_rearRight.getSpeed());
	}

	 double targetDistance = 1.5; // Target Area (NEED TO DO Target Area to Inches)
	 double driveScaleDown = 1;
	// double turnScaleDown = 0.05;
	 double turnScaleDown = 0.04;
	 double tx = 0.0;
	 double ta = 0.0;
	 double m_speed = 0.0;
	 double output = 0.0;
	 boolean turnInPlace = false;

	public  void LimelightBasic() {
		tx = Vision.getTX();
		ta = Vision.getTA();
		if (Math.abs(tx) > 0) {
			// m_drive.arcadeDrive((getDriveSpeed() * driveScaleDown), (tx *
			// turnScaleDown));
			Drive((getDriveSpeed() * driveScaleDown), (tx * turnScaleDown));
		} else {
			Drive(0.0, 0.0);
			// m_drive.arcadeDrive(0.0, 0.0);
		}
	}

	public  double getDriveSpeed() {
		double ta = Vision.getTA();
		m_speed = 0.6; // Max speed - Auto scales down the speed of the motor

		if (ta > 0) {
			m_speed = (m_speed * ((targetDistance - ta) / targetDistance)); // Distance in Inches

			if (m_speed <= -0.5) {
				m_speed = -0.5;
			}
		} else {
			m_speed = 0;
			output = 0;
		}

		if (turnInPlace) {
			m_speed = 0;
		}

		return m_speed;
	}

}