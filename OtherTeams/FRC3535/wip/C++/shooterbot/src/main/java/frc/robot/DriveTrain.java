package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Joystick;

public final class DriveTrain {
	// region Controller Definition
	// region Dead Band Variables JS1
	double js1lSpeed = 0; // Speed controls up & down
	double js1rSpeed = 0; // Speed controls up & down
	double js1lRotate = 0; // Rotate controls left & right
	double js1rRotate = 0; // Rotate controls left & right
	double js1LTSpeed = 0;
	double js1RTSpeed = 0;

	// JS1 Left Deadband
	double js1lDeadband_1 = 0.15;
	double js1lDeadband_2 = 0.90;

	// JS1 Right Deadband
	double js1rDeadband_1 = 0.15;
	double js1rDeadband_2 = 0.60;
	// endregion Dead Band Variables JS1

	// region Dead Band Variables JS2
	double js2lSpeed = 0; // Speed controls up & down
	double js2rSpeed = 0; // Speed controls up & down
	double js2lRotate = 0; // Rotate controls left & right
	double js2rRotate = 0; // Rotate controls left & right
	double js2LTSpeed = 0;
	double js2RTSpeed = 0;

	// JS2 Left Deadband
	double js2lDeadband_1 = 0.15;
	double js2lDeadband_2 = 0.90;

	// JS2 Right Deadband
	double js2rDeadband_1 = 0.15;
	double js2rDeadband_2 = 0.50;
	// endregion Dead Band Variables JS2

	public static double jsDeadBand(double input) {
		double jsToggle = 0;

		return jsToggle;
	}

}
