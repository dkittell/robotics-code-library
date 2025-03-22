package frc.robot.constants;

//region Imports
import frc.robot.frc3535_Variables;
import edu.wpi.first.wpilibj.Joystick;
//endregion Imports

public class frc3535_Joystick {

	// region Controller Definition
	private final static Joystick js1 = new Joystick(frc3535_Variables.js1);
	private final static Joystick js2 = new Joystick(frc3535_Variables.js2);
	// endregion Controller Definition

	public static double getDeadBand(int js, int analog, int direction) {

		// frc3535_Joystick.getDeadBand(1, 1, 1)
		// Joystick
		// 1 - Driver
		// 2 - Operator
		// Analog
		// 1 - Left
		// 2 - Right
		// Direction
		// 1 - Up or Down
		// 2 - Left or Right

		double deadbandSpeed = 0.0;
		double axisSpeed = 0.0;

		switch (js) {
			case 1: // Driver Controller
				switch (analog) {
					case 1: // Left Analog Stick
						switch (direction) {
							case 1: // Up or Down
								deadbandSpeed = js1.getRawAxis(frc3535_Variables.axisD_lUpDown); // Speed controls up &
																									// down
								if ((deadbandSpeed > frc3535_Variables.js1_L_UpDown_Deadband_Low)) {
									deadbandSpeed = ((deadbandSpeed - frc3535_Variables.js1_L_UpDown_Deadband_Low)
											* frc3535_Variables.js1_L_UpDown_Deadband_High);
								} else if ((deadbandSpeed < -frc3535_Variables.js1_L_UpDown_Deadband_Low)) {
									deadbandSpeed = ((deadbandSpeed + frc3535_Variables.js1_L_UpDown_Deadband_Low)
											* frc3535_Variables.js1_L_UpDown_Deadband_High);
								} else {
									deadbandSpeed = 0; // If between boundaries. Do nothing.
								}

							case 2: // Left or Right
								deadbandSpeed = js1.getRawAxis(frc3535_Variables.axisD_lLeftRight); // Speed controls up
																									// & down
								if ((deadbandSpeed > frc3535_Variables.js1_L_LeftRight_Deadband_Low)) {
									deadbandSpeed = ((deadbandSpeed - frc3535_Variables.js1_L_LeftRight_Deadband_Low)
											* frc3535_Variables.js1_L_LeftRight_Deadband_High);
								} else if ((deadbandSpeed < -frc3535_Variables.js1_L_LeftRight_Deadband_Low)) {
									deadbandSpeed = ((deadbandSpeed + frc3535_Variables.js1_L_LeftRight_Deadband_Low)
											* frc3535_Variables.js1_L_LeftRight_Deadband_High);
								} else {
									deadbandSpeed = 0; // If between boundaries. Do nothing.
								}
						}
					case 2: // Right Analog Stick
						switch (direction) {
							case 1: // Up or Down
								deadbandSpeed = js1.getRawAxis(frc3535_Variables.axisD_rUpDown); // Speed controls up &
																									// down
								if ((deadbandSpeed > frc3535_Variables.js1_R_UpDown_Deadband_Low)) {
									deadbandSpeed = ((deadbandSpeed - frc3535_Variables.js1_R_UpDown_Deadband_Low)
											* frc3535_Variables.js1_R_UpDown_Deadband_High);
								} else if ((deadbandSpeed < -frc3535_Variables.js1_R_UpDown_Deadband_Low)) {
									deadbandSpeed = ((deadbandSpeed + frc3535_Variables.js1_R_UpDown_Deadband_Low)
											* frc3535_Variables.js1_R_UpDown_Deadband_High);
								} else {
									deadbandSpeed = 0; // If between boundaries. Do nothing.
								}
							case 2: // Left or Right
								deadbandSpeed = js1.getRawAxis(frc3535_Variables.axisD_rLeftRight); // Speed controls up
																									// & down
								if ((deadbandSpeed > frc3535_Variables.js1_R_LeftRight_Deadband_Low)) {
									deadbandSpeed = ((deadbandSpeed - frc3535_Variables.js1_R_LeftRight_Deadband_Low)
											* frc3535_Variables.js1_R_LeftRight_Deadband_High);
								} else if ((deadbandSpeed < -frc3535_Variables.js1_R_LeftRight_Deadband_Low)) {
									deadbandSpeed = ((deadbandSpeed + frc3535_Variables.js1_R_LeftRight_Deadband_Low)
											* frc3535_Variables.js1_R_LeftRight_Deadband_High);
								} else {
									deadbandSpeed = 0; // If between boundaries. Do nothing.
								}
						}
				}

			case 2: // Operator Controller
				switch (analog) {
					case 1: // Left Analog Stick
						switch (direction) {
							case 1: // Up or Down
								axisSpeed = js2.getRawAxis(frc3535_Variables.axisD_lUpDown); // Speed controls up &
																								// down
								if ((axisSpeed > frc3535_Variables.js2_L_UpDown_Deadband_Low)) {
									deadbandSpeed = ((axisSpeed - frc3535_Variables.js2_L_UpDown_Deadband_Low)
											* frc3535_Variables.js2_L_UpDown_Deadband_High);
								} else if ((axisSpeed < -frc3535_Variables.js2_L_UpDown_Deadband_Low)) {
									deadbandSpeed = ((axisSpeed + frc3535_Variables.js2_L_UpDown_Deadband_Low)
											* frc3535_Variables.js2_L_UpDown_Deadband_High);
								} else {
									deadbandSpeed = 0.0; // If between boundaries. Do nothing.
								}
							case 2: // Left or Right
								axisSpeed = js2.getRawAxis(frc3535_Variables.axisD_lLeftRight); // Speed controls up
																								// & down
								if ((axisSpeed > frc3535_Variables.js2_L_LeftRight_Deadband_Low)) {
									deadbandSpeed = ((axisSpeed - frc3535_Variables.js2_L_LeftRight_Deadband_Low)
											* frc3535_Variables.js2_L_LeftRight_Deadband_High);
								} else if ((axisSpeed < -frc3535_Variables.js2_L_LeftRight_Deadband_Low)) {
									deadbandSpeed = ((axisSpeed + frc3535_Variables.js2_L_LeftRight_Deadband_Low)
											* frc3535_Variables.js2_L_LeftRight_Deadband_High);
								} else {
									deadbandSpeed = 0.0; // If between boundaries. Do nothing.
								}
						}
					case 2: // Right Analog Stick
						switch (direction) {
							case 1: // Up or Down
								axisSpeed = js2.getRawAxis(frc3535_Variables.axisD_rUpDown); // Speed controls up &
																								// down
								if ((axisSpeed > frc3535_Variables.js2_R_UpDown_Deadband_Low)) {
									deadbandSpeed = ((axisSpeed - frc3535_Variables.js2_R_UpDown_Deadband_Low)
											* frc3535_Variables.js2_R_UpDown_Deadband_High);
								} else if ((axisSpeed < -frc3535_Variables.js2_R_UpDown_Deadband_Low)) {
									deadbandSpeed = ((axisSpeed + frc3535_Variables.js2_R_UpDown_Deadband_Low)
											* frc3535_Variables.js2_R_UpDown_Deadband_High);
								} else {
									deadbandSpeed = 0.0; // If between boundaries. Do nothing.
								}
							case 2: // Left or Right
								axisSpeed = js2.getRawAxis(frc3535_Variables.axisD_rLeftRight); // Speed controls up
																								// & down
								if ((axisSpeed > frc3535_Variables.js2_R_LeftRight_Deadband_Low)) {
									deadbandSpeed = ((axisSpeed - frc3535_Variables.js2_R_LeftRight_Deadband_Low)
											* frc3535_Variables.js2_R_LeftRight_Deadband_High);
								} else if ((axisSpeed < -frc3535_Variables.js2_R_LeftRight_Deadband_Low)) {
									deadbandSpeed = ((axisSpeed + frc3535_Variables.js2_R_LeftRight_Deadband_Low)
											* frc3535_Variables.js2_R_LeftRight_Deadband_High);
								} else {
									deadbandSpeed = 0.0; // If between boundaries. Do nothing.
								}
						}
				}
		}

		return deadbandSpeed;
	}

}