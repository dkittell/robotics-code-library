package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.net.Socket;

// region Limelight
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
// endregion Limelight

public class Vision extends Subsystem {

	public Vision() {

	}

	double dTargetHeight = 8.5; // Height in feet
	double dLimeLightHeight = 4.0; // Height in feet
	static boolean m_LimelightHasValidTarget = false;
	static double m_LimelightDriveCommand = 0.0;
	static double m_LimelightSteerCommand = 0.0;

	@Override
	public void initDefaultCommand() {

	}

	/**
	 * Sets the LED mode to on, off, or blink
	 * 
	 * @param mode - the mode of the LEDs Example: 0: Sets the mode to what is in
	 *             the current pipeline 1: Turns off the LEDs 2: Blink mode on LEDs
	 *             3: Turns on the LEDs
	 */
	public static void setLEDMode(int mode) {
		try {
			NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(mode);
		} catch (Exception e) {

			System.out.println("Error Vision.java: " + Thread.currentThread().getStackTrace()[1].getLineNumber()
					+ " - Unable to connect to the LimeLight");
		}
	}

	/**
	 * Gets the Limelight's LED mode from network tables
	 * 
	 * @return - returns the LED mode value as an int from networktables
	 */
	public static int getLEDMode() {
		int nLEDMode = 0;

		try {
			nLEDMode = (int) NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").getValue()
					.getDouble();
		} catch (Exception e) {

			System.out.println("Error Vision.java: " + Thread.currentThread().getStackTrace()[1].getLineNumber()
					+ " - Unable to connect to the LimeLight");
		}

		return nLEDMode;
	}

	public static boolean checkLimeLightConnectivity() {
		boolean bLimeLightConnected = false;

		try {
			Socket socket = new Socket("10.35.35.11", 5801);
			bLimeLightConnected = socket.isConnected();
			socket.close();
		} catch (Exception e) {
			System.out.println("Error Vision.java: " + Thread.currentThread().getStackTrace()[1].getLineNumber()
					+ " - Unable to connect to the LimeLight");
		}
		SmartDashboard.putBoolean("limelight_Connected", bLimeLightConnected);
		return bLimeLightConnected;
	}

	public static void getLimeLightSmartDashboad() {

		try {

			Vision.setLEDMode(0);
			SmartDashboard.putNumber("limelight_X", Vision.getTX());
			SmartDashboard.putNumber("limelight_Y", Vision.getTY());
			SmartDashboard.putNumber("limelight_Area", Vision.getTA());
			SmartDashboard.putNumber("limelight_Latency", Vision.getTL());
			SmartDashboard.putNumber("limelight_Valid_Target", Vision.getTV());
			SmartDashboard.putNumber("limelight_Skew", Vision.getTS());
			SmartDashboard.putNumber("limelight_Steering_Adjust", Vision.getSteeringAdjust());

		} catch (Exception e) {
			System.out.println("Error Vision.java: " + Thread.currentThread().getStackTrace()[1].getLineNumber()
					+ " - Unable to connect to the LimeLight");
		}
	}

	// region Get LimeLight Values
	public static double getTX() {
		return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getValue().getDouble();
	}

	public static double getTY() {
		return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getValue().getDouble();
	}

	public static double getTA() {
		return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getValue().getDouble();
	}

	public static double getTL() {
		return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tl").getValue().getDouble();
	}

	public static double getTV() {
		return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getValue().getDouble();
	}

	public static double getTS() {
		return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ts").getValue().getDouble();
	}

	public static double getDistance() {
		double distance_adjust = 0.0;
		double dA = getTA();

		// dTargetHeight = 8.5; // Height in feet (H2)
		// dLimeLightHeight = 4.0; // Height in feet (H1)

		// tan(a1+a2) = (h2-h1) / d
		// d = (h2-h1) / tan(a1+a2)

		// double EQ1 = tan(dA+dA);

		return distance_adjust;
	}

	// public static double getDistance(){
	// double heightOfCamera = 43;
	// double heightOfTarget = 29;
	// double angleOfCamera = -20;
	// double angleofTarget = ty.getDouble(0.0);
	// return (heightOfTarget - heightOfCamera) /
	// Math.tan(Math.toRadians(angleOfCamera + angleofTarget));
	// }

	public static double getSteeringAdjust() {
		double dTX = getTX();
		double dTY = getTY();
		double dTV = getTV();
		double Kp = -0.1;
		double min_command = 0.05;
		double heading_error = -dTX;
		double distance_error = -dTY;
		// float KpDistance = -0.1f; // Proportional control constant for distance
		// float current_distance = Estimate_Distance(); // see the 'Case Study:
		// Estimating Distance'

		if (dTX > 1.0) {
			m_LimelightSteerCommand = Kp * heading_error - min_command;
		} else if (dTX < 1.0) {
			m_LimelightSteerCommand = Kp * heading_error + min_command;
		}
		return m_LimelightSteerCommand;
	}
	// endregion Get LimeLight Values

	/**
	 * Takes snapshots every 0.5 seconds if enabled
	 * 
	 * @param mode - snapshot mode Example: 0: no snapshots 1: two snapshots per
	 *             second
	 */
	public void snapShotMode(int mode) {
		NetworkTableInstance.getDefault().getTable("limelight").getEntry("snapshot").setNumber(mode);
	}

	/**
	 * Sets the stream mode using the limelight and an external webcam
	 * 
	 * @param mode - stream mode Example: 0: Standard - Side-by-side streams if a
	 *             webcam is attached to Limelight 1: PiP Main - The secondary
	 *             camera stream is placed in the lower-right corner of the primary
	 *             camera stream 2: PiP Secondary - The primary camera stream is
	 *             placed in the lower-right corner of the secondary camera stream
	 */
	public void streamMode(int mode) {
		NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(mode);
	}

	/**
	 * Sets the pipeline of the limelight
	 * 
	 * @param pipeline - desired pipeline number btween 0-9 0: Default 9: Driver Cam
	 */
	public void switchPipeLine(int pipeline) {
		NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipeline);
	}

}