package frc.robot;

/**
 * Const
 */
public class Const {

	// public static final double TRACK_WIDTH_INCHES = 24.5;

	public static final double dWheelRadius = 3.0; // inches
	public static final double dWheelDiameter = dWheelRadius * 2; // inches
	public static final double dWheelCircumference = Math.PI * dWheelDiameter;

	// public static final double TICKS_TO_INCHES_RATIO = (2 * Math.PI * 2) / 512;
	public static final double TICKS_TO_METERS_RATIO = dWheelCircumference / 7;

}
