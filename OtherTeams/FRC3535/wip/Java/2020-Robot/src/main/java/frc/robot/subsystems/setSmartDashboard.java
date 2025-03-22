package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.frc3535_Variables;
// import frc.robot.constants.frc3535_Joystick;

public class setSmartDashboard extends Subsystem {

	public setSmartDashboard() {
	}

	@Override
	public void initDefaultCommand() {
	}

  // region Controller Definition
  private final static Joystick js1 = new Joystick(frc3535_Variables.js1);
	// endregion Controller Definition
	private final static SendableChooser<String> c_Auton = new SendableChooser<>();
	private final static SendableChooser<String> c_Auton_Delay = new SendableChooser<>();
	private final static SendableChooser<String> c_DriveMode = new SendableChooser<>();

	public static void setData() {
		c_Auton.setDefaultOption("Default Auton", "Default");
		c_Auton.addOption("Left", "Left");
		c_Auton.addOption("Center", "Center");
		c_Auton.addOption("Right", "Right");
		SmartDashboard.putData("Auton choices", c_Auton);

		c_Auton_Delay.setDefaultOption("None", "None");
		c_Auton_Delay.addOption("1 second", "1 second");
		c_Auton_Delay.addOption("2 second", "2 second");
		c_Auton_Delay.addOption("3 second", "3 second");
		c_Auton_Delay.addOption("4 second", "4 second");
		c_Auton_Delay.addOption("5 second", "5 second");
		c_Auton_Delay.addOption("6 second", "6 second");
		SmartDashboard.putData("Auton Delay", c_Auton_Delay);

		c_DriveMode.setDefaultOption("Arcade", "Arcade");
		c_DriveMode.addOption("Tank", "Tank");
		SmartDashboard.putData("Drive Mode", c_DriveMode);

		SmartDashboard.putNumber("Battery_Voltage", edu.wpi.first.wpilibj.RobotController.getBatteryVoltage());
		SmartDashboard.putString("RoboRio", GetMacAddress.getRIOMAC());
		SmartDashboard.putNumber("js1_L_Actual", js1.getY());
		SmartDashboard.putNumber("js1_R_Actual", js1.getZ());
	
	//	SmartDashboard.putNumber("js1_L_Speed", frc3535_Joystick.getDeadBand(1, 1, 1));
	//	SmartDashboard.putNumber("js1_R_Speed", frc3535_Joystick.getDeadBand(1, 2, 2));
	}

	public static String getAuton() {
		return c_Auton.getSelected();
	}

	public static String getAutonDelay() {
		return c_Auton_Delay.getSelected();
	}

	public static String getDriveMode() {
		return c_DriveMode.getSelected();
	}

}