package frc.robot;

public class Constants {
	public static int tAuton = 0;

	// public static final double dForwardSpeed = 0.55;
	// public static final double dForwardSpeed = 0.65;
	public static final double dForwardSpeed = 0.70;
	// public static final double dForwardSpeed = 0.80;
	// public static final double dForwardSpeed = 0.90;

	// public static final double dTurnSpeed = 0.50;
	// public static final double dTurnSpeed = 0.60;
	public static final double dTurnSpeed = 0.70;
	// public static final double dTurnSpeed = 0.80;

	// region Motor to SPARK MAX Assignments
	static final int m_left01Motor = 2;
	static final int m_left02Motor = 5;
	static final int m_right01Motor = 4;
	static final int m_right02Motor = 3;
	// endregion Motor to SPARK MAX Assignments

	// region Controller Definition
	public static final int js1 = 0; // Driver
	public static final int js2 = 1; // Operator
	// endregion Controller Definition

	// region DeadbandConstants
	// region JS1 Left Deadband
	public static final double js1_L_UpDown_Deadband_Low = 0.15;
	public static final double js1_L_UpDown_Deadband_High = dForwardSpeed; // 0.90
	public static final double js1_L_UpDown_Deadband_Max = 1.00;
	public static final double js1_L_LeftRight_Deadband_Low = 0.15;
	public static final double js1_L_LeftRight_Deadband_High = dForwardSpeed; // 0.90
	public static final double js1_L_LeftRight_Deadband_Max = 1.00;
	// endregion JS1 Left Deadband

	// region JS1 Right Deadband
	public static final double js1_R_UpDown_Deadband_Low = 0.15;
	// public static final double js1_R_UpDown_Deadband_High = 0.90;
	public static final double js1_R_UpDown_Deadband_High = dForwardSpeed;
	public static final double js1_R_UpDown_Deadband_Max = 1.00;
	public static final double js1_R_LeftRight_Deadband_Low = 0.15;
	public static final double js1_R_LeftRight_Deadband_High = dTurnSpeed;
	public static final double js1_R_LeftRight_Deadband_Max = 1.00;
	// endregion JS1 Right Deadband

	// region JS2 Left Deadband
	public static final double js2_L_UpDown_Deadband_Low = 0.15;
	public static final double js2_L_UpDown_Deadband_High = 0.90;
	public static final double js2_L_UpDown_Deadband_Max = 1.00;
	public static final double js2_L_LeftRight_Deadband_Low = 0.15;
	public static final double js2_L_LeftRight_Deadband_High = 0.50;
	public static final double js2_L_LeftRight_Deadband_Max = 1.00;
	// endregion JS2 Left Deadband

	// region JS2 Right Deadband
	public static final double js2_R_UpDown_Deadband_Low = 0.15;
	public static final double js2_R_UpDown_Deadband_High = 0.90;
	public static final double js2_R_UpDown_Deadband_Max = 1.00;
	public static final double js2_R_LeftRight_Deadband_Low = 0.15;
	public static final double js2_R_LeftRight_Deadband_High = 0.50;
	public static final double js2_R_LeftRight_Deadband_Max = 1.00;
	// endregion JS2 Right Deadband
	// endregion DeadbandConstants

	// region Controller Layout Variables
	// region PXN
	public static final int btnPXND_A = 1;
	public static final int btnPXND_B = 2;
	public static final int btnPXND_X = 3;
	public static final int btnPXND_Y = 4;
	public static final int btnPXND_L1LB = 5;
	public static final int btnPXND_RB = 6;
	public static final int btnPXND_Share = 7;
	public static final int btnPXND_Options = 8;
	public static final int btnPXND_L3 = 9;
	public static final int btnPXND_R3 = 10;

	public static final int axisPXND_lUpDown = 1;
	public static final int axisPXND_lLeftRight = 0;
	public static final int axisPXND_L2LT = 2;
	public static final int axisPXND_R2RT = 3;
	// endregion PXN

	// region XBox Controller Layout
	public static final int btnXB_X = 3;
	public static final int btnXB_A = 1;
	public static final int btnXB_B = 2;
	public static final int btnXB_Y = 4;
	public static final int btnXB_LB = 5;
	public static final int btnXB_RB = 6;
	public static final int btnXB_Back = 7;
	public static final int btnXB_Start = 8;
	public static final int btnXB_LToggle = 9;
	public static final int btnXB_RToggle = 10;

	public static final int axisXB_lLeftRight = 0;
	public static final int axisXB_lUpDown = 1;
	public static final int axisXB_LTrigger = 2;
	public static final int axisXB_RTrigger = 3;
	public static final int axisXB_rLeftRight = 4;
	public static final int axisXB_rUpDown = 5;
	// endregion XBox Controller Layout

	// region Logitech Gamepad F310 Controller Layout - D Switch
	public static final int btnD_X = 1;
	public static final int btnD_A = 2;
	public static final int btnD_B = 3;
	public static final int btnD_Y = 4;
	public static final int btnD_LB = 5;
	public static final int btnD_RB = 6;
	public static final int btnD_LT = 7;
	public static final int btnD_RT = 8;
	public static final int btnD_Back = 9;
	public static final int btnD_Start = 10;
	public static final int btnD_LToggle = 11;
	public static final int btnD_RToggle = 12;

	public static final int axisD_lUpDown = 1;
	public static final int axisD_lLeftRight = 0;
	public static final int axisD_rUpDown = 3;
	public static final int axisD_rLeftRight = 2;
	// endregion Logitech Gamepad F310 Controller Layout - D Switch

	// region Logitech Gamepad F310 Controller Layout - X Switch
	public static final int btnX_X = 3;
	public static final int btnX_A = 1;
	public static final int btnX_B = 2;
	public static final int btnX_Y = 4;
	public static final int btnX_LB = 5;
	public static final int btnX_RB = 6;
	public static final int btnX_Back = 7;
	public static final int btnX_Start = 8;
	public static final int btnX_LToggle = 9;
	public static final int btnX_RToggle = 10;

	public static final int axisX_lUpDown = 1;
	public static final int axisX_lLeftRight = 0;
	public static final int axisX_rUpDown = 5;
	public static final int axisX_rLeftRight = 4;
	// endregion Logitech Gamepad F310 Controller Layout - X Switch

	// region Logitech Attack 3 J - UJ18
	public static final int btnAK_Trigger = 1;
	public static final int btnAK_2 = 2;
	public static final int btnAK_3 = 3;
	public static final int btnAK_4 = 4;
	public static final int btnAK_5 = 5;
	public static final int btnAK_6 = 6;
	public static final int btnAK_7 = 7;
	public static final int btnAK_8 = 8;
	public static final int btnAK_9 = 9;
	public static final int btnAK_10 = 10;
	public static final int btnAK_11 = 11;
	public static final int axisAK_UpDown = 0;
	public static final int axisAK_LeftRight = 1;
	// endregion Logitech Attack 3 J - UJ18
	// endregion Controller Layout Variables

	// region Climber
	static final int m_Climber01 = 8;
	static final int m_Climber02 = 9;

	static final int s_RatchetPull = 2;

	// region Climber Speed
	public static final double climb_up = -0.5;
	public static final double climb_down = 0.4;
	// endregion Climber Speed
	// endregion Climber

	// region Intake
	static final int m_Intake = 10;

	static final int s_IntakePush = 3;
	// endregion Intake
}
