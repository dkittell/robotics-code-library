package frc.robot;

/**
 * Variables
 */
public class frc7784_Constants {

  public static int tAuton = 0;

  public static final double dForwardSpeed = 0.55;
  // public static final double dForwardSpeed = 0.90;

  // region Limelight
  // 0: Sets the mode to what is in the current pipeline
  // 1: Turns off the LEDs
  // 2: Blink mode on LEDs
  // 3: Turns on the LEDs
  public static final int ll_pipeline = 0;
  public static final int ll_off = 1;
  public static final int ll_blink = 2;
  public static final int ll_on = 3;

  // endregion Limelight

  // region Pneumatics
  public static final int pcmPort = 0;
  public static final int pIntakeIn = 4;
  public static final int pIntakeOut = 5;
  public static final int pRachet = 3;
  public static final int pRachetIn = 6;
  public static final int pRachetOut = 7;

  // endregion Pneumatics

  // region Motor Assignments
  // region Venom Motors
  public static final int m_frontLeft = 2;
  public static final int m_rearLeft = 1;
  public static final int m_frontRight = 4;
  public static final int m_rearRight = 3;
  public static final int m_Shooter = 5;
  public static final double m_Shooter_Speed = 0.4;
  public static final double m_Shooter_Speed_Long = 0.7;
  public static final double m_Shooter_Speed_Reverse = -0.2;

  // endregion Venom Motors

  // region Talon SRX Motors
  public static final int m_Uptake = 1;
  public static final int m_Climber01 = 4; // Encoder
  // public static final int m_Climber02 = 5; // Rachet
  public static final int m_ControlPanel = 10;
  public static final int m_Intake = 0;
  public static final int m_ClimberPosition = 11;
  // endregion Talon SRX Motors
  // endregion Motor Assignments

  public static final int m_pdp = 0;

  // region Controller Definition
  public static final int js1 = 0; // Driver
  public static final int js2 = 1; // Operator
  // endregion Controller Definition

  // region Dead Band Variables
  // JS1 Left Deadband
  public static final double js1_L_UpDown_Deadband_Low = 0.15;
  public static final double js1_L_UpDown_Deadband_High = dForwardSpeed; // 0.90
  public static final double js1_L_UpDown_Deadband_Max = 1.00;
  public static final double js1_L_LeftRight_Deadband_Low = 0.15;
  public static final double js1_L_LeftRight_Deadband_High = dForwardSpeed; // 0.90
  public static final double js1_L_LeftRight_Deadband_Max = 1.00;
  // JS1 Right Deadband
  public static final double js1_R_UpDown_Deadband_Low = 0.15;
  public static final double js1_R_UpDown_Deadband_High = 0.90;
  public static final double js1_R_UpDown_Deadband_Max = 1.00;
  public static final double js1_R_LeftRight_Deadband_Low = 0.15;
  public static final double js1_R_LeftRight_Deadband_High = 0.60;
  public static final double js1_R_LeftRight_Deadband_Max = 1.00;
  // JS2 Left Deadband
  public static final double js2_L_UpDown_Deadband_Low = 0.15;
  public static final double js2_L_UpDown_Deadband_High = 0.90;
  public static final double js2_L_UpDown_Deadband_Max = 1.00;
  public static final double js2_L_LeftRight_Deadband_Low = 0.15;
  public static final double js2_L_LeftRight_Deadband_High = 0.50;
  public static final double js2_L_LeftRight_Deadband_Max = 1.00;
  // JS2 Right Deadband
  public static final double js2_R_UpDown_Deadband_Low = 0.15;
  public static final double js2_R_UpDown_Deadband_High = 0.90;
  public static final double js2_R_UpDown_Deadband_Max = 1.00;
  public static final double js2_R_LeftRight_Deadband_Low = 0.15;
  public static final double js2_R_LeftRight_Deadband_High = 0.50;
  public static final double js2_R_LeftRight_Deadband_Max = 1.00;
  // endregion Dead Band Variables

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

}
