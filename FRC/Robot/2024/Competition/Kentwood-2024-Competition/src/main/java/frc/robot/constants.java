package frc.robot;

public class constants {

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

  // region CAN Network
  public static final int can_id_frontLeft = 8;
  public static final int can_id_rearLeft = 11;
  public static final int can_id_frontRight = 14;
  public static final int can_id_rearRight = 12;
  public static final int can_id_Shooter1 = 5;
  public static final int can_id_Shooter2 = 6;
  public static final int can_id_pdh = 1;
  public static final int can_id_intake = 9;
  public static final int can_id_ph = 20;
  public static final int can_id_indexer = 7;
  public static final int can_id_Climber = 3;
  public static final int can_id_Pivot = 13;
  // endregion CAN Network

  // region PID Variables
  public static double kpFrontLeft = 6e-5;
  public static double kiFrontLeft = 0;
  public static double kdFrontLeft = 0;
  public static double kizFrontLeft = 0;
  public static double kffFrontLeft = 0.000015;
  public static double kMaxOutputFrontLeft = 1;
  public static double kMinOutputFrontLeft = -1;
  public static double maxRPMFrontLeft = 5700;

  public static double kpFrontRight = 6e-5;
  public static double kiFrontRight = 0;
  public static double kdFrontRight = 0;
  public static double kizFrontRight = 0;
  public static double kffFrontRight = 0.000015;
  public static double kMaxOutputFrontRight = 1;
  public static double kMinOutputFrontRight = -1;
  public static double maxRPMFrontRight = 5700;

  public static double kpBackLeft = 6e-5;
  public static double kiBackLeft = 0;
  public static double kdBackLeft = 0;
  public static double kizBackLeft = 0;
  public static double kffBackLeft = 0.000015;
  public static double kMaxOutputBackLeft = 1;
  public static double kMinOutputBackLeft = -1;
  public static double maxRPMBackLeft = 5700;

  public static double kpBackRight = 6e-5;
  public static double kiBackRight = 0;
  public static double kdBackRight = 0;
  public static double kizBackRight = 0;
  public static double kffBackRight = 0.000015;
  public static double kMaxOutputBackRight = 1;
  public static double kMinOutputBackRight = -1;
  public static double maxRPMBackRight = 5700;

  // endregion PID Variables

  // region Controller Definition
  public static final int js_Driver = 0; // Driver
  public static final int js_Operator = 1; // Operator
  // endregion Controller Definition

  // region DeadbandConstants
  // region js_Driver Left Deadband
  public static final double js_Driver_L_UpDown_Deadband_Low = 0.15;
  public static final double js_Driver_L_UpDown_Deadband_High = dForwardSpeed; // 0.90
  public static final double js_Driver_L_UpDown_Deadband_Max = 1.00;
  public static final double js_Driver_L_LeftRight_Deadband_Low = 0.15;
  public static final double js_Driver_L_LeftRight_Deadband_High =
    dForwardSpeed; // 0.90
  public static final double js_Driver_L_LeftRight_Deadband_Max = 1.00;
  // endregion js_Driver Left Deadband

  // region js_Driver Right Deadband
  public static final double js_Driver_R_UpDown_Deadband_Low = 0.15;
  // public static final double js_Driver_R_UpDown_Deadband_High = 0.90;
  public static final double js_Driver_R_UpDown_Deadband_High = dForwardSpeed;
  public static final double js_Driver_R_UpDown_Deadband_Max = 1.00;
  public static final double js_Driver_R_LeftRight_Deadband_Low = 0.15;
  public static final double js_Driver_R_LeftRight_Deadband_High = dTurnSpeed;
  public static final double js_Driver_R_LeftRight_Deadband_Max = 1.00;
  // endregion js_Driver Right Deadband

  // region js_Operator Left Deadband
  public static final double js_Operator_L_UpDown_Deadband_Low = 0.15;
  public static final double js_Operator_L_UpDown_Deadband_High = 0.90;
  public static final double js_Operator_L_UpDown_Deadband_Max = 1.00;
  public static final double js_Operator_L_LeftRight_Deadband_Low = 0.15;
  public static final double js_Operator_L_LeftRight_Deadband_High = 0.50;
  public static final double js_Operator_L_LeftRight_Deadband_Max = 1.00;
  // endregion js_Operator Left Deadband

  // region js_Operator Right Deadband
  public static final double js_Operator_R_UpDown_Deadband_Low = 0.15;
  public static final double js_Operator_R_UpDown_Deadband_High = 0.90;
  public static final double js_Operator_R_UpDown_Deadband_Max = 1.00;
  public static final double js_Operator_R_LeftRight_Deadband_Low = 0.15;
  public static final double js_Operator_R_LeftRight_Deadband_High = 0.50;
  public static final double js_Operator_R_LeftRight_Deadband_Max = 1.00;
  // endregion js_Operator Right Deadband
  // endregion DeadbandConstants

  static final double COUNTS_PER_MOTOR_REV = 42; // NEO Brushless 1650
  static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV / 70); // Inch to encoder count

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
  public static final int axisXB_LTrigger = 2;
  public static final int axisXB_RTrigger = 3;
  public static final int axisXB_lLeftRight = 0;
  public static final int axisXB_lUpDown = 1;
  public static final int axisXB_rLeftRight = 4;
  public static final int axisXB_rUpDown = 5;
  public static final int btnXB_A = 1;
  public static final int btnXB_B = 2;
  public static final int btnXB_Back = 7;
  public static final int btnXB_LB = 5;
  public static final int btnXB_LToggle = 9;
  public static final int btnXB_RB = 6;
  public static final int btnXB_RToggle = 10;
  public static final int btnXB_Start = 8;
  public static final int btnXB_X = 3;
  public static final int btnXB_Y = 4;
  // endregion XBox Controller Layout

  // region Logitech Gamepad F310 Controller Layout - X Switch
  public static final int axisX_LTrigger = 2;
  public static final int axisX_RTrigger = 3;
  public static final int axisX_lLeftRight = 0;
  public static final int axisX_lUpDown = 1;
  public static final int axisX_rLeftRight = 4;
  public static final int axisX_rUpDown = 5;
  public static final int btnX_A = 1;
  public static final int btnX_B = 2;
  public static final int btnX_Back = 7;
  public static final int btnX_LB = 5;
  public static final int btnX_LToggle = 9;
  public static final int btnX_RB = 6;
  public static final int btnX_RToggle = 10;
  public static final int btnX_Start = 8;
  public static final int btnX_X = 3;
  public static final int btnX_Y = 4;
  // endregion Logitech Gamepad F310 Controller Layout - X Switch

  // region Logitech Gamepad F310 Controller Layout - D Switch
  public static final int axisD_lLeftRight = 0;
  public static final int axisD_lUpDown = 1;
  public static final int axisD_rLeftRight = 2;
  public static final int axisD_rUpDown = 3;
  public static final int btnD_A = 2;
  public static final int btnD_B = 3;
  public static final int btnD_Back = 9;
  public static final int btnD_LB = 5;
  public static final int btnD_LToggle = 11;
  public static final int btnD_LTrigger = 7;
  public static final int btnD_RB = 6;
  public static final int btnD_RToggle = 12;
  public static final int btnD_RTrigger = 8;
  public static final int btnD_Start = 10;
  public static final int btnD_X = 1;
  public static final int btnD_Y = 4;
  // endregion Logitech Gamepad F310 Controller Layout - D Switch

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

  // region Logitech Extreme 3D Pro
  public static final int btnEP_Trigger = 1;
  public static final int btnEP_Thumb = 2;
  public static final int btnEP_3 = 3;
  public static final int btnEP_4 = 4;
  public static final int btnEP_5 = 5;
  public static final int btnEP_6 = 6;
  public static final int btnEP_7 = 7;
  public static final int btnEP_8 = 8;
  public static final int btnEP_9 = 9;
  public static final int btnEP_10 = 10;
  public static final int btnEP_11 = 11;
  public static final int btnEP_12 = 12;
  public static final int axisEP_LeftRight = 1;
  public static final int axisEP_UpDown = 2;
  public static final int axisEP_Twist = 3;
  public static final int axisEP_Bottom = 4;
  public static final int axisEP_Hat_UpDown = 5;
  public static final int axisEP_Hat_LeftRight = 6;
  // endregion Logitech Extreme 3D Pro
  // endregion Controller Layout Variables

  // region Pneumatics
  public static final int p_ds_Drivetrain_Forward_id = 0;
  public static final int p_ds_Drivetrain_Reverse_id = 1;
  public static final int p_ds_Claw_Forward_id = 2;
  public static final int p_ds_Claw_Reverse_id = 3;
  public static final int p_as_Claw_id = 1;
  public static final int p_as_Drivetrain_id = 0;
  // endregion Pneumatics

  public static final int n_pdh_channels = 24;
}
