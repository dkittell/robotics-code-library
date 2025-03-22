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
  public static final int can_id_frontLeft = 3;
  public static final int can_id_rearLeft = 1;
  public static final int can_id_frontRight = 4;
  public static final int can_id_rearRight = 2;
  public static final int can_id_elbow = 5;
  public static final int can_id_extend = 6;
  public static final int can_id_pdh = 7;
  public static final int can_id_intake = 8;
  // public static final int can_id_wrist = 9;
  // public static final int can_id_intakeLeft = 10;
  // public static final int can_id_intakeRight = 11;
  public static final int can_id_ph = 12;
  // endregion CAN Network

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

  public static final double encoderPerRev = 42;
  public static final double gearRatioMecanum = 10.71 / 1.0; // our gear ratio (TO DO: caluclate sprockets also)
  public static final double gearRatioTraction = 10.71 / 1.0; // our gear ratio (TO DO: caluclate sprockets also)
  public static final double wheelDiameterMecanum = 6.0; // in inches (TO DO: verify values)
  public static final double wheelDiameterTraction = 4.0; // in inches (TO DO: verify values)
  // public static final double countsPerInchMecanum =
  //   (encoderPerRev * gearRatioMecanum) / (wheelDiameterMecanum * 3.14159);
  // public static final double countsPerInchTraction =
  //   (encoderPerRev * gearRatioTraction) / (wheelDiameterTraction * 3.14159);

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
