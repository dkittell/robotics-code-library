package frc.robot;

public class constants {

    public static int tAuton = 0;

    public static final double dForwardSpeed = 0.70;

    public static final double dTurnSpeed = 0.70;

    // region CAN Network
    public static final int can_id_frontLeft = 3;
    public static final int can_id_rearLeft = 1;
    public static final int can_id_frontRight = 4;
    public static final int can_id_rearRight = 2;
    public static final int can_id_pdh = 7;
    // endregion CAN Network

    // region Controller Definition
    public static final int js_Driver = 0; // Driver
    public static final int js_Operator = 1; // Operator
    // endregion Controller Definition

    // region DeadbandConstants

    // region js_Driver Left Deadband
    public static final double js_Driver_L_UpDown_Deadband_Low = 0.15;
    public static final double js_Driver_L_UpDown_Deadband_High = dForwardSpeed;
    public static final double js_Driver_L_UpDown_Deadband_Max = 1.00;
    public static final double js_Driver_L_LeftRight_Deadband_Low = 0.15;
    public static final double js_Driver_L_LeftRight_Deadband_High = dForwardSpeed;
    public static final double js_Driver_L_LeftRight_Deadband_Max = 1.00;
    // endregion js_Driver Left Deadband

    // region js_Driver Right Deadband
    public static final double js_Driver_R_UpDown_Deadband_Low = 0.15;
    public static final double js_Driver_R_UpDown_Deadband_High = dForwardSpeed;
    public static final double js_Driver_R_UpDown_Deadband_Max = 1.00;
    public static final double js_Driver_R_LeftRight_Deadband_Low = 0.15;
    public static final double js_Driver_R_LeftRight_Deadband_High = dTurnSpeed;
    public static final double js_Driver_R_LeftRight_Deadband_Max = 1.00;
    // endregion js_Driver Right Deadband

    // endregion DeadbandConstants

    // region Encoder Constants

    public static final double encoderPerRev = 42;
    public static final double gearRatioMecanum = 10.71 / 1.0; // our gear ratio (TO DO: caluclate sprockets also)
    public static final double wheelDiameterMecanum = 6.0; // in inches (TO DO: verify values)
    public static final double countsPerInchMecanum = (encoderPerRev * gearRatioMecanum)
            / (wheelDiameterMecanum * 3.14159);

    // endregion Encoder Constants

    // region Controller Layout Variables

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
}