// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class MotorConstants {

    // region CAN Network
    public static final int can_id_frontLeft = 8;
    public static final int can_id_backLeft = 11;
    public static final int can_id_frontRight = 14;
    public static final int can_id_backRight = 12;
    // endregion CAN Network
  }

  public static class ShooterConstants {

    // region CAN Network
    public static final int can_id_Shooter1 = 5;
    public static final int can_id_Shooter2 = 6;
    public static final int can_id_intake = 9;
    public static final int can_id_indexer = 7;
    public static final int can_id_Pivot = 13;
    // endregion CAN Network

  }

  public static class OperatorConstants {

    public static final int kDriverControllerPort = 0;
  }

  public static class Controller_Buttons_F310_X {

    public static final int axis_LTrigger = 2;
    public static final int axis_RTrigger = 3;
    public static final int axis_lLeftRight = 0;
    public static final int axis_lUpDown = 1;
    public static final int axis_rLeftRight = 4;
    public static final int axis_rUpDown = 5;
    public static final int btn_A = 1;
    public static final int btn_B = 2;
    public static final int btn_Back = 7;
    public static final int btn_LB = 5;
    public static final int btn_LToggle = 9;
    public static final int btn_RB = 6;
    public static final int btn_RToggle = 10;
    public static final int btn_Start = 8;
    public static final int btn_X = 3;
    public static final int btn_Y = 4;
  }

  public static class Controller_Buttons_F310_D {

    public static final int axis_lLeftRight = 0;
    public static final int axis_lUpDown = 1;
    public static final int axis_rLeftRight = 2;
    public static final int axis_rUpDown = 3;
    public static final int btn_A = 2;
    public static final int btn_B = 3;
    public static final int btn_Back = 9;
    public static final int btn_LB = 5;
    public static final int btn_LToggle = 11;
    public static final int btn_LTrigger = 7;
    public static final int btn_RB = 6;
    public static final int btn_RToggle = 12;
    public static final int btn_RTrigger = 8;
    public static final int btn_Start = 10;
    public static final int btn_X = 1;
    public static final int btn_Y = 4;
  }
}
