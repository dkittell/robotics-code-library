package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "PoppysPlaytime12 (Blocks to Java)")
public class PoppysPlaytime12 extends LinearOpMode {

  private DcMotor left_drive;
  private DcMotor right_drive;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    int HD_COUNTS_PER_REV;
    int DRIVE_GEAR_REDUCTION;
    double WHEEL_CIRCUMFERENCE_MM;
    double DRIVE_COUNTS_PER_MM;
    double DRIVE_COUNTS_PER_IN;
    double rightTarget;
    double leftTarget;

    left_drive = hardwareMap.get(DcMotor.class, "left_drive");
    right_drive = hardwareMap.get(DcMotor.class, "right_drive");

    // Put initialization blocks here.
    HD_COUNTS_PER_REV = 28;
    DRIVE_GEAR_REDUCTION = 20;
    WHEEL_CIRCUMFERENCE_MM = 90 * Math.PI;
    DRIVE_COUNTS_PER_MM = (HD_COUNTS_PER_REV * DRIVE_GEAR_REDUCTION) / WHEEL_CIRCUMFERENCE_MM;
    DRIVE_COUNTS_PER_IN = DRIVE_COUNTS_PER_MM * 25.4;
    left_drive.setDirection(DcMotorSimple.Direction.REVERSE);
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      rightTarget = right_drive.getCurrentPosition() + 30 * DRIVE_COUNTS_PER_IN;
      leftTarget = left_drive.getCurrentPosition() + 15 * DRIVE_COUNTS_PER_IN;
      right_drive.setTargetPosition((int) rightTarget);
      left_drive.setTargetPosition((int) leftTarget);
      right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      right_drive.setPower(1);
      left_drive.setPower(1);
      while (opModeIsActive() && (left_drive.isBusy() || right_drive.isBusy())) {
      }
      right_drive.setPower(0);
      left_drive.setPower(0);
    }
    HD_COUNTS_PER_REV = 28;
  }
}
