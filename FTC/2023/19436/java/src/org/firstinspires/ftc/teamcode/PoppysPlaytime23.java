package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "PoppysPlaytime23 (Blocks to Java)")
public class PoppysPlaytime23 extends LinearOpMode {

  private DcMotor right_drive;
  private DcMotor left_drive;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    int HD_COUNTS_PER_REV;
    double RightBacksmackahat;
    double LeftBacksmackahat;
    int DRIVE_GEAR_REDUCTION;
    int rightTarget;
    double RightTurnieTurn;
    int leftTarget;
    double LeftTurnieTurn;
    double WHEEL_CIRCUMFERENCE_MM;
    double DRIVE_COUNTS_PER_MM;
    double DRIVE_COUNTS_PER_IN;

    right_drive = hardwareMap.get(DcMotor.class, "right_drive");
    left_drive = hardwareMap.get(DcMotor.class, "left_drive");

    // Put initialization blocks here.
    HD_COUNTS_PER_REV = 28;
    DRIVE_GEAR_REDUCTION = 20;
    WHEEL_CIRCUMFERENCE_MM = 90 * Math.PI;
    DRIVE_COUNTS_PER_MM = (HD_COUNTS_PER_REV * DRIVE_GEAR_REDUCTION) / WHEEL_CIRCUMFERENCE_MM;
    DRIVE_COUNTS_PER_IN = DRIVE_COUNTS_PER_MM * 25.4;
    right_drive.setDirection(DcMotorSimple.Direction.REVERSE);
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      rightTarget = (int) (right_drive.getCurrentPosition() + 20 * DRIVE_COUNTS_PER_IN);
      leftTarget = (int) (left_drive.getCurrentPosition() + 20 * DRIVE_COUNTS_PER_IN);
      RightTurnieTurn = right_drive.getCurrentPosition() + 50 * DRIVE_COUNTS_PER_IN;
      LeftTurnieTurn = left_drive.getCurrentPosition() - 50 * DRIVE_COUNTS_PER_IN;
      RightBacksmackahat = right_drive.getCurrentPosition() - 50 * DRIVE_COUNTS_PER_IN;
      LeftBacksmackahat = left_drive.getCurrentPosition() - 50 * DRIVE_COUNTS_PER_IN;
      right_drive.setTargetPosition(rightTarget);
      left_drive.setTargetPosition(leftTarget);
      right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      right_drive.setPower(1);
      left_drive.setPower(1);
      left_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      right_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      while (opModeIsActive() && (right_drive.isBusy() || left_drive.isBusy())) {
      }
      right_drive.setTargetPosition((int) (RightTurnieTurn + rightTarget));
      left_drive.setTargetPosition((int) (LeftTurnieTurn + leftTarget));
      while (opModeIsActive() && (right_drive.isBusy() || left_drive.isBusy())) {
      }
      right_drive.setTargetPosition((int) (RightBacksmackahat + rightTarget + RightTurnieTurn));
      left_drive.setTargetPosition((int) (LeftBacksmackahat + leftTarget + LeftTurnieTurn));
      while (opModeIsActive() && (right_drive.isBusy() || left_drive.isBusy())) {
      }
      right_drive.setPower(0);
      left_drive.setPower(0);
      HD_COUNTS_PER_REV = 28;
    }
  }
}
