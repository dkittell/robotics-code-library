package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.ServoController;

@Autonomous(name = "PoppysPlaytimeRedSide5 (Blocks to Java)")
public class PoppysPlaytimeRedSide5 extends LinearOpMode {

  private DcMotor right_drive;
  private DcMotor left_drive;
  private ServoController ControlHub_ServoController;

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
    double RightTurnieTurn;
    double LeftTurnieTurn;
    double RightBacksmackahat;
    double LeftBacksmackahat;

    right_drive = hardwareMap.get(DcMotor.class, "right_drive");
    left_drive = hardwareMap.get(DcMotor.class, "left_drive");
    ControlHub_ServoController = hardwareMap.get(ServoController.class, "Control Hub");

    // Put initialization blocks here.
    HD_COUNTS_PER_REV = 28;
    DRIVE_GEAR_REDUCTION = 20;
    WHEEL_CIRCUMFERENCE_MM = 90 * Math.PI;
    DRIVE_COUNTS_PER_MM = (HD_COUNTS_PER_REV * DRIVE_GEAR_REDUCTION) / WHEEL_CIRCUMFERENCE_MM;
    DRIVE_COUNTS_PER_IN = 1 * DRIVE_COUNTS_PER_MM;
    right_drive.setDirection(DcMotorSimple.Direction.REVERSE);
    right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    telemetry.addData("Left Position", left_drive.getCurrentPosition());
    telemetry.addData("Right Position", right_drive.getCurrentPosition());
    telemetry.update();
    ControlHub_ServoController.pwmDisable();
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      rightTarget = right_drive.getCurrentPosition() - 20 * DRIVE_COUNTS_PER_IN;
      leftTarget = left_drive.getCurrentPosition() - 20 * DRIVE_COUNTS_PER_IN;
      RightTurnieTurn = right_drive.getCurrentPosition() - 55 * DRIVE_COUNTS_PER_IN;
      LeftTurnieTurn = left_drive.getCurrentPosition() + 55 * DRIVE_COUNTS_PER_IN;
      RightBacksmackahat = right_drive.getCurrentPosition() + 115 * DRIVE_COUNTS_PER_IN;
      LeftBacksmackahat = left_drive.getCurrentPosition() + 115 * DRIVE_COUNTS_PER_IN;
      right_drive.setTargetPosition((int) rightTarget);
      left_drive.setTargetPosition((int) leftTarget);
      right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      right_drive.setPower(1);
      left_drive.setPower(1);
      left_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      right_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      while (opModeIsActive() && (right_drive.isBusy() || left_drive.isBusy())) {
        telemetry.addData("Left Position", left_drive.getCurrentPosition());
        telemetry.addData("Right Position", right_drive.getCurrentPosition());
        telemetry.addData("Stage", "1");
        telemetry.update();
      }
      right_drive.setTargetPosition((int) (RightTurnieTurn + rightTarget));
      left_drive.setTargetPosition((int) (LeftTurnieTurn + leftTarget));
      while (opModeIsActive() && (right_drive.isBusy() || left_drive.isBusy())) {
        telemetry.addData("Left Position", left_drive.getCurrentPosition());
        telemetry.addData("Right Position", right_drive.getCurrentPosition());
        telemetry.addData("Stage", "2");
        telemetry.update();
      }
      right_drive.setTargetPosition((int) (RightBacksmackahat + rightTarget + RightTurnieTurn));
      left_drive.setTargetPosition((int) (LeftBacksmackahat + leftTarget + LeftTurnieTurn));
      while (opModeIsActive() && (right_drive.isBusy() || left_drive.isBusy())) {
        telemetry.addData("Left Position", left_drive.getCurrentPosition());
        telemetry.addData("Right Position", right_drive.getCurrentPosition());
        telemetry.addData("Stage", "3");
        telemetry.update();
      }
      right_drive.setPower(0);
      left_drive.setPower(0);
      HD_COUNTS_PER_REV = 28;
    }
  }
}
