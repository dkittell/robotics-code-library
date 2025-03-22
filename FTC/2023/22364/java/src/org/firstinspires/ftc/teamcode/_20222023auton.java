package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "_20222023auton (Blocks to Java)", preselectTeleOp = "POWERPLAY-Starter-Bot-Teleop")
public class _20222023auton extends LinearOpMode {

  private DcMotor left_drive;
  private DcMotor leftarmmotor;
  private DcMotor rightarmmotor;
  private DcMotor right_drive;

  double driveCountsPermm;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    left_drive = hardwareMap.get(DcMotor.class, "left_drive");
    leftarmmotor = hardwareMap.get(DcMotor.class, "left arm motor");
    rightarmmotor = hardwareMap.get(DcMotor.class, "right arm motor");
    right_drive = hardwareMap.get(DcMotor.class, "right_drive");

    // Put initialization blocks here.
    left_drive.setDirection(DcMotorSimple.Direction.REVERSE);
    leftarmmotor.setDirection(DcMotorSimple.Direction.REVERSE);
    driveCountsPermm = (20.1524 * 28) / (90 * Math.PI);
    waitForStart();
    goToPositionmm(-1000, -1000);
    if (opModeIsActive()) {
      // Put run blocks here.
      while (opModeIsActive()) {
        // Put loop blocks here.
        telemetry.update();
      }
    }
    leftarmmotor.setPower(0.1);
    rightarmmotor.setPower(0.1);
  }

  /**
   * Describe this function...
   */
  private void goToPositionmm(int lDist, int rDist) {
    right_drive.setTargetPosition((int) (right_drive.getCurrentPosition() + rDist * driveCountsPermm));
    left_drive.setTargetPosition((int) (left_drive.getCurrentPosition() + lDist * driveCountsPermm));
    right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    right_drive.setPower(-0.25);
    left_drive.setPower(-0.25);
    while (right_drive.isBusy() || left_drive.isBusy()) {
    }
    right_drive.setPower(0);
    left_drive.setPower(0);
  }
}
