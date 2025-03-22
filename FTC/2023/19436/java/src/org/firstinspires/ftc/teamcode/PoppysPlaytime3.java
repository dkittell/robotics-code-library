package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "PoppysPlaytime3 (Blocks to Java)")
public class PoppysPlaytime3 extends LinearOpMode {

  private DcMotor right_drive;
  private DcMotor left_drive;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    right_drive = hardwareMap.get(DcMotor.class, "right_drive");
    left_drive = hardwareMap.get(DcMotor.class, "left_drive");

    // Put initialization blocks here.
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      right_drive.setPower(1);
      left_drive.setPower(1);
      right_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      left_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      right_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      left_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      right_drive.setTargetPosition(90);
      left_drive.setTargetPosition(90);
      while (opModeIsActive()) {
        // Put loop blocks here.
        telemetry.update();
      }
    }
  }
}
