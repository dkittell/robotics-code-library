package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "hi2 (Blocks to Java)")
public class hi2 extends LinearOpMode {

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
    right_drive.setDirection(DcMotorSimple.Direction.REVERSE);
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      while (opModeIsActive()) {
        // Put loop blocks here.
        right_drive.setPower(1);
        left_drive.setPower(1);
        right_drive.setTargetPosition(3);
        left_drive.setTargetPosition(3);
        telemetry.update();
      }
    }
  }
}
