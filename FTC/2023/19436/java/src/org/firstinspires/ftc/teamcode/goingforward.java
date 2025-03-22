package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "goingforward (Blocks to Java)")
public class goingforward extends LinearOpMode {

  private DcMotor left_drive;
  private DcMotor right_drive;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    float gpLeftY;
    float gpRightY;

    left_drive = hardwareMap.get(DcMotor.class, "left_drive");
    right_drive = hardwareMap.get(DcMotor.class, "right_drive");

    // Put initialization blocks here.
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      while (opModeIsActive()) {
        // Put loop blocks here.
        telemetry.update();
        // Get value of up/down of left and right sticks
        gpLeftY = gamepad1.left_stick_y;
        gpRightY = gamepad1.right_stick_y;
        // If values are small, make them zero
        if (Math.abs(gpLeftY) <= 0.2) {
          gpLeftY = 0;
        }
        if (Math.abs(gpRightY) <= 0.2) {
          gpRightY = 0;
        }
        // Make left motor go backwards
        gpLeftY = gpLeftY * -1;
        // Make motors mov
        left_drive.setPower(gpLeftY);
        right_drive.setPower(gpRightY);
      }
    }
  }
}
