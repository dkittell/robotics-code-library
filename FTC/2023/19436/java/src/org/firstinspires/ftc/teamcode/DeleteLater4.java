package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name = "DeleteLater4 (Blocks to Java)")
public class DeleteLater4 extends LinearOpMode {

  private CRServo carousel_motor;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    carousel_motor = hardwareMap.get(CRServo.class, "carousel_motor");

    // Put initialization blocks here.
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      while (opModeIsActive()) {
        // Put loop blocks here.
        if (gamepad1.dpad_right) {
          carousel_motor.setPower(1);
        } else {
          carousel_motor.setPower(0);
        }
        telemetry.update();
      }
    }
  }

  /**
   * Describe this function...
   */
  private double runOpMode2() {
    waitForStart();
    if (opModeIsActive()) {
      while (opModeIsActive()) {
        telemetry.update();
      }
    }
    return carousel_motor.getPower();
  }
}
