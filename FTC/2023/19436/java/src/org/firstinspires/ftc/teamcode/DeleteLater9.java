package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "DeleteLater9 (Blocks to Java)")
public class DeleteLater9 extends LinearOpMode {

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
        while (gamepad1.dpad_right) {
          carousel_motor.setDirection(DcMotorSimple.Direction.FORWARD);
          carousel_motor.setPower(1);
          carousel_motor.setDirection(DcMotorSimple.Direction.REVERSE);
          carousel_motor.setPower(-1);
        }
        carousel_motor.setPower(0);
        telemetry.update();
      }
    }
  }
}
