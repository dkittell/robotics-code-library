package org.firstinspires.ftc.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@Disabled
@TeleOp(name = "Servo_Tester (Blocks to Java)")
public class Servo_Tester extends LinearOpMode {

  private CRServo testservo;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    testservo = hardwareMap.get(CRServo.class, "testservo");

    // Put initialization blocks here.
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      while (opModeIsActive()) {
        // Put loop blocks here.
        if (gamepad1.a) {
          testservo.setPower(1);
        } else if (gamepad1.x) {
          testservo.setPower(-1);
        } else {
          testservo.setPower(0);
        }
        telemetry.update();
      }
    }
  }
}
