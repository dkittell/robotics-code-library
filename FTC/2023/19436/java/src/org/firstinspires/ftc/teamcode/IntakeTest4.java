package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "IntakeTest4 (Blocks to Java)")
public class IntakeTest4 extends LinearOpMode {

  private DcMotor intake_motor;
  private DcMotor lift_motor;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    intake_motor = hardwareMap.get(DcMotor.class, "intake_motor");
    lift_motor = hardwareMap.get(DcMotor.class, "lift_motor");

    // Put initialization blocks here.
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      while (opModeIsActive()) {
        // Put loop blocks here.
        if (gamepad1.a) {
          intake_motor.setPower(1);
        } else {
          lift_motor.setPower(0);
        }
        telemetry.update();
      }
    }
  }
}
