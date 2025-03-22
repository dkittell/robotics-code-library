package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "IntakeTest5 (Blocks to Java)")
public class IntakeTest5 extends LinearOpMode {

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
        intake_motor.setPower(0.5);
        if (gamepad1.a) {
          lift_motor.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        if (gamepad1.b) {
          lift_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        if (gamepad1.x) {
          lift_motor.setPower(0);
        }
      }
    }
  }
}
