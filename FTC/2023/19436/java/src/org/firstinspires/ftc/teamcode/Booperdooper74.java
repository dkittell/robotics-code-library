package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Booperdooper74 (Blocks to Java)")
public class Booperdooper74 extends LinearOpMode {

  private DcMotor lift_motor;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    lift_motor = hardwareMap.get(DcMotor.class, "lift_motor");

    // Put initialization blocks here.
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      while (opModeIsActive()) {
        // Put loop blocks here.
        while (gamepad1.dpad_up) {
          lift_motor.setPower(0.5);
          lift_motor.setDirection(DcMotorSimple.Direction.FORWARD);
          lift_motor.setTargetPosition(20);
        }
        while (gamepad1.dpad_down) {
          lift_motor.setPower(0.5);
          lift_motor.setDirection(DcMotorSimple.Direction.REVERSE);
          lift_motor.setTargetPosition(0);
        }
        telemetry.update();
      }
    }
  }
}
