package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Booperdooper72 (Blocks to Java)")
public class Booperdooper72 extends LinearOpMode {

  private DcMotor right_drive;
  private DcMotor lift_motor;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    right_drive = hardwareMap.get(DcMotor.class, "right_drive");
    lift_motor = hardwareMap.get(DcMotor.class, "lift_motor");

    // Put initialization blocks here.
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      while (opModeIsActive()) {
        // Put loop blocks here.
        while (gamepad1.dpad_up) {
          right_drive.setPower(0.5);
          lift_motor.setDirection(DcMotorSimple.Direction.REVERSE);
          right_drive.setTargetPosition(20);
        }
        while (gamepad1.dpad_up) {
          right_drive.setPower(0.5);
          lift_motor.setDirection(DcMotorSimple.Direction.FORWARD);
          lift_motor.setTargetPosition(0);
        }
        telemetry.update();
      }
    }
  }
}
