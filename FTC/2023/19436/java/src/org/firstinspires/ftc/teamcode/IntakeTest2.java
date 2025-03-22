package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "IntakeTest2 (Blocks to Java)")
public class IntakeTest2 extends LinearOpMode {

  private DcMotor intake_motor;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    intake_motor = hardwareMap.get(DcMotor.class, "intake_motor");

    // Put initialization blocks here.
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      while (opModeIsActive()) {
        // Put loop blocks here.
        intake_motor.setPower(1);
        telemetry.addData("Intake Power", intake_motor.getPower());
        telemetry.update();
      }
    }
  }
}
