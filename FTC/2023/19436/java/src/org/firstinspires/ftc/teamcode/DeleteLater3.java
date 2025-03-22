package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.List;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.JavaUtil;

@TeleOp(name = "DeleteLater3 (Blocks to Java)")
public class DeleteLater3 extends LinearOpMode {

  private DcMotor lift_motor;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    lift_motor = hardwareMap.get(DcMotor.class, "lift_motor");

    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      while (opModeIsActive()) {
        // Put loop blocks here.
        telemetry.update();
      }
    }
  }

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  private void runOpMode3() {
  }

  /**
   * Describe this function...
   */
  private List runOpMode2() {
    // Put initialization blocks here.
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      while (opModeIsActive()) {
        // Put loop blocks here.
        telemetry.update();
      }
    }
    return JavaUtil.createListWith(lift_motor.getPower(), lift_motor.getCurrentPosition(), null);
  }
}
