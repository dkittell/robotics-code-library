package org.firstinspires.ftc.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "HeadLightTest (Blocks to Java)")
@Disabled
public class HeadLightTest extends LinearOpMode {

  private DcMotor mArm;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    mArm = hardwareMap.get(DcMotor.class, "mArm");

    // Put initialization blocks here.
    waitForStart();
    if (opModeIsActive()) {
      mArm.setPower(1);
      sleep(50000);
      // Put run blocks here.
    }
  }
}
