package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "armTest")
public class armTest extends LinearOpMode {

  private DcMotor mLinear2;
  private DcMotor mPivot;
  private DcMotor mLinear1;

  /**
   * This function is executed when this OpMode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    mLinear2 = hardwareMap.get(DcMotor.class, "mLinear2");
    mPivot = hardwareMap.get(DcMotor.class, "mPivot");
    mLinear1 = hardwareMap.get(DcMotor.class, "mLinear1");

    // Put initialization blocks here.
    mLinear2.setDirection(DcMotor.Direction.REVERSE);
    mPivot.setDirection(DcMotor.Direction.REVERSE);
    mLinear1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    mLinear2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    mPivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      while (opModeIsActive()) {
        // Put loop blocks here.
        if (gamepad1.left_stick_y > 0.17) {
          mLinear1.setPower(0.7);
          mLinear2.setPower(0.7);
        } else if (gamepad1.left_stick_y < -0.17) {
          mLinear1.setPower(-0.7);
          mLinear2.setPower(-0.7);
        } else {
          mLinear1.setPower(0);
          mLinear2.setPower(0);
        }
        if (gamepad1.right_stick_y > 0.17) {
          mPivot.setPower(0.7);
        } else if (gamepad1.right_stick_y < -0.17) {
          mPivot.setPower(-0.7);
        } else {
          mPivot.setPower(0);
        }
        telemetry.update();
      }
    }
  }
}
