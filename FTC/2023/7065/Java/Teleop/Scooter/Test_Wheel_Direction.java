package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Test_Wheel_Direction2 (Blocks to Java)", group = "Diagnostic")
public class Test_Wheel_Direction2 extends LinearOpMode {

  private DcMotor mFrontLeft;
  private DcMotor mBackLeft;
  private DcMotor mFrontRight;
  private DcMotor mBackRight;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    double mP;

    mFrontLeft = hardwareMap.get(DcMotor.class, "mFrontLeft");
    mBackLeft = hardwareMap.get(DcMotor.class, "mBackLeft");
    mFrontRight = hardwareMap.get(DcMotor.class, "mFrontRight");
    mBackRight = hardwareMap.get(DcMotor.class, "mBackRight");

    // Put initialization blocks here.
    // Motor Direction Defined - Start
    mFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    mBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    mFrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
    mBackRight.setDirection(DcMotorSimple.Direction.FORWARD);
    // Motor Direction Defined - End
    // Motor Zero Power Defined - Start
    mFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    mBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    mFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    mBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    // Motor Zero Power Defined - End
    mP = 0.2;
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      while (opModeIsActive()) {
        if (gamepad1.x) {
          mFrontLeft.setPower(mP);
        } else {
          mFrontLeft.setPower(0);
        }
        if (gamepad1.y) {
          mFrontRight.setPower(mP);
        } else {
          mFrontRight.setPower(0);
        }
        if (gamepad1.a) {
          mBackLeft.setPower(mP);
        } else {
          mBackLeft.setPower(0);
        }
        if (gamepad1.b) {
          mBackRight.setPower(mP);
        } else {
          mBackRight.setPower(0);
        }
        // Test Wheel Direction - End
      }
      // Test Wheel Direction - Start
    }
  }
}
