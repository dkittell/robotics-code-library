package org.firstinspires.ftc.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
@Disabled
@Autonomous(name = "Test_Wheel_Direction (Blocks to Java)")
public class Test_Wheel_Direction extends LinearOpMode {

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
    //mFrontLeft = 0;
    mBackLeft = hardwareMap.get(DcMotor.class, "mBackLeft");
    mFrontRight = hardwareMap.get(DcMotor.class, "mFrontRight");
    mBackRight = hardwareMap.get(DcMotor.class, "mBackRight");

    // Put initialization blocks here.
    mFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    mBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    mFrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
    mBackRight.setDirection(DcMotorSimple.Direction.FORWARD);
    mFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    mBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    mFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    mBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    mP = 0.2;
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      // Encoder Count Drive Forward - Start
      mFrontLeft.setPower(mP);
      mBackLeft.setPower(0);
      mFrontRight.setPower(0);
      mBackRight.setPower(0);
      sleep(1000);
      // Encoder Count Drive Forward - End
    }
  }
}
