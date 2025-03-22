package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "DuckSpinner (Blocks to Java)")
public class DuckSpinner extends LinearOpMode {

  private DcMotor mDuck;
  private DcMotor mFrontLeft;
  private DcMotor mFrontRight;
  private DcMotor mBackLeft;
  private DcMotor mBackRight;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    mDuck = hardwareMap.get(DcMotor.class, "mDuck");
    mFrontLeft = hardwareMap.get(DcMotor.class, "mFrontLeft");
    mFrontRight = hardwareMap.get(DcMotor.class, "mFrontRight");
    mBackLeft = hardwareMap.get(DcMotor.class, "mBackLeft");
    mBackRight = hardwareMap.get(DcMotor.class, "mBackRight");

    // Put initialization blocks here.
    mDuck.setDirection(DcMotorSimple.Direction.REVERSE);
    mDuck.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    // Reset Encoders For Driving Auton
    mFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    mFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    mBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    mBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    // Set Motor Direction
    mFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    mFrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
    mBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    mBackRight.setDirection(DcMotorSimple.Direction.FORWARD);
    // Set Motors To Run With Encoders
    mFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    mFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    mBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    mBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    // Set Motor Braking
    mFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    mFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    mBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    mBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      mDuck.setPower(1);
      telemetry.update();
      
      // Encoder Drive Forward - Start
      // Set Motor Speed
      mFrontLeft.setPower(0.5);
      mFrontRight.setPower(0.5);
      mBackLeft.setPower(0.5);
      mBackRight.setPower(0.5);
      
      // Set Encoder Drive Target Position
      mFrontLeft.setTargetPosition(1120);
      mFrontRight.setTargetPosition(1120);
      mBackLeft.setTargetPosition(1120);
      mBackRight.setTargetPosition(1120);
      
      // Set the motors to run to the position
      mFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      mFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      mBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      mBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      telemetry.addData("Encoder Count - Left", mBackLeft.getCurrentPosition());
      telemetry.addData("Encoder Count - Right", mBackRight.getCurrentPosition());
      telemetry.update();
      while (!mFrontLeft.isBusy() && !mFrontRight.isBusy() && !mBackLeft.isBusy() && !mBackRight.isBusy()) {
      }
      sleep(1000);
      telemetry.update();
      // Encoder Drive Forward  - End
      
      sleep(1000);
      // Encoder Drive Backward - Start
      // Set Motor Speed
      mFrontLeft.setPower(0.5);
      mFrontRight.setPower(0.5);
      mBackLeft.setPower(0.5);
      mBackRight.setPower(0.5);
      // Set Encoder Drive Target Position
      mFrontLeft.setTargetPosition(-1120);
      mFrontRight.setTargetPosition(-1120);
      mBackLeft.setTargetPosition(-1120);
      mBackRight.setTargetPosition(-1120);
      // Set the motors to run to the position
      mFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      mFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      mBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      mBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      telemetry.addData("Encoder Count - Left", mBackLeft.getCurrentPosition());
      telemetry.addData("Encoder Count - Right", mBackRight.getCurrentPosition());
      telemetry.update();
      while (!mFrontLeft.isBusy() && !mFrontRight.isBusy() && !mBackLeft.isBusy() && !mBackRight.isBusy()) {
      }
      sleep(1000);
      telemetry.update();
      // Encoder Drive Backward - End
    }
  }
}
