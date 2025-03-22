package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(
  name = "EncoderCountDrive (Blocks to Java)",
  preselectTeleOp = "2022 Iterative"
)
public class EncoderCountDrive extends LinearOpMode {

  private DcMotor mFrontLeft;
  private DcMotor mBackLeft;
  private DcMotor mFrontRight;
  private DcMotor mBackRight;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    double MotorPower;
    double StartTime;

    mFrontLeft = hardwareMap.get(DcMotor.class, "mFrontLeft");
    mBackLeft = hardwareMap.get(DcMotor.class, "mBackLeft");
    mFrontRight = hardwareMap.get(DcMotor.class, "mFrontRight");
    mBackRight = hardwareMap.get(DcMotor.class, "mBackRight");

    // Set motor directions so robot will move forward.
    mFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    mBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    mFrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
    mBackRight.setDirection(DcMotorSimple.Direction.FORWARD);
    mFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    mBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    mFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    mBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    MotorPower = 0.5;
    // Wait for start to be pressed on the Driver Station
    waitForStart();
    mFrontLeft.setPower(MotorPower);
    mFrontRight.setPower(MotorPower);
    mBackLeft.setPower(MotorPower);
    mBackRight.setPower(MotorPower);
    StartTime = getRuntime();
    while (!(isStopRequested() || getRuntime() - StartTime > 3)) {
      // Let motors run
    }
    // Make sure the encoder tick counts are zero.
    mFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    mFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    mBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    mBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    // Reverse both motors relative to previous settings.
    if (!isStopRequested()) {
      mFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      mFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      mBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      mBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      mFrontLeft.setTargetPosition((int) (384.5 * 4));
      mFrontRight.setTargetPosition((int) (384.5 * 4));
      mBackLeft.setTargetPosition((int) (384.5 * 4));
      mBackRight.setTargetPosition((int) (384.5 * 4));
      mFrontLeft.setPower(MotorPower);
      mFrontRight.setPower(MotorPower);
      mBackLeft.setPower(MotorPower);
      mBackRight.setPower(MotorPower);
      // Loop until both motors are no longer busy.
      while (
        !(isStopRequested() || !mFrontLeft.isBusy() && !mFrontRight.isBusy())
      ) {
        // Let motors run
      }
      // We're done so we turn off the motors.
      mFrontLeft.setPower(0);
      mFrontRight.setPower(0);
      mBackLeft.setPower(0);
      mBackRight.setPower(0);
    }
  }
}
