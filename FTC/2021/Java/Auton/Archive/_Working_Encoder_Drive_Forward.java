// Blue Wheel Robot

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "_Working_Encoder_Drive_Forward (Blocks to Java)")
public class _Working_Encoder_Drive_Forward extends LinearOpMode {

  private DcMotor FrontLeft;
  private DcMotor BackLeft;
  private DcMotor FrontRight;
  private DcMotor BackRight;

  /**
   * This function is executed when this Op Mode is selected from the Driver
   * Station.
   */
  @Override
  public void runOpMode() {
    int ecFR;
    double inFR;
    int ecPerInch;
    double mP;

    ecFR = 1120;
    inFR = 12.75;
    ecPerInch = ecFR / (int) inFR;
    mP = 0.2;

    FrontLeft = hardwareMap.get(DcMotor.class, "mFrontLeft");
    BackLeft = hardwareMap.get(DcMotor.class, "mBackLeft");
    FrontRight = hardwareMap.get(DcMotor.class, "mFrontRight");
    BackRight = hardwareMap.get(DcMotor.class, "mBackRight");

    // Put initialization blocks here.
    FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    FrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
    BackRight.setDirection(DcMotorSimple.Direction.FORWARD);
    FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    ecForward = ecPerInch * 2;
    ecStrafeL = 150;
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      // Encoder Count Drive Forward - Start
      FrontLeft.setPower(mP);
      BackLeft.setPower(mP);
      FrontRight.setPower(mP);
      BackRight.setPower(mP);
      FrontLeft.setTargetPosition(ecForward);
      BackLeft.setTargetPosition(ecForward);
      FrontRight.setTargetPosition(ecForward);
      BackRight.setTargetPosition(ecForward);
      FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      while (!(!FrontLeft.isBusy() && !BackLeft.isBusy() && !FrontRight.isBusy() && !BackRight.isBusy())) {
      }
      sleep(1000);
      // Encoder Count Drive Forward - End
    }
  }
}
