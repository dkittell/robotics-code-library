package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "_Working_Encoder_Strafe_Right (Blocks to Java)")
public class _Working_Encoder_Strafe_Right extends LinearOpMode {

  private DcMotor FrontLeft;
  private DcMotor BackLeft;
  private DcMotor FrontRight;
  private DcMotor BackRight;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    int ecFR;
    double inFR;
    double ecPerInch;
    double ecStrafeRight01;
    double mP;
    int ecForward;
    int ecStrafeL;

    FrontLeft = hardwareMap.get(DcMotor.class, "Front Left");
    BackLeft = hardwareMap.get(DcMotor.class, "Back Left");
    FrontRight = hardwareMap.get(DcMotor.class, "Front Right");
    BackRight = hardwareMap.get(DcMotor.class, "Back Right");

    // Put initialization blocks here.
    FrontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
    BackLeft.setDirection(DcMotorSimple.Direction.FORWARD);
    FrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
    BackRight.setDirection(DcMotorSimple.Direction.REVERSE);
    FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    ecFR = 1120;
    inFR = 12.75;
    ecPerInch = ecFR / inFR;
    ecStrafeRight01 = ecPerInch * 25;
    mP = 0.5;
    ecForward = 6000;
    ecStrafeL = 150;
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      // Encoder Count Strafe Right - Start
      FrontLeft.setPower(mP);
      BackLeft.setPower(mP);
      FrontRight.setPower(mP);
      BackRight.setPower(mP);
      FrontLeft.setTargetPosition((int) ecStrafeRight01);
      BackLeft.setTargetPosition((int) -ecStrafeRight01);
      FrontRight.setTargetPosition((int) -ecStrafeRight01);
      BackRight.setTargetPosition((int) ecStrafeRight01);
      FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      while (!(!FrontLeft.isBusy() && !BackLeft.isBusy() && !FrontRight.isBusy() && !BackRight.isBusy())) {
      }
      sleep(1000);
      // Encoder Count Strafe Right - End
    }
  }
}
