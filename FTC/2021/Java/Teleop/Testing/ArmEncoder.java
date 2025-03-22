package org.firstinspires.ftc.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Disabled
@TeleOp(name = "ArmEncoder (Blocks to Java)")
public class ArmEncoder extends LinearOpMode {

  private DcMotor m_Arm;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    int eU;
    double inchL;
    double mP;
    double inchM;
    double inchH;
    double inchG;

    m_Arm = hardwareMap.get(DcMotor.class, "m_Arm");

    // Put initialization blocks here.
    m_Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    m_Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    m_Arm.setDirection(DcMotorSimple.Direction.REVERSE);
    eU = 637;
    // 637 x 1.9 = 13.75 in
    inchL = eU * 1.9;
    mP = 0.8;
    // 637 x 3.3 = 23.75 in
    inchM = eU * 3.3;
    // 637 x 4.8 = 34.0.3125 in
    inchH = eU * 4.8;
    // 637 x 0.3 =
    inchG = eU * 0.3;
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      while (opModeIsActive()) {
        if (gamepad1.a) {
          m_Arm.setPower(mP);
          m_Arm.setTargetPosition((int) inchL);
          m_Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
          sleep(1000);
        }
        if (gamepad1.b) {
          m_Arm.setPower(mP);
          m_Arm.setTargetPosition((int) inchM);
          m_Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
          sleep(1000);
        }
        if (gamepad1.x) {
          m_Arm.setPower(mP);
          m_Arm.setTargetPosition((int) inchH);
          m_Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
          sleep(5000);
        }
        if (gamepad1.y) {
          m_Arm.setPower(mP);
          m_Arm.setTargetPosition((int) inchG);
          m_Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
          sleep(5000);
        }
        if (gamepad1.right_bumper) {
          m_Arm.setPower(0);
        }
        telemetry.addData("Encoder's Elapsed", m_Arm.getCurrentPosition());
        telemetry.update();
      }
    }
  }
}
