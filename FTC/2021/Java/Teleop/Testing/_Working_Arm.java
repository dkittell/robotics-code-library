package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(name = "_Working_Arm")
public class _Working_Arm extends LinearOpMode {

  private DcMotor PivotMotor;
  private DcMotor ArmMotor;
  private TouchSensor TouchSensor_TouchSensor;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    PivotMotor = hardwareMap.get(DcMotor.class, "PivotMotor");
    ArmMotor = hardwareMap.get(DcMotor.class, "ArmMotor");
    TouchSensor_TouchSensor = hardwareMap.get(TouchSensor.class, "TouchSensor");

    // Put initialization blocks here.
    PivotMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    ArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    PivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    ArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      while (opModeIsActive()) {
        // Put loop blocks here.
        PivotMotor.setPower(gamepad2.left_stick_y);
        telemetry.addData("RightStickX", gamepad2.right_stick_y);
        if (gamepad2.right_stick_y < 0) {
          if (TouchSensor_TouchSensor.isPressed()) {
            ArmMotor.setPower(0);
          } else {
            ArmMotor.setPower(gamepad2.right_stick_y);
          }
          telemetry.update();
        } else {
          ArmMotor.setPower(gamepad2.right_stick_y);
        }
        telemetry.update();
      }
    }
  }
}
