package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "EncoderTest (Blocks to Java)", group = "")
public class EncoderTest extends LinearOpMode {
  private DcMotor FrontRight;
  private DcMotor BackRight;
  private DcMotor FrontLeft;
  private DcMotor BackLeft;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    double Gear;
    double Dpaduplast;
    double Dpaddownlast;

    FrontRight = hardwareMap.get(DcMotor.class, "Front Right");
    BackRight = hardwareMap.get(DcMotor.class, "Back Right");
    FrontLeft = hardwareMap.get(DcMotor.class, "Front Left");
    BackLeft = hardwareMap.get(DcMotor.class, "Back Left");

    // Put initialization blocks here.
    FrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
    BackRight.setDirection(DcMotorSimple.Direction.REVERSE);
    FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    Gear = 0.1;
    Dpaduplast = 0;
    Dpaddownlast = 0;
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      while (opModeIsActive()) {
        // Put loop blocks here.
        // Arcade style meccanum drive
        if (Dpaduplast == 0) {
          if (Gear < 0.5) {
            if (gamepad1.dpad_up) {
              Dpaduplast = 1;
              Gear += 0.1;
            }
          }
        } else if (!gamepad1.dpad_up) {
          Dpaduplast = 0;
        }
        if (Dpaddownlast == 0) {
          if (Gear > 0.11) {
            if (gamepad1.dpad_down) {
              Gear += -0.1;
              Dpaddownlast = 1;
            }
          }
        } else if (!gamepad1.dpad_down) {
          Dpaddownlast = 0;
        }
        telemetry.addData("Time Passed", getRuntime());
        FrontRight.setPower(
          (-gamepad1.left_stick_y * Gear - gamepad1.right_stick_x * Gear) -
          gamepad1.left_stick_x *
          Gear
        );
        BackRight.setPower(
          (-gamepad1.left_stick_y * Gear - gamepad1.right_stick_x * Gear) +
          gamepad1.left_stick_x *
          Gear
        );
        FrontLeft.setPower(
          -gamepad1.left_stick_y *
          Gear +
          gamepad1.right_stick_x *
          Gear +
          gamepad1.left_stick_x *
          Gear
        );
        BackLeft.setPower(
          (-gamepad1.left_stick_y * Gear + gamepad1.right_stick_x * Gear) -
          gamepad1.left_stick_x *
          Gear
        );
        telemetry.addData("Gear", Gear);
        telemetry.addData("Front Left Encoder", FrontLeft.getCurrentPosition());
        telemetry.addData(
          "Front Right Encoder",
          FrontRight.getCurrentPosition()
        );
        telemetry.addData("Back Left Encoder", BackLeft.getCurrentPosition());
        telemetry.addData("Back Right Encoder", BackRight.getCurrentPosition());
        telemetry.addData(
          "Front Left RPM",
          ((DcMotorEx) FrontLeft).getVelocity()
        );
        telemetry.addData(
          "Back Left RPM",
          ((DcMotorEx) BackLeft).getVelocity()
        );
        telemetry.addData(
          "Front Right RPM",
          ((DcMotorEx) FrontRight).getVelocity()
        );
        telemetry.addData(
          "Back Right RPM",
          ((DcMotorEx) BackRight).getVelocity()
        );
        telemetry.update();
      }
    }
  }
}
