package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "test5 (Blocks to Java)")
public class test5 extends LinearOpMode {

  private DcMotor right_drive;
  private DcMotor left_drive;
  private DcMotor intake_motor;
  private DcMotor lift_motor;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    right_drive = hardwareMap.get(DcMotor.class, "right_drive");
    left_drive = hardwareMap.get(DcMotor.class, "left_drive");
    intake_motor = hardwareMap.get(DcMotor.class, "intake_motor");
    lift_motor = hardwareMap.get(DcMotor.class, "lift_motor");

    // Reverse one of the drive motors.
    // You will have to determine which motor to reverse for your robot.
    // In this example, the right motor was reversed so that positive
    // applied power makes it move the robot in the forward direction.
    right_drive.setDirection(DcMotorSimple.Direction.REVERSE);
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      while (opModeIsActive()) {
        // Put loop blocks here.
        // The Y axis of a joystick ranges from -1 in its topmost position
        // to +1 in its bottommost position. We negate this value so that
        // the topmost position corresponds to maximum forward power.
        left_drive.setPower(-gamepad1.left_stick_y);
        right_drive.setPower(-gamepad1.right_stick_y);
        telemetry.addData("Left Pow", left_drive.getPower());
        telemetry.addData("Right Pow", right_drive.getPower());
        if (gamepad1.a) {
          intake_motor.setPower(1);
          intake_motor.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        if (gamepad1.b) {
          intake_motor.setPower(1);
          intake_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        if (gamepad1.x) {
          intake_motor.setPower(0);
        }
        while (gamepad1.dpad_up) {
          lift_motor.setPower(0.5);
          lift_motor.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        while (gamepad1.dpad_down) {
          lift_motor.setPower(0.5);
          lift_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        lift_motor.setDirection(DcMotorSimple.Direction.FORWARD);
        lift_motor.setPower(0.1);
        telemetry.update();
      }
    }
  }
}
