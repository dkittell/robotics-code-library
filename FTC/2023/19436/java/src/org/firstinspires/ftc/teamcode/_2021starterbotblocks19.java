package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "_2021starterbotblocks19 (Blocks to Java)")
public class _2021starterbotblocks19 extends LinearOpMode {

  private DcMotor Arm;
  private DcMotor LeftDrive;
  private DcMotor RightDrive;
  private Servo DuckSpinner;
  private DcMotor Intake;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    double motorSpeed;

    Arm = hardwareMap.get(DcMotor.class, "Arm");
    LeftDrive = hardwareMap.get(DcMotor.class, "LeftDrive");
    RightDrive = hardwareMap.get(DcMotor.class, "RightDrive");
    DuckSpinner = hardwareMap.get(Servo.class, "DuckSpinner");
    Intake = hardwareMap.get(DcMotor.class, "Intake");

    // Reverse one of the drive motors.
    // You will have to determine which motor to reverse for your robot.
    // In this example, the right motor was reversed so that positive
    // applied power makes it move the robot in the forward direction.
    LeftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
    RightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
    DuckSpinner.setPosition(0);
    // Wait for the game to start (driver presses PLAY)
    waitForStart();
    if (opModeIsActive()) {
      // Run until end of the match
      while (opModeIsActive()) {
        // DRIVETRAIN CODE
        if (gamepad1.left_stick_button) {
          motorSpeed = 1;
        } else {
          motorSpeed = 0.7;
        }
        // The Y axis of a joystick ranges from -1 in its topmost position
        // to +1 in its bottommost position. We negate this value so that
        // the topmost position corresponds to maximum forward power.
        LeftDrive.setPower(-(motorSpeed * (gamepad1.left_stick_y + gamepad1.left_stick_x)));
        RightDrive.setPower(-(motorSpeed * (gamepad1.left_stick_y - gamepad1.left_stick_x)));
      }
      telemetry.update();
      // INTAKE CODE
      if (gamepad1.right_trigger > 0.5) {
        Intake.setPower(gamepad1.right_trigger);
      } else if (gamepad1.left_trigger > 0.5) {
        Intake.setPower(-gamepad1.left_trigger);
      } else {
        Intake.setPower(0);
      }
      telemetry.update();
      // ARM CODE
      if (gamepad1.a) {
        // On the ground for starting and intake
        Arm.setTargetPosition(0);
        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Arm.setPower(1);
      } else if (gamepad1.x) {
        // Low level on the goal
        Arm.setTargetPosition(120);
        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Arm.setPower(1);
      } else if (gamepad1.y) {
        // Mid level on the goal
        Arm.setTargetPosition(260);
        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Arm.setPower(1);
      } else if (gamepad1.b) {
        // High level on the goal
        Arm.setTargetPosition(410);
        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Arm.setPower(1);
      } else if (gamepad1.right_bumper) {
        // High level on the goal scoring backwards
        Arm.setTargetPosition(1420);
        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Arm.setPower(1);
      } else if (gamepad1.left_bumper) {
        // Mid level on the goal scoring backwards
        Arm.setTargetPosition(1570);
        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Arm.setPower(1);
      } else if (gamepad1.dpad_right) {
        // Duck spinner direction #1
        DuckSpinner.setPosition(0);
      } else if (gamepad1.dpad_left) {
        // Duck spinner direction #2
        DuckSpinner.setPosition(1);
      } else {
        // Duck spinner stop when not pressing buttons
        DuckSpinner.setPosition(0.5);
      }
      telemetry.update();
    }
  }
}
