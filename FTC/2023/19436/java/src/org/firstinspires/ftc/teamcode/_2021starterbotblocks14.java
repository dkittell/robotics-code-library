package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "_2021starterbotblocks14 (Blocks to Java)")
public class _2021starterbotblocks14 extends LinearOpMode {

  private DcMotor ArmAsDcMotor;
  private DcMotor LeftDriveAsDcMotor;
  private DcMotor RightDriveAsDcMotor;
  private Servo DuckSpinnerAsServo;
  private DcMotor IntakeAsDcMotor;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    ArmAsDcMotor = hardwareMap.get(DcMotor.class, "ArmAsDcMotor");
    LeftDriveAsDcMotor = hardwareMap.get(DcMotor.class, "LeftDriveAsDcMotor");
    RightDriveAsDcMotor = hardwareMap.get(DcMotor.class, "RightDriveAsDcMotor");
    DuckSpinnerAsServo = hardwareMap.get(Servo.class, "DuckSpinnerAsServo");
    IntakeAsDcMotor = hardwareMap.get(DcMotor.class, "IntakeAsDcMotor");

    // Reverse one of the drive motors.
    // You will have to determine which motor to reverse for your robot.
    // In this example, the right motor was reversed so that positive
    // applied power makes it move the robot in the forward direction.
    LeftDriveAsDcMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    RightDriveAsDcMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    DuckSpinnerAsServo.setPosition(0);
    // Wait for the game to start (driver presses PLAY)
    waitForStart();
    if (opModeIsActive()) {
      // Run until end of the match
      while (opModeIsActive()) {
        // DRIVETRAIN CODE
        // The Y axis of a joystick ranges from -1 in its topmost position
        // to +1 in its bottommost position. We negate this value so that
        // the topmost position corresponds to maximum forward power.
        LeftDriveAsDcMotor.setPower(-gamepad1.right_stick_y);
        RightDriveAsDcMotor.setPower(-gamepad1.left_stick_y);
      }
      telemetry.update();
      // INTAKE CODE
      if (gamepad1.right_trigger > 0.5) {
        IntakeAsDcMotor.setPower(gamepad1.right_trigger);
      } else if (gamepad1.left_trigger > 0.5) {
        IntakeAsDcMotor.setPower(-gamepad1.left_trigger);
      } else {
        IntakeAsDcMotor.setPower(0);
      }
      telemetry.update();
      // ARM CODE
      if (gamepad1.a) {
        // On the ground for starting and intake
        ArmAsDcMotor.setTargetPosition(0);
        ArmAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ArmAsDcMotor.setPower(1);
      } else if (gamepad1.x) {
        // Low level on the goal
        ArmAsDcMotor.setTargetPosition(120);
        ArmAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ArmAsDcMotor.setPower(1);
      } else if (gamepad1.y) {
        // Mid level on the goal
        ArmAsDcMotor.setTargetPosition(260);
        ArmAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ArmAsDcMotor.setPower(1);
      } else if (gamepad1.b) {
        // High level on the goal
        ArmAsDcMotor.setTargetPosition(410);
        ArmAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ArmAsDcMotor.setPower(1);
      } else if (gamepad1.right_bumper) {
        // High level on the goal scoring backwards
        ArmAsDcMotor.setTargetPosition(1420);
        ArmAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ArmAsDcMotor.setPower(1);
      } else if (gamepad1.left_bumper) {
        // Mid level on the goal scoring backwards
        ArmAsDcMotor.setTargetPosition(1570);
        ArmAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ArmAsDcMotor.setPower(1);
      } else if (gamepad1.dpad_right) {
        // Duck spinner direction #1
        DuckSpinnerAsServo.setPosition(0);
      } else if (gamepad1.dpad_left) {
        // Duck spinner direction #2
        DuckSpinnerAsServo.setPosition(1);
      } else {
        // Duck spinner stop when not pressing buttons
        DuckSpinnerAsServo.setPosition(0.5);
      }
      telemetry.update();
    }
  }
}
