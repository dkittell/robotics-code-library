// IDENTIFIERS_USED=ArmAsDcMotor,DuckSpinnerAsServo,gamepad1,IntakeAsDcMotor,LeftDriveAsDcMotor,RightDriveAsDcMotor

var motorSpeed;

/**
 * This function is executed when this Op Mode is selected from the Driver Station.
 */
function runOpMode() {
  // You will have to determine which motor to reverse for your robot.
  // In this example, the right motor was reversed so that positive
  // applied power makes it move the robot in the forward direction.
  LeftDriveAsDcMotor.setDirection("FORWARD");
  RightDriveAsDcMotor.setDirection("REVERSE");
  DuckSpinnerAsServo.setPosition(0);
  linearOpMode.waitForStart();
  if (linearOpMode.opModeIsActive()) {
    while (linearOpMode.opModeIsActive()) {
      if (gamepad1.getLeftStickButton()) {
        motorSpeed = 1;
      } else {
        motorSpeed = 0.7;
      }
      // The Y axis of a joystick ranges from -1 in its topmost position
      // to +1 in its bottommost position. We negate this value so that
      // the topmost position corresponds to maximum forward power.
      LeftDriveAsDcMotor.setDualPower(-(motorSpeed * (gamepad1.getLeftStickY() + gamepad1.getLeftStickX() * 0.75)), RightDriveAsDcMotor, -(motorSpeed * (gamepad1.getLeftStickY() - gamepad1.getLeftStickX() * 0.75)));
      telemetry.update();
      if (gamepad1.getRightTrigger() > 0.5) {
        IntakeAsDcMotor.setPower(0.25);
      } else if (gamepad1.getLeftTrigger() > 0.5) {
        IntakeAsDcMotor.setPower(-0.25);
      } else {
        IntakeAsDcMotor.setPower(0);
      }
      telemetry.update();
      if (gamepad1.getRightBumper()) {
        ArmAsDcMotor.setPower(0.5);
      } else if (gamepad1.getLeftBumper()) {
        ArmAsDcMotor.setPower(-0.5);
      } else {
        ArmAsDcMotor.setPower(0);
      }
    }
    if (gamepad1.getA()) {
      ArmAsDcMotor.setTargetPosition(0);
      ArmAsDcMotor.setMode("RUN_TO_POSITION");
      ArmAsDcMotor.setPower(1);
    } else if (gamepad1.getX()) {
      ArmAsDcMotor.setTargetPosition(120);
      ArmAsDcMotor.setMode("RUN_TO_POSITION");
      ArmAsDcMotor.setPower(1);
    } else if (gamepad1.getY()) {
      ArmAsDcMotor.setTargetPosition(260);
      ArmAsDcMotor.setMode("RUN_TO_POSITION");
      ArmAsDcMotor.setPower(1);
    } else if (gamepad1.getB()) {
      ArmAsDcMotor.setTargetPosition(410);
      ArmAsDcMotor.setMode("RUN_TO_POSITION");
      ArmAsDcMotor.setPower(1);
    } else if (gamepad1.getRightBumper()) {
      ArmAsDcMotor.setTargetPosition(1420);
      ArmAsDcMotor.setMode("RUN_TO_POSITION");
      ArmAsDcMotor.setPower(1);
    } else if (gamepad1.getLeftBumper()) {
      ArmAsDcMotor.setTargetPosition(1570);
      ArmAsDcMotor.setMode("RUN_TO_POSITION");
      ArmAsDcMotor.setPower(1);
    } else if (gamepad1.getDpadRight()) {
      DuckSpinnerAsServo.setPosition(0);
    } else if (gamepad1.getDpadLeft()) {
      DuckSpinnerAsServo.setPosition(1);
    } else {
      DuckSpinnerAsServo.setPosition(0.5);
    }
    telemetry.update();
  }
}
