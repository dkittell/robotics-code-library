// IDENTIFIERS_USED=gamepad1,intake_motorAsDcMotor,left_driveAsDcMotor,lift_motorAsDcMotor,right_driveAsDcMotor

/**
 * This function is executed when this Op Mode is selected from the Driver Station.
 */
function runOpMode() {
  // You will have to determine which motor to reverse for your robot.
  // In this example, the right motor was reversed so that positive
  // applied power makes it move the robot in the forward direction.
  right_driveAsDcMotor.setDirection("REVERSE");
  linearOpMode.waitForStart();
  if (linearOpMode.opModeIsActive()) {
    while (linearOpMode.opModeIsActive()) {
      // The Y axis of a joystick ranges from -1 in its topmost position
      // to +1 in its bottommost position. We negate this value so that
      // the topmost position corresponds to maximum forward power.
      left_driveAsDcMotor.setDualPower(-gamepad1.getLeftStickY(), right_driveAsDcMotor, -gamepad1.getRightStickY());
      telemetry.addNumericData('Left Pow', left_driveAsDcMotor.getPower());
      telemetry.addNumericData('Right Pow', right_driveAsDcMotor.getPower());
      if (gamepad1.getA()) {
        intake_motorAsDcMotor.setPower(1);
        intake_motorAsDcMotor.setDirection("FORWARD");
      }
      if (gamepad1.getB()) {
        intake_motorAsDcMotor.setPower(1);
        intake_motorAsDcMotor.setDirection("REVERSE");
      }
      if (gamepad1.getX()) {
        intake_motorAsDcMotor.setPower(0);
      }
      while (gamepad1.getLeftBumper()) {
        lift_motorAsDcMotor.setPower(0.5);
        lift_motorAsDcMotor.setDirection("FORWARD");
      }
      while (gamepad1.getRightBumper()) {
        lift_motorAsDcMotor.setPower(0.5);
        lift_motorAsDcMotor.setDirection("REVERSE");
      }
      lift_motorAsDcMotor.setDirection("FORWARD");
      lift_motorAsDcMotor.setPower(0.1);
      telemetry.update();
    }
  }
}
