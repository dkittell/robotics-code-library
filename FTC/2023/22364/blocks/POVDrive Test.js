// IDENTIFIERS_USED=gamepad1,left_driveAsDcMotor,right_driveAsDcMotor

/**
 * This function is executed when this Op Mode is selected from the Driver Station.
 */
function runOpMode() {
  // You will have to determine which motor to reverse for your robot.
  // In this example, the right motor was reversed so that positive
  // applied power makes it move the robot in the forward direction.
  left_driveAsDcMotor.setDirection("REVERSE");
  linearOpMode.waitForStart();
  if (linearOpMode.opModeIsActive()) {
    while (linearOpMode.opModeIsActive()) {
      // The Y axis of a joystick ranges from -1 in its topmost position
      // to +1 in its bottommost position. We negate this value so that
      // the topmost position corresponds to maximum forward power.
      left_driveAsDcMotor.setDualPower(-gamepad1.getLeftStickY() + gamepad1.getRightStickX(), right_driveAsDcMotor, -gamepad1.getLeftStickY() - gamepad1.getRightStickX());
      telemetry.addNumericData('Left Pow', left_driveAsDcMotor.getPower());
      telemetry.addNumericData('Right Pow', right_driveAsDcMotor.getPower());
      telemetry.update();
    }
  }
}
